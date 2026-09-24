#include <deque>

#define RXD2 16 // Pin físico RX del UART2 (Hardware Serial) para conectar al RYLR998
#define TXD2 17 // Pin físico TX del UART2

// --- DIRECCIONAMIENTO DE RED LoRa ---
// Aísla el tráfico para no colisionar con otras arquitecturas en el mismo espectro físico.
#define BASE_ADDRESS  2
#define ROBOT_ADDRESS 1
#define NETWORK_ID    5

// -------------------------------------------------------
//  PARÁMETROS DE CONTROL DE FLUJO (ARQ Protocol)
// -------------------------------------------------------
const long    COMMAND_TIMEOUT_MS = 5000;   // Ventana de tiempo máxima (5s) esperando Acuse de Recibo (ACK) antes de retransmitir
const int     MAX_RETRIES        = 3;      // Umbral de descarte (Drop): Es preferible fallar rápido y saltar al siguiente comando que acumular latencia crítica
const long    POLL_INTERVAL_MS   = 2000;   // Cadencia base de telemetría (2s) para refrescar el Dashboard
const long    INTER_PACKET_MS    = 200;    // Pausa inter-trama: Permite la descarga del condensador y estabilización térmica del chip LoRa

// -------------------------------------------------------
//  ESTADO DEL SISTEMA (Máquina de Estados Finita)
// -------------------------------------------------------
// Buffer dinámico FIFO para encolar tramas provenientes de Node-RED
std::deque<String> commandQueue;

// El gateway no puede enviar a lo ciego, el aire es un medio semidúplex (Half-Duplex)
enum State { STATE_IDLE, STATE_WAITING_FOR_ACK };
State currentState = STATE_IDLE;

String        pendingCmd    = ""; // Copia en caché del comando en vuelo para posibles retransmisiones
int           retryCount    = 0;
unsigned long lastSentTime  = 0; // Timestamp de la última inyección al aire
unsigned long lastPollTime  = 0; // Timestamp de la última solicitud de métricas espaciales

// -------------------------------------------------------
void setup() {
  // Ensanchamiento del buffer serial nativo de Arduino para evitar desbordamientos (Overrun) 
  // cuando Node-RED inyecta lotes masivos de waypoints de golpe.
  Serial.setRxBufferSize(1024);
  Serial.begin(115200); // Interfaz Puente Serial -> USB -> Servidor Fog (Node-RED)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Interfaz Periférico -> Transceptor LoRa RYLR998

  delay(1000); // Tiempo de arranque físico (Cold boot delay)
  Serial.println("{\"log\":\"Base v8 - POLL pausado durante ACK, fix manuales\"}");

  // Aprovisionamiento del módem LoRa mediante comandos AT
  Serial2.println("AT+ADDRESS=" + String(BASE_ADDRESS));
  delay(200); // Delay necesario para escritura en EEPROM interna del módulo
  Serial2.println("AT+NETWORKID=" + String(NETWORK_ID));
  delay(200);

  currentState = STATE_IDLE;
  
  // Handshake inicial: Informa al motor Fog (Node-RED) que la antena está lista para transmitir
  Serial.println("BASEREADY");
}

// -------------------------------------------------------
void loop() {

  // ── 1. RECEPCIÓN DESDE NODO SUPERIOR (Fog / Node-RED) ────────────────────────────
  // Escucha el puerto USB Serial. Si entra un payload válido, lo empuja a la cola FIFO.
  if (Serial.available()) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim(); // Sanitización: Limpieza de caracteres de retorno de carro
    if (incoming.length() > 0 && incoming.length() <= 240) { // Límite físico del payload del RYLR998 (240 bytes)
      commandQueue.push_back(incoming);
      Serial.println("{\"log\":\"Encolado: " + incoming +
                     " (Total: " + String(commandQueue.size()) + ")\"}");
    }
  }

  // ── 2. DESPACHADOR (Ejecución de Tráfico) ───────────────────────
  // Extrae y dispara un comando aéreo solo si el canal no está bloqueado esperando un ACK anterior.
  if (currentState == STATE_IDLE && !commandQueue.empty()) {
    delay(INTER_PACKET_MS); // Rampa de aceleración de tráfico para no saturar al receptor
    String nextCmd = commandQueue.front();
    commandQueue.pop_front(); // Desencola destructivamente
    sendWithRetry(nextCmd, true);
  }

  // ── 3. AUTO-POLLING TELEMÉTRICO ──────────────────────────────────
  // Rutina en background para forzar al robot a escupir sus coordenadas GPS actuales.
  // FIX BUG 2: Prevención de Colisión de Canal (CSMA por software).
  // Solo se lanza un "POLL" si no hay tráfico pesado (waypoints) encolado y han pasado 
  // 2.5s (2500ms) desde la última transmisión dura para dejar respirar la banda.
  if (currentState == STATE_IDLE &&
      commandQueue.empty() &&
      (millis() - lastPollTime > POLL_INTERVAL_MS) &&
    (millis() - lastSentTime  > 2500)) {
    String pollCmd = "AT+SEND=" + String(ROBOT_ADDRESS) + ",4,POLL";
    Serial2.println(pollCmd);
    lastPollTime = millis();
  }

  // ── 4. ESCUCHA DE BANDA LORA (Recepción Inalámbrica) ──────────────────────────────────
  // Verifica si el hardware de radio atrapó un paquete dirigido a nuestra dirección (BASE_ADDRESS)
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("+RCV=")) handleLoraReceive(msg); // Desvía al analizador léxico (Parser)
  }

  // ── 5. RUTINA DE TIMEOUT Y REINTENTO (ARQ) ────────────────────────────
  // Monitorea si el tiempo de espera por un ACK ha expirado (Robot fuera de rango o paquete colisionado)
  if (currentState == STATE_WAITING_FOR_ACK &&
      (millis() - lastSentTime > COMMAND_TIMEOUT_MS)) {
    retryCount++;
    if (retryCount <= MAX_RETRIES) {
      // Intento iterativo: Reinyecta exactamente la misma trama al aire
      Serial.println("{\"log\":\"TIMEOUT reintento " + String(retryCount) +
                     "/" + String(MAX_RETRIES) +
                     " cmd=" + pendingCmd + "\"}");
      executeSend(pendingCmd);
    } else {
      // Umbral superado: Falla dura (Hard Fail). Descartar el paquete (Drop) y desatascar el sistema
      Serial.println("{\"error\":\"DROPPED: " + pendingCmd + "\"}");
      pendingCmd   = "";
      retryCount   = 0;
      currentState = STATE_IDLE; // Liberar candado
      if (commandQueue.empty()) Serial.println("BASEREADY"); // Informar a Fog que vuelva a inyectar comandos
    }
  }
}

// -------------------------------------------------------
// Envío crudo al hardware a través del protocolo AT propietario de REYAX
void executeSend(String cmd) {
  String at = "AT+SEND=" + String(ROBOT_ADDRESS) + "," +
              String(cmd.length()) + "," + cmd;
  Serial2.println(at);
  lastSentTime = millis(); // Registrar marca de tiempo exacta para calcular el timeout
  pendingCmd   = cmd; // Respaldar payload
  currentState = STATE_WAITING_FOR_ACK; // Cerrar el candado de estado (Bloqueo de transmisión)
}

// Wrapper para inyección inicial reseteando la cuenta de errores
void sendWithRetry(String cmd, bool resetCounter) {
  if (resetCounter) retryCount = 0;
  executeSend(cmd);
}

// -------------------------------------------------------
// Determinador Booleano de Teleoperación
bool esComandoManual(String cmd) {
  // Filtra comandos de maniobra cinemática directa (F=Forward, B=Backward, L=Left, R=Right, S=Stop)
  return (cmd.length() == 1 &&
          (cmd == "F" || cmd == "B" || cmd == "L" ||
           cmd == "R" || cmd == "S"));
}

// -------------------------------------------------------
// Analizador Léxico (Parser) de recepción inalámbrica
// Desempaqueta la trama cruda AT (+RCV=Address,Length,Data,RSSI,SNR)
void handleLoraReceive(String loraMsg) {
  // Navegación por comas (Aritmética de índices)
  int c1 = loraMsg.indexOf(',');           if (c1 == -1) return; // Validación de integridad
  int c2 = loraMsg.indexOf(',', c1 + 1);  if (c2 == -1) return;
  int lastC = loraMsg.lastIndexOf(',');
  int rssiC = loraMsg.lastIndexOf(',', lastC - 1);
  if (rssiC == -1 || rssiC <= c2) return;

  // Extracción pura de Carga Útil (Payload) y Nivel de Señal (RSSI)
  String payload = loraMsg.substring(c2 + 1, rssiC);
  String rssi    = loraMsg.substring(rssiC + 1, lastC);

  // ── A. Flujo de Telemetría GPS ────────────────────────────────────
  // Si la trama responde al AUTO-POLL de estado
  if (payload.indexOf("LAT:") != -1) {
    String lat_s = getValue(payload, "LAT");
    String lon_s = getValue(payload, "LON");
    String hdg_s = getValue(payload, "HDG");
    
    // Serialización JSON para empaquetado directo hacia Node-RED
    String json  = "{\"lat\":" + lat_s + ",\"lon\":" + lon_s +
                   ",\"heading\":" + hdg_s + ",\"rssi\":" + rssi + "}";
    Serial.println(json);
    return; // Retorno temprano para no evaluar este payload como un ACK
  }

  // ── B. Evaluación de Acuse de Recibo (ACK) ───────────────────────────
  // Diccionario condicional de comandos válidos de confirmación de estado
  bool esAck =
      payload == "MSG:RESET_OK"           ||
      payload.startsWith("MSG:WPT_OK_")   ||
      payload.startsWith("MSG:COUNT=")    ||
      payload == "MSG:NAV_STARTED"        ||
      payload == "MSG:STOPPED"            ||
      payload == "MSG:EVASION_ON"         ||   
      payload == "MSG:EVASION_OFF"        ||   
      payload == "ACK:CMD"                ||   // ← ACK para maniobras manuales cinemáticas (F,B,L,R,S)
      payload.startsWith("ACK:GPSF_")     ||   
      payload == "ACK:KP"                 ||   
      payload == "ACK:KD"                 ||
      payload == "ACK:XTEKP"              ||   
      payload == "ACK:XTEMAX"             ||
      payload == "ACK:DECL"               ||   
      payload.startsWith("ACK:");

  if (esAck) {
    Serial.println(payload);  // Passthrough: Enrutado incondicional hacia Node-RED

    // RESOLUCIÓN DE CONDICIONES DE CARRERA LÓGICAS (Race Conditions):
    // FIX BUG 3: Una orden MSG:STOPPED puede surgir asíncronamente desde el RTOS del robot 
    // cuando finaliza su ruta autónoma. Esto NO debe interpretarse como un ACK a un CMD:STOP 
    // si el Gateway jamás mandó un CMD:STOP.
    // FIX BUG 1: Correlación genérica (ACK:CMD) libera el candado de cualquier movimiento manual.
    bool isMatch =
        (pendingCmd.startsWith("CMD:RESET") && payload == "MSG:RESET_OK")          ||
        (pendingCmd.startsWith("WPT:")      && payload.startsWith("MSG:WPT_OK_"))  ||
        (pendingCmd.startsWith("CMD:CHECK") && payload.startsWith("MSG:COUNT="))   ||
        (pendingCmd.startsWith("CMD:START") && payload == "MSG:NAV_STARTED")       ||
        (pendingCmd.startsWith("CMD:STOP")  && payload == "MSG:STOPPED")           ||
        (pendingCmd.startsWith("CMD:EVASION_ON")  && payload == "MSG:EVASION_ON")  ||
        (pendingCmd.startsWith("CMD:EVASION_OFF") && payload == "MSG:EVASION_OFF") ||
        (esComandoManual(pendingCmd)        && payload == "ACK:CMD")               ||
        payload.startsWith("ACK:");   // Comodín: Ajustes paramétricos siempre liberan

    // Si el payload entrante corresponde a la solicitud en espera
    if (isMatch) {
      Serial.println("{\"log\":\"ACK liberado: " + pendingCmd + "\"}");
      pendingCmd   = ""; // Purgar caché de retransmisión
      retryCount   = 0;
      currentState = STATE_IDLE; // ABRIR CANDADO: La máquina de estados permite enviar el próximo paquete
      lastPollTime = millis(); // Reiniciar timer de POLL para asegurar ancho de banda a los comandos
      
      // Si ya no quedan paquetes en la RAM del Gateway, notificar al Fog
      if (commandQueue.empty()) Serial.println("BASEREADY");
    }
    // Si MSG:STOPPED llegó de forma espontánea (Robot terminó ruta) y isMatch es false,
    // se ignora la liberación del estado pero el mensaje ya fue despachado al servidor.
    return;
  }

  // ── C. Flujo de Logística de Navegación Autónoma ────────────
  // El nodo remoto avisa asíncronamente cuando el algoritmo cinemático intercepta una coordenada válida.
  if (payload.startsWith("MSG:WPT_REACHED_")) {
    Serial.println(payload); // Despacho a Fog
    return;
  }

  // ── D. Flujo de Error/Debug Estándar ───────────────────────────────────
  if (payload.startsWith("MSG:") || payload.startsWith("ERR:")) {
    Serial.println(payload);
    return;
  }

  // Fallback: Si el texto no cumple ningún formato, encapsular en JSON preventivo para no romper la consola.
  Serial.println("{\"robot\":\"" + payload + "\"}");
}

// -------------------------------------------------------
// Helper Matemático de Extracción (String Slicing)
// Extrae un valor numérico basándose en la posición indexada de la etiqueta (key)
String getValue(String data, String key) {
  int ki = data.indexOf(key + ":");
  if (ki == -1) return "0"; // Valor nulo en caso de corrupción de paquete
  int vi = ki + key.length() + 1; // Puntero inicial del valor numérico
  int ci = data.indexOf(',', vi); // Puntero delimitador final
  if (ci == -1) return data.substring(vi); // Caso especial: Último valor del string sin coma final
  return data.substring(vi, ci);
}