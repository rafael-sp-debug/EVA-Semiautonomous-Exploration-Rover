#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include "esp_task_wdt.h"
#include <LittleFS.h>
#define LORA_FREQ 433E6
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0
#define WDT_TIMEOUT 10   // seconds

//////////////////////////////////////////////////////////////////////////////////////////////////// HTML embebido
const char index_html[] PROGMEM = R"rawliteral(

)rawliteral";

//////////////////////////////////////////////////////////////////////////////////////////////////// JS embebido
const char main_js[] PROGMEM = R"rawliteral(

)rawliteral";

const char* apSSID = "EVA_Dashboard";
const char* apPassword = "VOYAGER21"; //min 8 caracteres

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

uint8_t controllerClient = 255; // 255 = no hay controlador aún

String imgBuffer = "";
String webImage = "";

TaskHandle_t watchdogTaskHandle = NULL;

int expectedChunks = 0;
int receivedChunks = 0;
bool receivingImage = false;

String msgToSend = "";                    // Fila de mensaje de monitor serial
bool msgQueued = false;

unsigned long retryInterval = 100;        // Default LoRa SHORT: ms entre intervalos de envío
unsigned long ackTimeout = 500;           // Default LoRa MID: 500ms espera en reintento
unsigned long chunkTimeout = 1500;        // Default LoRa MID: 1.5 segundos para reenviar solicitud de chunk

unsigned long msgStartTime = 0;           // Cuando el mensaje fue enviado
unsigned long lastSendAttempt = 0;        // Cuando el mensaje se intentó enviar
unsigned long lastChunkRequestTime = 0;   // Cuando el chunk se solicitó por última vez

int expectedLength = 200;
float csvInt = 1.5;
int nemaStep = 3740;
String currGoal = "";

// Estados LoRa
enum LoRaRange { SHORT, MID, LONG };
LoRaRange currentRange = SHORT;  // Iniciar SHORT

// RSSI Umbral
const int shortToMid = -65;   // SHORT -> MID
const int midToShort  = -45;  // MID -> SHORT
const int midToLong   = -110; // MID -> LONG
const int longToMid   = -90;  // LONG -> MID

// Promedio RSSI
const int rssiSampleCount = 4;    // Muestras en promediado
int rssiSamples[rssiSampleCount] = {0};
int sampleIndex = 0;
int sampleTotal = 0;
int sampleFilled = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!LittleFS.begin()) {
    Serial.println("ERR_LittleFS");
  }
  
  const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);

  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_LOOP_ENABLED");

  xTaskCreatePinnedToCore(
    watchdogTask,           // task function
    "watchdogTask",         // name
    4096,                   // stack size
    NULL,                   // parameter
    1,                      // priority
    &watchdogTaskHandle,    // task handle
    0                       // core 0 (low priority)
  );

  // Start AP
  esp_log_level_set("wifi", ESP_LOG_NONE);
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\nConectar a Dashboard: http://");
  Serial.println(IP);

  // Serve HTML
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  // Serve JS
  server.on("/main.js", HTTP_GET, []() {
    server.send_P(200, "application/javascript", main_js);
  });

  server.on("/chart.umd.min.js", HTTP_GET, []() {
    File file = LittleFS.open("/chart.umd.min.js", "r");
    if (!file) {
      server.send(404, "text/plain", "chart.umd.min.js not found");
      return;
    }
    server.streamFile(file, "application/javascript");
    file.close();
  });


  server.begin();

  // WebSocket - Servidor
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(7); // Spreading Factor
  LoRa.setSignalBandwidth(250E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(6); // Preamble:
  LoRa.enableCrc(); // CRC

  Serial.println("Voyager21: Comunicación LoRa Habillitada");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  unsigned long now = millis();

  esp_task_wdt_reset();

  webSocket.loop();
  server.handleClient();

  // Recepción LORA
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) received += (char)LoRa.read();
    received.trim();

    // Filtrar basura
    bool valid = true;
    for (unsigned int i = 0; i < received.length(); i++) {
      char c = received[i];
      if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
        valid = false;
        break;
      }
    }

    if (!valid) {
      Serial.println("NONSENSE_RECEIVED");
      return;  // Descartar antes de actualizar RSSI
    }
    
    // Obtener RSSI
    int rssi = LoRa.packetRssi();  // RSSI de mensaje entrante
    sampleTotal -= rssiSamples[sampleIndex];    // Quitar T4
    rssiSamples[sampleIndex] = rssi;            // Guardar T0
    sampleTotal += rssiSamples[sampleIndex];    // Sumar al total

    sampleIndex = (sampleIndex + 1) % rssiSampleCount;    // Circular buffer
    if (sampleFilled < rssiSampleCount) sampleFilled++;

    int avgRssi = sampleTotal / sampleFilled;  // Promediar

    Serial.println("  RECV_ROVER_" + String(rssi) + "," + String(avgRssi) + "," + received);

    if (msgQueued && received.startsWith("ACK_")) {
      String ackFor = received.substring(4);
      if (ackFor == msgToSend) {
          msgQueued = false;
          msgToSend = "";

          if (ackFor == "SRA") LoRaShort();
          else if (ackFor == "MRA") LoRaMid();
          else if (ackFor == "LRA") LoRaLong();

          else if (receivingImage && ackFor.startsWith("REQ_")) lastChunkRequestTime = now;
      }
        
      if (ackFor.startsWith("CK")) {
        String lenCK = ackFor.substring(2);
        int newlen = lenCK.toInt();
        if (newlen > 0) expectedLength = newlen;
        Serial.println("  SET_CHUNK_SIZE_" + String(expectedLength));
        webSocket.broadcastTXT("CK_" + String(expectedLength));
      }

      else if (ackFor.startsWith("INT")) {
        String lenINT = ackFor.substring(3);
        float newInt = lenINT.toFloat();
        if (newInt > 0) csvInt = newInt;
        Serial.println("  SET_CSV_INTERVAL_" + String(csvInt));
        webSocket.broadcastTXT("CSVINT_" + String(csvInt));
      }

      else if (ackFor.startsWith("SET")) {
        String lenSTEP = ackFor.substring(3);
        int newStep = lenSTEP.toInt();
        if (newStep > 0) nemaStep = newStep;
        Serial.println("  SET_STEP_SIZE_" + String(nemaStep));
        webSocket.broadcastTXT("NEMA_" + String(nemaStep));
      }

      else if (ackFor.startsWith("GOAL")) {
        String lenGOAL = ackFor.substring(4);
        currGoal = lenGOAL;
        Serial.println("  GOAL_UPDATED" + String(currGoal));
        webSocket.broadcastTXT("GOAL_" + String(currGoal));
      }
    }

    // Cambio dinámico dependiendo de Thresholds
    switch (currentRange) {
        case SHORT:
            if (avgRssi < shortToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
            }
            break;

        case MID:
            if (avgRssi > midToShort) {
                Serial.println("AUTOSWITCH_SHORT");
                queueMessage("SRA");
            } else if (avgRssi < midToLong) {
                Serial.println("AUTOSWITCH_LONG");
                queueMessage("LRA");
            }
            break;

        case LONG:
            if (avgRssi > longToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
            }
            break;
    }

    // Tamaño de Imagen recibido, iniciar modo recepción de Imagen
    if (received.startsWith("IMG_SIZE")) {
      expectedChunks = received.substring(received.indexOf(',') + 1).toInt();
      receivedChunks = 0;
      imgBuffer = "";
      webImage = "";
      receivingImage = true;
      Serial.printf("EXPECTING_%d_CHUNKS\n", expectedChunks);
      webSocket.broadcastTXT("IMG_START");

      // Solicitar Chunk 1
      msgToSend = "REQ_1";
      msgQueued = true;
      lastSendAttempt = now;
      msgStartTime = now;
      lastChunkRequestTime = now;
    }

    else if (received.startsWith("M,")) {
      webSocket.broadcastTXT(received);
    }

    else if (received.indexOf(',') != -1) {
      String webTelemetry = String(rssi) + "," + String(avgRssi) + "," + received;
      webSocket.broadcastTXT(webTelemetry);
    }

    // Evaluación de Chunk entrante
    else if (received.startsWith("C_")) {
      if (receivingImage) {
        String chunkData = received.substring(2);

        bool lastChunk = (receivedChunks == expectedChunks - 1);

        bool b64OK = true;
        for (int i = 0; i < chunkData.length(); i++) {
          char c = chunkData[i];
          if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=')) {
            b64OK = false;
            break;
          }
        }

        bool lengthOK = (lastChunk && chunkData.length() <= expectedLength) ||
                        (chunkData.length() == expectedLength);

        if (lengthOK && b64OK) {
          imgBuffer += chunkData;
          receivedChunks++;
          Serial.printf("  RECV_CHUNK_%d/%d\n", receivedChunks, expectedChunks);

          if (receivedChunks < expectedChunks) {
            msgToSend = "REQ_" + String(receivedChunks + 1);
            msgQueued = true;
            lastSendAttempt = now;
            msgStartTime = now;
          } else {
            receivingImage = false;
            Serial.printf("IMG_RECEPTION_COMPLETE_%d\n", receivedChunks);
            Serial.println("B64_IMAGE_START");
            Serial.println(imgBuffer);
            webImage = imgBuffer;
            Serial.println("B64_IMAGE_END");
            imgBuffer = "";
            webSocket.broadcastTXT("IMG_DONE");
          }
        } else {
          Serial.printf("CHUNK_%d_INVALID(len=%d, b64=%s, last=%d)\n",
                        receivedChunks + 1, chunkData.length(),
                        b64OK ? "OK" : "FAIL", lastChunk);
          msgToSend = "REQ_" + String(receivedChunks + 1);
          msgQueued = true;
          lastSendAttempt = now;
          msgStartTime = now;
          lastChunkRequestTime = now;
        }
      }
    }
  }

  // REINTENTAR mensaje en cola con Timeout
  if (msgQueued) {
      // Mandar y reintentar
      if (now - lastSendAttempt >= retryInterval && LoRa.beginPacket()) {
          LoRa.print(msgToSend);
          LoRa.endPacket();
          lastSendAttempt = now;

          // Comenzar timeout al mandar primera vez
          if (msgStartTime == 0) msgStartTime = now;

          Serial.println("SENT_EST_" + msgToSend);
      }

      // Timeout de no recibir ACK
      if (msgStartTime > 0 && now - msgStartTime >= ackTimeout) {
          Serial.println("MSG_TIMEOUT");
          msgQueued = false;
          msgToSend = "";
          msgStartTime = 0;
      }
  }

  // Timeout de Chunk, Resolicitar
  if (receivingImage && !msgQueued && (now - lastChunkRequestTime >= chunkTimeout)) {
    Serial.printf("CHUNK_%d_TIMEOUT_REREQUESTING\n", receivedChunks + 1);
    msgToSend = "REQ_" + String(receivedChunks + 1);
    msgQueued = true;
    lastSendAttempt = now;
    msgStartTime = now;
    lastChunkRequestTime = now;
  }
  
  // Mensajes Serial
  handleSerial();
}

void handleSerial() {
  while (Serial.available()) {
    String serialInput = Serial.readStringUntil('\n');
    serialInput.trim();

    if (serialInput.length() == 0) continue;

    if (serialInput.equalsIgnoreCase(".COMLIST")) {
      printCommandList();
    }

    if (serialInput.equalsIgnoreCase(".RESETSTA")) {
      delay(200);
      ESP.restart();
    }

    else if (serialInput.startsWith(".CHUNK")) {
      String setChunksStr = serialInput.substring(6);
      setChunksStr.trim();
      int setChunks = setChunksStr.toInt();

      if (setChunks > 0 && setChunks < 201) {
        expectedLength = setChunks;
        queueMessage("CK" + String(setChunks));
      } else {
        Serial.println("CHUNK_SIZE_INVALID");
      }
    }
    else if (serialInput.startsWith(".INTERVAL")) {
      String intervalStr = serialInput.substring(9);
      intervalStr.trim();
      float intervalVal = intervalStr.toFloat();

      if (intervalVal > 0) {
        queueMessage("INT" + String(intervalVal));
      } else {
        Serial.println("CSV_INTERVAL_INVALID");
      }
    }
    else if (serialInput.startsWith(".STEP")) {
      String stepsStr = serialInput.substring(5);
      stepsStr.trim();
      int stepsVal = stepsStr.toInt();

      if (stepsVal > 0) {
        queueMessage("SET" + String(stepsVal));
      } else {
        Serial.println("STEP_SIZE_INVALID");
      }
    }

    else if (serialInput.equalsIgnoreCase(".FORCESHORT")) {
      LoRaShort();
    }
    else if (serialInput.equalsIgnoreCase(".FORCEMID")) {
      LoRaMid();
    }
    else if (serialInput.equalsIgnoreCase(".FORCELONG")) {
      LoRaLong();
    }
    else if (serialInput.equalsIgnoreCase(".CANCEL")) {
      // Detener recepción y limpiar buffer
      cancelImageTransfer();
    }
    else if (serialInput.equalsIgnoreCase(".W")) queueMessage("W");
    else if (serialInput.equalsIgnoreCase(".S")) queueMessage("S");
    else if (serialInput.equalsIgnoreCase(".A")) queueMessage("A");
    else if (serialInput.equalsIgnoreCase(".D")) queueMessage("D");
    else if (serialInput.equalsIgnoreCase(".IMAGE")) queueMessage("IMG");
    else if (serialInput.equalsIgnoreCase(".SHORT")) queueMessage("SRA");
    else if (serialInput.equalsIgnoreCase(".MID")) queueMessage("MRA");
    else if (serialInput.equalsIgnoreCase(".LONG")) queueMessage("LRA");
    else if (serialInput.equalsIgnoreCase(".START")) queueMessage("GO");
    else if (serialInput.equalsIgnoreCase(".GO")) queueMessage("GO");
    else if (serialInput.equalsIgnoreCase(".STOP")) queueMessage("STP");
    else if (serialInput.equalsIgnoreCase(".NOSTEP")) queueMessage(".NOSTEP");

    else if (serialInput.startsWith(".OBJ")) {
      String params = serialInput.substring(4);
      params.trim();
      int sp = params.indexOf(' ');
      if (sp > 0) {
        String xStr = params.substring(0, sp);
        String yStr = params.substring(sp + 1);
        xStr.trim();
        yStr.trim();
        queueMessage("OBJ" + xStr + "," + yStr);
      } else {
        Serial.println("ERROR_OBJ_FORMAT");
      }
    }
    else if (serialInput.startsWith(".GOAL")) {
      String params = serialInput.substring(5);
      params.trim();
      int sp = params.indexOf(' ');
      if (sp > 0) {
        String xStr = params.substring(0, sp);
        String yStr = params.substring(sp + 1);
        xStr.trim();
        yStr.trim();
        queueMessage("GOAL" + xStr + "," + yStr);
      } else {
        Serial.println("ERROR_GOAL_FORMAT");
      }
    }
    else if (serialInput.equalsIgnoreCase(".AUTO")) queueMessage("AUTO");
    else if (serialInput.equalsIgnoreCase(".STOPAUTO")) queueMessage("STOPAUTO");
    else if (serialInput.equalsIgnoreCase(".PAUSE")) queueMessage("PAUSE");
    else if (serialInput.equalsIgnoreCase(".RESUME")) queueMessage("RESUME");
    else if (serialInput.equalsIgnoreCase(".REVERSE")) queueMessage("REVERSE");
    else if (serialInput.equalsIgnoreCase(".CLEAR")) queueMessage("CLEAR");
    else if (serialInput.equalsIgnoreCase(".CLEARALL")) queueMessage("CLEARALL");
    else if (serialInput.equalsIgnoreCase(".RESET")) queueMessage("RES");
    else if (serialInput.equalsIgnoreCase(".RECAM")) queueMessage("RECAM");
    else if (serialInput.equalsIgnoreCase(".PARTY")) queueMessage("PARTY");
    else  {
      Serial.println("UNKNOWN_IGNORED");
    }
  }
}

void queueMessage(String msg) {
  msgToSend = msg;
  msgQueued = true;
  lastSendAttempt = 0;  // Enviar inmediatamente mensaje en cola
  msgStartTime = 0;
}

void broadcastControlState() {
  String msg = "CTRL_" + String(controllerClient);
  webSocket.broadcastTXT(msg);
}

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED: {
      Serial.printf("CLIENT_%u_CONN\n", client_num);

      // Asignar ID dado por Estación
      webSocket.sendTXT(client_num, "ASSIGN_ID_" + String(client_num));

      // Mandar estado de Control
      String helloMsg = "CTRL_" + String(controllerClient);
      webSocket.sendTXT(client_num, helloMsg);

      // Si no hay controlador, hacer este cliente controlador
      if (controllerClient == 255) {
        controllerClient = client_num;
        broadcastControlState();
        Serial.printf("CLIENT_%u_IS_CONTROLLER\n", client_num);
      }
      break;
    }

    case WStype_DISCONNECTED: {
      Serial.printf("CLIENT_%u_DC\n", client_num);

      // Si controlador DC, control disponible
      if (client_num == controllerClient) {
        controllerClient = 255;
        broadcastControlState();
        Serial.println("CONTROLLER_DC_RELEASED");
      }
      break;
    }

    case WStype_TEXT: {
      String msg = String((char*)payload);
      msg.trim();

      // Manejar solicitudes de Control
      if (msg == "REQUEST_CONTROL") {
        if (controllerClient == 255) {
          controllerClient = client_num;
          broadcastControlState();
          Serial.printf("CLIENT_%u_TAKES_CONTROL\n", client_num);
        } else {
          Serial.printf("CLIENT_%u_REQUEST_DENIED\n", client_num, controllerClient);
        }
        return;
      }

      // Manejar renuncio de Control
      if (msg == "RELEASE_CONTROL") {
        if (client_num == controllerClient) {
          controllerClient = 255;
          broadcastControlState();
          Serial.printf("CLIENT_%u_RELEASES_CONTROL\n", client_num);
        } else {
          // Nada
        }
        return;
      }

      // Get: Imagen, todos pueden (no solicitarla, desplegarla en UI si existe)
      if (msg == "WEB_IMG") {
        webSocket.sendTXT(client_num, "IMG_" + webImage);
        return;
      }

      // Solo Controlador puede enviar comands
      if (client_num == controllerClient) {
        if (msg == "UP") queueMessage("W");
        else if (msg == "DOWN") queueMessage("S");
        else if (msg == "LEFT") queueMessage("A");
        else if (msg == "RIGHT") queueMessage("D");
        else if (msg == "CAPTURE_IMG") queueMessage("IMG");
        else if (msg == "CANCEL_IMG") cancelImageTransfer();
        else if (msg == "RESET_CAM") queueMessage("RECAM");
        else if (msg == "START_TEL") queueMessage("GO");
        else if (msg == "STOP_TEL") queueMessage("STP");

        // GOAL_X,Y
        else if (msg.startsWith("GOAL_")) {
          String params = msg.substring(5);
          params.trim();

          int sep = params.indexOf(',');
          if (sep > 0) {
            String xGoal = params.substring(0, sep);
            String yGoal = params.substring(sep + 1);
            queueMessage("GOAL" + xGoal + "," + yGoal);
          }
          return;
        }

        // OBJECT_X,Y
        else if (msg.startsWith("OBJECT_")) {
          String params = msg.substring(7);
          params.trim();

          int sep = params.indexOf(',');
          if (sep > 0) {
            String xObj = params.substring(0, sep);
            String yObj = params.substring(sep + 1);
            queueMessage("OBJ" + xObj + "," + yObj);
          }
          return;
        }

        else if (msg.startsWith("CSV_")) {
          String intervalInput = msg.substring(4);
          intervalInput.trim();
          queueMessage("INT" + intervalInput);
          return;
        }
        
        // CHUNK_size
        else if (msg.startsWith("CHUNK_")) {
          String chunkSize = msg.substring(6);
          chunkSize.trim();
          queueMessage("CK" + chunkSize);
          return;
        }

        // STEP_size
        else if (msg.startsWith("STEP_")) {
          String stepSize = msg.substring(5);
          stepSize.trim();
          queueMessage("SET" + stepSize);
          return;
        }

        else if (msg == "START_AUTO") queueMessage("AUTO");
        else if (msg == "STOP_AUTO") queueMessage("STOPAUTO");
        else if (msg == "PAUSE_AUTO") queueMessage("PAUSE");
        else if (msg == "RESUME_AUTO") queueMessage("RESUME");
        else if (msg == "REVERSE_AUTO") queueMessage("REVERSE");
        else if (msg == "CLEAR_MAP") queueMessage("CLEAR");
        else if (msg == "CLEAR_ALL") queueMessage("CLEARALL");
        else if (msg == "RESET_ROVER") queueMessage("RES");
        else if (msg == "PARTY_MODE") queueMessage("PARTY");

        else if (msg == "LORA_SHORT") LoRaShort();
        else if (msg == "LORA_MEDIUM") LoRaMid();
        else if (msg == "LORA_LONG") LoRaLong();
      } else {
        Serial.printf("CLIENT_%u_ATTEMPTED_CMD\n", client_num);
      }
      break;
    }

    default:
      break;
  }
}

void LoRaShort() {
  Serial.println("LoRa_CHANGE_SF7");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(6);
  delay(50);
  retryInterval = 100;
  ackTimeout = 500;
  chunkTimeout = 1500;
  currentRange = SHORT;
  expectedLength = 200;
  webSocket.broadcastTXT("CSVINT_1.5");
  webSocket.broadcastTXT("CK_200");
}

void LoRaMid() {
  Serial.println("LoRa_CHANGE_SF9");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setPreambleLength(8);
  delay(50);
  retryInterval = 300;
  ackTimeout = 1500;
  chunkTimeout = 2000;
  currentRange = MID;
  expectedLength = 128;
  webSocket.broadcastTXT("CSVINT_2.5");
  webSocket.broadcastTXT("CK_128");
}

void LoRaLong() {
  Serial.println("LoRa_CHANGE_SF11");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(10);
  delay(50);
  retryInterval = 1500;
  ackTimeout = 6000;
  chunkTimeout = 5000;
  currentRange = LONG;
  expectedLength = 64;
  webSocket.broadcastTXT("CSVINT_5.0");
  webSocket.broadcastTXT("CK_64");
}

void printCommandList() {
  Serial.println("  Lista de Comandos:");
  Serial.println("    '.RESETSTA'     : Reiniciar ESP de Estación Terrestre");
  Serial.println("    '.W'            : Movimiento Hacia Adelante");
  Serial.println("    '.S'            : Movimiento Hacia Atrás");
  Serial.println("    '.A'            : Movimiento CCW");
  Serial.println("    '.D'            : Movimiento CW");
  Serial.println("    '.CHUNK'        : Elegir tamaño de Chunks (Bytes)");
  Serial.println("    '.STEP#'        : Elegir cantidad de Steps (1/32: 6400/Vuelta)");
  Serial.println("    '.INTERVAL#'    : Elegir intervalo de CSV");
  Serial.println("    '.FORCE###'     : Cambiar configuración LoRa: SHORT, MID, LONG");
  Serial.println("    '.GO' / '.START': Inicia envío de telemetría");
  Serial.println("    '.STOP'         : Detiene envío de telemetría");
  Serial.println("    '.IMAGE'        : Capturar y recibir Imagen");
  Serial.println("    '.CANCEL'       : Detener envío de Imagen");
  Serial.println("    '.AUTO'         : Inicia modo autónomo");
  Serial.println("    '.STOPAUTO'     : Detiene modo autónomo");
  Serial.println("    '.GOAL X Y'     : Establece objetivo en coordenadas X,Y");
  Serial.println("    '.OBJ X Y'      : Marca obstáculo en coordenadas X,Y");
  Serial.println("    '.PAUSE'        : Pausa modo autónomo");
  Serial.println("    '.RESUME'       : Reanuda modo autónomo");
  Serial.println("    '.REVERSE'      : Establece objetivo en (0,0)");
  Serial.println("    '.CLEAR'        : Limpia mapa manteniendo pose");
  Serial.println("    '.CLEARALL'     : Limpia mapa y resetea pose");
  Serial.println("    '.RESET'        : Resetear Rover - EVA");
  Serial.println("    '.RECAM'        : Resetear CAM");
}

void cancelImageTransfer() {
  receivingImage = false;
  imgBuffer = "";
  expectedChunks = 0;
  receivedChunks = 0;
  msgQueued = false;
  msgToSend = "";
  Serial.println("IMAGE_TRANSFER_CANCELLED");
  webSocket.broadcastTXT("IMG_DONE");
  queueMessage("STPIMG");
}

void watchdogTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_TASK_ENABLED");

 while (true) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
