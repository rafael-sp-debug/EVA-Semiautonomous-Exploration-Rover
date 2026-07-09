//IMPORTACIONES//
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h" //wd

//DECLARACIONES//
VL53L0X sensor1; // Sensor de distancia 1
VL53L0X sensor2; // Sensor de distancia 2
VL53L0X sensor3; // Sensor de distancia 3
Adafruit_INA219 ina219_ESP(0x40); // Sensor de corriente ESP32
Adafruit_INA219 ina219_M1(0x41); // Sensor de corriente Motor 1
Adafruit_INA219 ina219_M2(0x45); // Sensor de corriente Motor 2
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Sensor temperatura y humedad SHT31
Adafruit_MPU6050 mpu; // Sensor inercial MPU6050
Adafruit_AHTX0 aht; // Sensor temperatura y humedad AHT10

#define DIR1_PIN 27 // Pin dirección motor 1
#define STEP1_PIN 26 // Pin paso motor 1
#define SLEEP_PIN 25 // Pin sleep motores
#define DIR2_PIN 18 // Pin dirección motor 2
#define STEP2_PIN 19 // Pin paso motor 2

#define XSHUT1 16 // Pin XSHUT sensor VL53L0X 1
#define XSHUT2 17 // Pin XSHUT sensor VL53L0X 2
#define XSHUT3 4 // Pin XSHUT sensor VL53L0X 3
#define RX_PIN 3 // Pin UART RX LoRa
#define TX_PIN 1 // Pin UART TX LoRa

#define LED_PIN 5 // Pin control LEDs NeoPixel
#define LED_COUNT 8 // Cantidad de LEDs NeoPixel
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Tira de LEDs NeoPixel

int stepsPerRev = 3840; // Pasos por revolución del motor
#define STEP_DELAY_US 70 // Velocidad de paso en microsegundos
#define WDT_TIMEOUT 10 // Timeout watchdog en segundos

bool ledBreathing = false; // Modo LED respiración activo
bool ledFlashing = false; // Modo LED parpadeo activo
bool ledRainbow = false; // Modo LED arcoíris activo
bool ledDot = false; // Modo LED punto activo
bool ledIdle = true; // Modo LED inactivo activo
bool ledCam = false; // Modo LED cámara activo
bool ledObstacle = false; // Modo LED obstáculo rojo activo

bool shtPresent = false; // Sensor SHT31 presente
bool ahtPresent = false; // Sensor AHT10 presente
bool inaESP_Present = false; // Sensor INA219 ESP32 presente
bool inaM1_Present = false; // Sensor INA219 Motor 1 presente
bool inaM2_Present = false; // Sensor INA219 Motor 2 presente
bool mpuPresent = false; // Sensor MPU6050 presente

unsigned long previousMillis = 0; // Tiempo anterior para intervalo CSV
unsigned long previousPing = 0; // Tiempo anterior para intervalo PING
float intervalSec = 1.5; // Intervalo en segundos para envío CSV
unsigned long interval = intervalSec * 1000; // Intervalo convertido a milisegundos
unsigned long pinginterval = 30000; // Intervalo convertido a milisegundos
bool sendCSV = false; // Bandera para controlar envío CSV
bool sendingImg = false; // Bandera para indicar si imagen se está enviando
bool sendTelemetry = true; // Bandera para alternar entre telemetría y grid

#define CSV_BUFFER_SIZE 96 // Tamaño del buffer CSV
char csvBuffer[CSV_BUFFER_SIZE]; // Buffer para datos CSV

enum MotorCommand {
  MOTOR_IDLE, // Motor inactivo
  MOTOR_BOTH_CW, // Ambos motores sentido horario
  MOTOR_BOTH_CCW, // Ambos motores sentido antihorario
  MOTOR_OPPOSITE_A, // Motores opuestos giro izquierda
  MOTOR_OPPOSITE_D // Motores opuestos giro derecha
};

volatile MotorCommand currentCommand = MOTOR_IDLE; // Comando actual del motor
TaskHandle_t watchdogTaskHandle = NULL; // Handle de tarea watchdog

const int GRID_W = 10; // Ancho del grid en celdas
const int GRID_H = 10; // Alto del grid en celdas
uint8_t occGrid[GRID_W][GRID_H]; // Grid de ocupación: 0 libre, 1 obstáculo

const float WHEEL_RADIUS = 0.04f; // Radio de rueda en metros
const float WHEEL_CIRC = 2.0 * 3.14159265358979323846f * WHEEL_RADIUS; // Circunferencia de rueda en metros
const float CELL_SIZE_M = 0.365f; // Tamaño de celda en metros (valor por defecto)

volatile int posX = 0; // Posición X actual en celdas
volatile int posY = 0; // Posición Y actual en celdas
volatile int heading = 0; // Orientación: 0=Norte, 1=Este, 2=Sur, 3=Oeste

volatile int goalX = 0; // Coordenada X del objetivo en celdas
volatile int goalY = 0; // Coordenada Y del objetivo en celdas
volatile bool haveGoal = false; // Bandera de objetivo definido

volatile bool autoMode = false; // Bandera modo autónomo activo
volatile bool autoPaused = false; // Bandera modo autónomo pausado
volatile bool movingFlag = false; // Bandera de movimiento en ejecución
volatile bool obstacleFlag = false; // Bandera de obstáculo detectado
volatile bool evasionMode = false; // Bandera de modo evasión
volatile int evasionCellsCount = 0; // Contador de celdas avanzadas en modo evasión
const int EVASION_CELLS_MIN = 2; // Mínimo de celdas a avanzar en modo evasión antes de recalcular path

struct Cell { int x; int y; }; // Estructura de celda del grid
Cell pathCells[GRID_W * GRID_H]; // Array de celdas del camino
int pathLen = 0; // Longitud del camino
int pathIndex = 0; // Índice actual en el camino

volatile int stepsPerCell = 0; // Declaración de stepsPercell
const int turnSteps = 3740; // Pasos para giro de 90 grados

volatile int stepsToDo = 0; // Pasos objetivo para motorTask
volatile int stepsDone = 0; // Pasos completados por motorTask
volatile MotorCommand activeMotorCommand = MOTOR_IDLE; // Comando de motor en ejecución
volatile int desiredHeadingAfterTurn = -1; // Dirección deseada después del giro (-1 = no hay giro pendiente)

unsigned long lastAutoTick = 0; // Último tick del modo autónomo
const unsigned long AUTO_STEP_DELAY_MS = 50; // Retardo entre pasos autónomos en ms

const int MAXN = GRID_W * GRID_H; // Tamaño máximo para arrays A*
static bool astar_closed[MAXN]; // Array de nodos cerrados A*
static int astar_gscore[MAXN]; // Array de puntuación G A*
static int astar_fscore[MAXN]; // Array de puntuación F A*
static int astar_came_from[MAXN]; // Array de nodos previos A*
static int astar_openList[MAXN]; // Lista abierta A*
static int astar_revPath[MAXN]; // Camino reverso A*

//Declaración de funciones
void planPathAstar();
bool astar(int sx, int sy, int gx, int gy, Cell *outPath, int &outLen);
int heuristic(int ax, int ay, int bx, int by);
void occupancyMarkFromSensorReadings();
void markObstacleCellFromSensor(int sensorIndex, int distance_mm);
void executeNextStepFromPathNonBlocking();
void rotateToDirNonBlocking(int desiredDir);
void queueForwardOneCellNonBlocking();
void queueBackwardOneCellNonBlocking();
void calculateStepsPerCell();
void updatePoseAfterForward();
void clearMapKeepPose();
void clearAllResetPose();
void appendNavBinary();
bool checkObstacleInFront();
void executeAutonomousMode();

// Inicialización del sistema
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);

  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_LOOP_ENABLED");

  xTaskCreatePinnedToCore(
    watchdogTask,
   "watchdogTask",
    4096,
    NULL,
    1,
    &watchdogTaskHandle,
    0
  );

  strip.begin();
  strip.show();

  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);

  digitalWrite(SLEEP_PIN, LOW);
  
  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    ledTask,
    "LED Task",
    2048,
    NULL,
    1,
    NULL,
    1
  );

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(1000);

  Wire.begin();
  delay(200);

  digitalWrite(XSHUT1, HIGH);
  delay(150);
  sensor1.init(true);
  sensor1.setAddress(0x30);

  digitalWrite(XSHUT2, HIGH);
  delay(150);
  sensor2.init(true);
  sensor2.setAddress(0x31);

  digitalWrite(XSHUT3, HIGH);
  delay(150);
  sensor3.init(true);
  sensor3.setAddress(0x32);
  
  delay(100);

  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  
  inaESP_Present = ina219_ESP.begin();
  if (!inaESP_Present) Serial.println("INA219_ESP NOT FOUND");

  inaM1_Present = ina219_M1.begin();
  if (!inaM1_Present) Serial.println("INA219_M1 NOT FOUND");

  inaM2_Present = ina219_M2.begin();
  if (!inaM2_Present) Serial.println("INA219_M2 NOT FOUND");

  shtPresent = sht31.begin(0x44);
  if (!shtPresent) Serial.println("SHT31 NOT FOUND");

  ahtPresent = aht.begin();
  if (!ahtPresent) Serial.println("AHT10 NOT FOUND");

  mpuPresent = mpu.begin();
  if (!mpuPresent) Serial.println("MPU6050 NOT FOUND");
  else {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  for (int i=0;i<GRID_W;i++) for (int j=0;j<GRID_H;j++) occGrid[i][j]=0;
  calculateStepsPerCell();
}

// Bucle principal
void loop() {
  unsigned long currentMillis = millis();

  esp_task_wdt_reset();

  if (sendCSV && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (sendTelemetry) {
      getAllSensorsCSV();
      Serial.print(csvBuffer);
      Serial.println();
    } else {
      sendGridPacket();
    }
    sendTelemetry = !sendTelemetry;
  }

  else if (!sendCSV && !sendingImg && currentMillis - previousPing >= pinginterval) {
    previousPing = currentMillis;
    Serial.println("PING");
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("IMGSTART")) {
      sendingImg = true;
      previousPing = millis();
      ledFlashing = true;
      ledBreathing = ledRainbow = ledDot = ledCam = false;
    }
    else if (cmd.equalsIgnoreCase("IMGDONE")) {
      sendingImg = false;
      ledFlashing = false;
    }
    else if (cmd.equalsIgnoreCase("GO")) {
      if (sendCSV) {
        Serial.println("CSV_ALREADY_STARTED");
      } else {
        sendCSV = true;
        Serial.println("CSV_SENDING_STARTED");
        ledBreathing = true;
        ledFlashing = ledRainbow = ledDot = ledCam = false;
      }
    }
    else if (cmd.equalsIgnoreCase("STP") || cmd.equalsIgnoreCase(".STOP")) {
      if (!sendCSV) {
        Serial.println("CSV_ALREADY_STOPPED");
      } else {
        sendCSV = false;
        ledBreathing = false;
        Serial.println("CSV_SENDING_STOPPED");
      }
    }
    else if (cmd.equalsIgnoreCase("RES")) {
      Serial.println("RE_EVA");
      delay(200);
      ESP.restart();
    }
    else if (cmd.startsWith("INT")) {
      float newIntervalSec = cmd.substring(3).toFloat();
      intervalSec = newIntervalSec;
      interval = (unsigned long)(intervalSec * 1000);
      Serial.println("CSV_INT" + String(intervalSec, 2));
    }
    else if (cmd.startsWith("SET")) {
      int newSteps = cmd.substring(3).toInt();
      if (newSteps > 0) {
        stepsPerRev = newSteps;
        // NO recalcular stepsPerCell aquí - el usuario puede cambiarlo manualmente
        Serial.println("NEMA_" + String(stepsPerRev));
      } else {
        Serial.println("ERROR_INVALID_STEPS");
      }
    } 
    else if (cmd.equalsIgnoreCase("W")) {
      if (autoMode && !autoPaused) {
        Serial.println("CURRENT_AUTONOMOUS_MODE");
      } else if (checkObstacleInFront()) {
        Serial.println("OBJECT_BLOCKING_FRONT");
      } else if (!movingFlag) {
        // Usar stepsPerCell directamente
        digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, LOW);
        stepsToDo = stepsPerCell;
        stepsDone = 0;
        activeMotorCommand = MOTOR_BOTH_CW;
        movingFlag = true;
      }
    }
    else if (cmd.equalsIgnoreCase("S")) {
      if (autoMode && !autoPaused) {
        Serial.println("CURRENT_AUTONOMOUS_MODE");
      } else if (!movingFlag) {
        // Usar stepsPerCell directamente
        digitalWrite(DIR1_PIN, HIGH);
        digitalWrite(DIR2_PIN, HIGH);
        stepsToDo = stepsPerCell;
        stepsDone = 0;
        activeMotorCommand = MOTOR_BOTH_CCW;
        movingFlag = true;
      }
    }
    else if (cmd.equalsIgnoreCase(".NOSTEP")) {
      calculateStepsPerCell();
      Serial.println("STEPS_PER_CELL_RESET_TO_DEFAULT");
    }
    else if (cmd.equalsIgnoreCase("A")) {
      if (autoMode && !autoPaused) {
        Serial.println("CURRENT_AUTONOMOUS_MODE");
      } else {
        currentCommand = MOTOR_OPPOSITE_A;
      }
    }
    else if (cmd.equalsIgnoreCase("D")) {
      if (autoMode && !autoPaused) {
        Serial.println("CURRENT_AUTONOMOUS_MODE");
      } else {
        currentCommand = MOTOR_OPPOSITE_D;
      }
    }

    else if (cmd.startsWith("GOAL") || cmd.startsWith(".GOAL")) {
      int offset = cmd.startsWith(".GOAL") ? 5 : 4;
      String params = cmd.substring(offset);
      params.trim();
      int commaIndex = params.indexOf(',');
      if (commaIndex > 0) {
        String xStr = params.substring(0, commaIndex);
        String yStr = params.substring(commaIndex + 1);
        xStr.trim();
        yStr.trim();
        int gx = xStr.toInt();
        int gy = yStr.toInt();
        if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
          goalX = gx;
          goalY = gy;
          haveGoal = true;
          Serial.println("GOAL_SET " + String(gx) + "," + String(gy));
        } else {
          Serial.println("ERROR_GOAL_OUT_OF_BOUNDS");
        }
      } else {
        Serial.println("ERROR_GOAL_FORMAT");
      }
    }
    else if (cmd.startsWith("OBJ") || cmd.startsWith(".OBJ")) {
      int offset = cmd.startsWith(".OBJ") ? 4 : 3;
      String params = cmd.substring(offset);
      params.trim();
      int commaIndex = params.indexOf(',');
      int spaceIndex = params.indexOf(' ');
      int separatorIndex = (commaIndex > 0) ? commaIndex : spaceIndex;
      
      if (separatorIndex > 0) {
        String xStr = params.substring(0, separatorIndex);
        String yStr = params.substring(separatorIndex + 1);
        xStr.trim();
        yStr.trim();
        int objX = xStr.toInt();
        int objY = yStr.toInt();
        if (objX >= 0 && objX < GRID_W && objY >= 0 && objY < GRID_H) {
          occGrid[objX][objY] = 1;
          Serial.println("OBJ_SET");
          if (autoMode && pathLen > 0) {
            pathIndex = 0;
            pathLen = 0;
            planPathAstar();
          }
        } else {
          Serial.println("ERROR_OBJ_OUT_OF_BOUNDS");
        }
      } else {
        Serial.println("ERROR_OBJ_FORMAT");
      }
    }
    else if (cmd.equalsIgnoreCase(".AUTO") || cmd.equalsIgnoreCase("AUTO")) {
      if (!haveGoal) {
        Serial.println("NO_GOAL_DEFINED");
      } else {
        pathLen = 0;
        pathIndex = 0;
        movingFlag = false;
        stepsToDo = 0;
        stepsDone = 0;
        activeMotorCommand = MOTOR_IDLE;
        desiredHeadingAfterTurn = -1;
        evasionMode = false;
        evasionCellsCount = 0;
        autoMode = true;
        autoPaused = false;
        lastAutoTick = millis();
        Serial.println("AUTO_STARTED");
      }
    }
    else if (cmd.equalsIgnoreCase("STOPAUTO") || cmd.equalsIgnoreCase(".STOPAUTO")) {
      autoPaused = true;
      autoMode = false;
      Serial.println("AUTO_STOPPED");
    }
    else if (cmd.equalsIgnoreCase("PAUSE") || cmd.equalsIgnoreCase(".PAUSE")){
      if (autoMode && !autoPaused) {
        autoPaused = true;
        Serial.println("AUTO_PAUSED");
      } else if (!autoMode) {
        Serial.println("AUTO_NOT_ACTIVE");
      } else {
        Serial.println("AUTO_ALREADY_PAUSED");
      }
    }
    else if (cmd.equalsIgnoreCase("RESUME") || cmd.equalsIgnoreCase(".RESUME")) {
      if (autoMode && autoPaused) {
        autoPaused = false;
        lastAutoTick = millis();
        // Recalcular ruta desde la posición actual
        pathIndex = 0;
        pathLen = 0;
        planPathAstar();
        Serial.println("AUTO_RESUMED");
      } else if (!autoMode) {
        Serial.println("AUTO_NOT_ACTIVE");
      } else {
        Serial.println("AUTO_NOT_PAUSED");
      }
    }
    else if (cmd.equalsIgnoreCase("REVERSE") || cmd.equalsIgnoreCase(".REVERSE")) {
      goalX = 0;
      goalY = 0;
      haveGoal = true;
      Serial.println("REVERSE_GOAL_SET");
    }
    else if (cmd.equalsIgnoreCase("CLEAR") || cmd.equalsIgnoreCase(".CLEAR")) {
      clearMapKeepPose();
      Serial.println("MAP_CLEARED");
    }
    else if (cmd.equalsIgnoreCase("CLEARALL") || cmd.equalsIgnoreCase(".CLEARALL")) {
      clearAllResetPose();
      Serial.println("MAP_CLEARED_AND_RESET");
    }
    else if (cmd.equalsIgnoreCase("FCAM")) {
      ledCam = true;
      ledBreathing = ledFlashing = ledRainbow = ledDot = false;
    }
    else if (cmd.equalsIgnoreCase("PARTY")) {
      ledRainbow = true;
      ledBreathing = ledFlashing =  ledDot = ledCam = false;
    }
  }
  if (autoMode && !autoPaused) {
    executeAutonomousMode(); // Llamar a función de modo autónomo
  }

  delay(2);
}

String getAllSensorsCSV() { //Función para obtener la info de los sensores en CSV
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    int16_t sht_temp_i = 0; // Temperatura SHT31 en centésimas
    int16_t sht_hum_i  = 0; // Humedad SHT31 en centésimas

    if (shtPresent) {
        float t = sht31.readTemperature();
        float h = sht31.readHumidity();
        if (!isnan(t) && !isnan(h)) {
            sht_temp_i = (int16_t)(t * 100);
            sht_hum_i  = (int16_t)(h * 100);
        } else {
            shtPresent = false;
        }
    }
    if (!shtPresent) shtPresent = tryBeginSHT();

    int16_t int_temp_i = 0; // Temperatura AHT10 en centésimas
    int16_t int_hum_i  = 0; // Humedad AHT10 en centésimas

    if (ahtPresent) {
        sensors_event_t ah, at;
        aht.getEvent(&ah, &at);
        if (!isnan(at.temperature)) {
            int_temp_i = (int16_t)(at.temperature * 100);
            int_hum_i  = (int16_t)(ah.relative_humidity * 100);
        } else {
            ahtPresent = false;
        }
    }
    if (!ahtPresent) ahtPresent = tryBeginAHT();

    int16_t espBusV_i = 0; // Voltaje bus ESP32 en centésimas
    int16_t espCurrent_i = 0; // Corriente ESP32 en décimas de mA
    int16_t espPower_i = 0; // Potencia ESP32 en mW

    if (inaESP_Present) {
        float v = ina219_ESP.getBusVoltage_V();
        if (!isnan(v)) {
            espBusV_i   = (int16_t)(v * 100);
            espCurrent_i = (int16_t)(ina219_ESP.getCurrent_mA() * 10);
            espPower_i   = (int16_t)(ina219_ESP.getPower_mW());
        } else {
            inaESP_Present = false;
        }
    }
    if (!inaESP_Present) inaESP_Present = tryBeginINA(ina219_ESP);

    int16_t m1BusV_i = 0; // Voltaje bus Motor 1 en centésimas
    int16_t m1Current_i = 0; // Corriente Motor 1 en décimas de mA
    int16_t m1Power_i = 0; // Potencia Motor 1 en mW

    if (inaM1_Present) {
        float v = ina219_M1.getBusVoltage_V();
        if (!isnan(v)) {
            m1BusV_i = (int16_t)(v * 100);
            m1Current_i = (int16_t)(ina219_M1.getCurrent_mA() * 10);
            m1Power_i = (int16_t)(ina219_M1.getPower_mW());
        } else {
            inaM1_Present = false;
        }
    }
    if (!inaM1_Present) inaM1_Present = tryBeginINA(ina219_M1);

    int16_t m2BusV_i = 0; // Voltaje bus Motor 2 en centésimas
    int16_t m2Current_i = 0; // Corriente Motor 2 en décimas de mA
    int16_t m2Power_i = 0; // Potencia Motor 2 en mW

    if (inaM2_Present) {
        float v = ina219_M2.getBusVoltage_V();
        if (!isnan(v)) {
            m2BusV_i = (int16_t)(v * 100);
            m2Current_i = (int16_t)(ina219_M2.getCurrent_mA() * 10);
            m2Power_i = (int16_t)(ina219_M2.getPower_mW());
        } else {
            inaM2_Present = false;
        }
    }
    if (!inaM2_Present) inaM2_Present = tryBeginINA(ina219_M2);

    int16_t accX_i = a.acceleration.x * 100; // Aceleración X en centésimas
    int16_t accY_i = a.acceleration.y * 100; // Aceleración Y en centésimas
    int16_t accZ_i = a.acceleration.z * 100; // Aceleración Z en centésimas
    int16_t gyroX_i = g.gyro.x * 100; // Giroscopio X en centésimas
    int16_t gyroY_i = g.gyro.y * 100; // Giroscopio Y en centésimas
    int16_t gyroZ_i = g.gyro.z * 100; // Giroscopio Z en centésimas

    int16_t dist1 = sensor1.readRangeContinuousMillimeters(); // Distancia sensor 1 en mm
    int16_t dist2 = sensor2.readRangeContinuousMillimeters(); // Distancia sensor 2 en mm
    int16_t dist3 = sensor3.readRangeContinuousMillimeters(); // Distancia sensor 3 en mm

    snprintf(csvBuffer, CSV_BUFFER_SIZE,
        "%d,%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d",
        sht_temp_i, sht_hum_i, int_temp_i, int_hum_i,
        espBusV_i, espCurrent_i, espPower_i,
        m1BusV_i, m1Current_i, m1Power_i,
        m2BusV_i, m2Current_i, m2Power_i,
        accX_i, accY_i, accZ_i, gyroX_i, gyroY_i, gyroZ_i,
        dist1, dist2, dist3
    );

    return String(csvBuffer);
}

// Genera y envía el paquete del grid
void sendGridPacket() {
    uint8_t tempGrid[GRID_W][GRID_H];
    for (int x = 0; x < GRID_W; x++) {
        for (int y = 0; y < GRID_H; y++) {
            tempGrid[x][y] = occGrid[x][y];
        }
    }
    int robotValue = 2; // Por defecto Norte
    if (heading == 0) robotValue = 2;      // Norte
    else if (heading == 1) robotValue = 3; // Este
    else if (heading == 2) robotValue = 5; // Sur
    else robotValue = 4;                  // Oeste
    tempGrid[posX][posY] = robotValue;
    // Marcar la meta con 6 si está definida
    if (haveGoal && goalX >= 0 && goalX < GRID_W && goalY >= 0 && goalY < GRID_H) {
        tempGrid[goalX][goalY] = 6;
    }
    Serial.print("M");
    for (int y = 0; y < GRID_H; y++) {
        Serial.print(",");
        for (int x = 0; x < GRID_W; x++) {
            Serial.print(tempGrid[x][y]);
        }
    }
    Serial.println();
}
// Intenta inicializar sensor SHT31
bool tryBeginSHT() {
    if (sht31.begin(0x44)) {
        return true;
    }
    return false;
}
// Intenta inicializar sensor AHT10
bool tryBeginAHT() {
    if (aht.begin()) {
        return true;
    }
    return false;
}
// Intenta inicializar sensor INA219
bool tryBeginINA(Adafruit_INA219 &ina) {
    if (ina.begin()) {
        return true;
    }
    return false;
}
// Tarea de control de motores no bloqueante
void motorTask(void * parameter) {
  while (true) {
    MotorCommand cmdLocal = currentCommand;
    int toDoLocal = stepsToDo;
    int doneLocal = stepsDone;
    MotorCommand activeCmdLocal = activeMotorCommand;
    bool autoModeLocal = autoMode;
    int desiredHeadingLocal = desiredHeadingAfterTurn;
    bool movingFlagLocal = movingFlag;
    int headingLocal = heading;
    int posXLocal = posX;
    int posYLocal = posY;
    bool sendCSVLocal = sendCSV;
    int stepsPerRevLocal = stepsPerRev;
    //Si hay pasos pendientes del sistema
    if (toDoLocal > 0) {
      if (doneLocal == 0) {
        digitalWrite(SLEEP_PIN, HIGH);
        ledDot = true;
        ledBreathing = ledFlashing = ledRainbow = ledCam = false;
      }

      unsigned long stepDelay = STEP_DELAY_US;

      digitalWrite(STEP1_PIN, HIGH);
      digitalWrite(STEP2_PIN, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP1_PIN, LOW);
      digitalWrite(STEP2_PIN, LOW);
      delayMicroseconds(stepDelay);

      doneLocal++;
      stepsDone = doneLocal; // Actualizar variable global con el valor local
      if (doneLocal >= toDoLocal) {
        // Movimiento completado
        bool wasForward = (activeCmdLocal == MOTOR_BOTH_CW);
        bool wasTurn = (activeCmdLocal == MOTOR_OPPOSITE_A || activeCmdLocal == MOTOR_OPPOSITE_D);
        bool wasAutonomous = autoModeLocal;
        stepsToDo = 0;
        stepsDone = 0;
        MotorCommand completedCommand = activeCmdLocal;
        activeMotorCommand = MOTOR_IDLE;
        digitalWrite(SLEEP_PIN, LOW);
        ledDot = false;
        movingFlag = false;
        
        // Actualizar pose según el tipo de movimiento
        if (wasForward) {
          updatePoseAfterForward();
        } else if (activeCmdLocal == MOTOR_BOTH_CCW) {
          // Leer heading y posiciones actualizadas antes de modificar
          headingLocal = heading;
          posXLocal = posX;
          posYLocal = posY;
          if (headingLocal == 0) posY -= 1;
          else if (headingLocal == 1) posX -= 1;
          else if (headingLocal == 2) posY += 1;
          else posX += 1;
          if (posX < 0) posX = 0;
          if (posY < 0) posY = 0;
          if (posX >= GRID_W) posX = GRID_W - 1;
          if (posY >= GRID_H) posY = GRID_H - 1;
        } else if (wasTurn) {
          if (wasAutonomous && desiredHeadingLocal >= 0) {
            heading = desiredHeadingLocal;
            desiredHeadingAfterTurn = -1;
          } else {
            headingLocal = heading;
            if (completedCommand == MOTOR_OPPOSITE_A) {
              heading = (headingLocal + 3) % 4;
            } else if (completedCommand == MOTOR_OPPOSITE_D) {
              heading = (headingLocal + 1) % 4;
            }
          }
        }
        // Procesamiento adicional para giros autónomos
        if (wasTurn && wasAutonomous && desiredHeadingLocal >= 0) {
          vTaskDelay(50 / portTICK_PERIOD_MS); //delay para que los sensores se estabilicen
          int dfront1 = sensor1.readRangeContinuousMillimeters();
          int dfront2 = sensor1.readRangeContinuousMillimeters();
          int dfront = (dfront1 < dfront2) ? dfront1 : dfront2;
          const int TH_MM = 150;
          if (dfront == 0 || dfront >= TH_MM) {
            obstacleFlag = false;
          }
        }
        if (!wasAutonomous) {
          occupancyMarkFromSensorReadings();
          if (sendCSVLocal) {
            ledBreathing = true;
            ledFlashing = ledRainbow = ledDot = ledCam = false;
          }
        }
        if (wasAutonomous) {
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
      } else {
        movingFlag = true;
      }
    }
    //Si hay un comando manual nuevo y no hay movimiento en curso
    else if (cmdLocal != MOTOR_IDLE && !movingFlagLocal) {
      // Convertir comando manual al sistema no bloqueante
      int dir1 = LOW, dir2 = LOW;
      int stepsToRun = stepsPerRevLocal;
      switch (cmdLocal) {
        case MOTOR_BOTH_CW:
          dir1 = LOW;
          dir2 = LOW;
          stepsToRun = stepsPerRevLocal;
          break;
        case MOTOR_BOTH_CCW:
          dir1 = HIGH;
          dir2 = HIGH;
          stepsToRun = stepsPerRevLocal;
          break;
        case MOTOR_OPPOSITE_A:
          dir1 = HIGH;
          dir2 = LOW;
          stepsToRun = turnSteps;
          break;
        case MOTOR_OPPOSITE_D:
          dir1 = LOW;
          dir2 = HIGH;
          stepsToRun = turnSteps;
          break;
        default:
          stepsToRun = stepsPerRevLocal;
      }
      // Establecer direcciones y activar sistema
      digitalWrite(DIR1_PIN, dir1);
      digitalWrite(DIR2_PIN, dir2);
      stepsToDo = stepsToRun;
      stepsDone = 0;
      activeMotorCommand = cmdLocal; // Guardar comando para saber qué hacer al terminar
      movingFlag = true;
      currentCommand = MOTOR_IDLE; // Limpiar comando manual
    }
    else {
      if (digitalRead(SLEEP_PIN) == HIGH) {
        digitalWrite(SLEEP_PIN, LOW);
      }
    }

    vTaskDelay(1);
  }
}

// Tarea de control de LEDs
void ledTask(void *parameter) {
  unsigned long previousMillis = 0; // Tiempo anterior para animaciones
  int brightness = 0; // Brillo actual
  int fadeAmount = 5; // Cantidad de fade
  static int dotIndex = 0; // Índice del punto LED
  static int dotDirection = 1; // Dirección del punto LED
  static uint16_t rainbowOffset = 0; // Offset del arcoíris

  while (true) {
    if (ledBreathing) {
      float level = (sin(millis() / 500.0) + 1.0) / 2.0;
      setColor(0, 20 * level, 0);
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    else if (ledFlashing) {
      static bool state = false; // Estado del parpadeo
      state = !state;
      if (state) setColor(10, 10, 10);
      else setColor(0, 0, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

  else if (ledRainbow) {
    static unsigned long rainbowStart = 0; // Inicio del arcoíris

    if (rainbowStart == 0) {
      rainbowStart = millis();
    }

    for (int i = 0; i < LED_COUNT; i++) {
      int pixelHue = (i * 65536L / LED_COUNT) + rainbowOffset;
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show();

    rainbowOffset += 256;
    vTaskDelay(20 / portTICK_PERIOD_MS);

    if (millis() - rainbowStart >= 5000) {
      ledRainbow = false;
      rainbowStart = 0;
    }
  }

  else if (ledObstacle) {
    static unsigned long obstacleStart = 0; // Inicio del efecto obstáculo

    if (obstacleStart == 0) {
      obstacleStart = millis();
      strip.clear();
    }

    unsigned long elapsed = millis() - obstacleStart;
    
    if (elapsed < 800) { // Primera fase: efecto de subida (800ms)
      int ledToLight = (elapsed * LED_COUNT) / 800;
      if (ledToLight > LED_COUNT) ledToLight = LED_COUNT;
      strip.clear();
      for (int i = 0; i < ledToLight; i++) {
        strip.setPixelColor(i, strip.Color(100, 0, 0)); // Rojo
      }
      strip.show();
      vTaskDelay(20 / portTICK_PERIOD_MS);
    } else if (elapsed < 2000) {
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(100, 0, 0)); // Rojo
      }
      strip.show();
      vTaskDelay(20 / portTICK_PERIOD_MS);
    } else {
      ledObstacle = false;
      obstacleStart = 0;
      strip.clear();
      strip.show();
    }
  }

    else if (ledDot) {
      strip.clear();

      strip.setPixelColor(dotIndex, strip.Color(0, 0, 15));
      strip.setPixelColor(dotIndex + 1, strip.Color(0, 0, 25));
      strip.setPixelColor(dotIndex + 2, strip.Color(0, 0, 25));

      strip.show();

      dotIndex += dotDirection;

      if (dotIndex >= LED_COUNT - 2 || dotIndex <= 0) {
        dotDirection = -dotDirection;
      }

      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    else if (ledCam) {
      setColor(150, 150, 150);
      strip.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      ledCam = false;
    }

    else if (!ledBreathing && !ledFlashing && !ledRainbow && !ledDot && !ledCam && !ledObstacle) {
      float level = (sin(millis() / 500.0) + 1.0) / 2.0;
      setColor(0, 10 * level, 10 * level);
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    else {
      setColor(0, 0, 0);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// Tarea watchdog
void watchdogTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_TASK_ENABLED");

  while (true) {
    esp_task_wdt_reset();
   vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Establece color de todos los LEDs
void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  uint32_t color = strip.Color(red, green, blue);
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Calcula heurística Manhattan para A*
int heuristic(int ax, int ay, int bx, int by) {
  return abs(ax - bx) + abs(ay - by);
}


// Algoritmo A* para encontrar camino
bool astar(int sx, int sy, int gx, int gy, Cell *outPath, int &outLen) {
  for (int i = 0; i < MAXN; i++) {
    astar_closed[i] = false;
    astar_gscore[i] = 1000000;
    astar_fscore[i] = 1000000;
    astar_came_from[i] = -1;
  }

  auto idx = [](int x, int y) { return y * GRID_W + x; };

  int startIdx = idx(sx, sy); // Índice de inicio
  int goalIdx = idx(gx, gy); // Índice de objetivo

  astar_gscore[startIdx] = 0;
  astar_fscore[startIdx] = heuristic(sx, sy, gx, gy);

  int openLen = 0; // Longitud de lista abierta
  astar_openList[openLen++] = startIdx;

  int iterCount = 0; // Contador de iteraciones
  while (openLen > 0) {
    if (++iterCount % 50 == 0) {
      yield();
    }
    
    int bestI = 0; // Mejor índice en lista abierta
    for (int i = 1; i < openLen; i++) {
      if (astar_fscore[astar_openList[i]] < astar_fscore[astar_openList[bestI]]) bestI = i;
    }
    int current = astar_openList[bestI]; // Nodo actual
    for (int i = bestI; i < openLen - 1; i++) astar_openList[i] = astar_openList[i + 1];
    openLen--;

    if (current == goalIdx) {
      int cur = current;
      int revLen = 0; // Longitud del camino reverso
      while (cur != -1) {
        astar_revPath[revLen++] = cur;
        cur = astar_came_from[cur];
      }
      outLen = 0;
      for (int i = revLen - 1; i >= 0; i--) {
        int id = astar_revPath[i];
        int cx = id % GRID_W;
        int cy = id / GRID_W;
        outPath[outLen++] = {cx, cy};
      }
      return true;
    }

    astar_closed[current] = true;

    int cx = current % GRID_W; // Coordenada X de celda actual
    int cy = current / GRID_W; // Coordenada Y de celda actual
    const int d4[4][2] = {{0,1},{1,0},{0,-1},{-1,0}}; // Direcciones: N, E, S, O
    for (int k = 0; k < 4; k++) {
      int nx = cx + d4[k][0]; // Coordenada X vecina
      int ny = cy + d4[k][1]; // Coordenada Y vecina
      if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;
      if (occGrid[nx][ny]) continue;
      int nidx = idx(nx, ny); // Índice del vecino
      if (astar_closed[nidx]) continue;
      int tentative_g = astar_gscore[current] + 1; // Puntuación G tentativa
      if (tentative_g < astar_gscore[nidx]) {
        astar_came_from[nidx] = current;
        astar_gscore[nidx] = tentative_g;
        astar_fscore[nidx] = tentative_g + heuristic(nx, ny, gx, gy);
        bool present = false; // Vecino presente en lista abierta
        for (int i = 0; i < openLen; i++) if (astar_openList[i] == nidx) { present = true; break; }
        if (!present) {
          astar_openList[openLen++] = nidx;
        }
      }
    }
  }

  outLen = 0;
  return false;
}

// Planifica camino usando A*
void planPathAstar() {
  int sx = posX, sy = posY; // Coordenadas inicio
  int gx = goalX, gy = goalY; // Coordenadas objetivo
  Cell tmpPath[GRID_W * GRID_H]; // Camino temporal
  int outLen = 0; // Longitud del camino
  bool ok = astar(sx, sy, gx, gy, tmpPath, outLen);
  if (ok && outLen > 0) {
    pathLen = outLen;
    for (int i = 0; i < pathLen; i++) pathCells[i] = tmpPath[i];
    pathIndex = 0;
  } else {
    pathLen = 0;
    pathIndex = 0;
  }
}

// Marca ocupación del grid usando lecturas de sensores
void occupancyMarkFromSensorReadings() {
  // Sensor frontal
  int d1_1 = sensor1.readRangeContinuousMillimeters(); // Lectura 1 sensor 1
  int d1_2 = sensor1.readRangeContinuousMillimeters(); // Lectura 2 sensor 1
  int d1 = (d1_1 < d1_2) ? d1_1 : d1_2;
  
  // Sensores laterales
  int d2_1 = sensor2.readRangeContinuousMillimeters(); // Lectura 1 sensor 2 
  int d2_2 = sensor2.readRangeContinuousMillimeters(); // Lectura 2 sensor 2
  int d2_3 = sensor2.readRangeContinuousMillimeters(); // Lectura 3 sensor 2
  // Usar la mediana de las 3 lecturas para mayor estabilidad
  int d2;
  if (d2_1 <= d2_2 && d2_2 <= d2_3) d2 = d2_2;
  else if (d2_2 <= d2_1 && d2_1 <= d2_3) d2 = d2_1;
  else d2 = d2_3;
  
  int d3_1 = sensor3.readRangeContinuousMillimeters(); // Lectura 1 sensor 3 (izquierda)
  int d3_2 = sensor3.readRangeContinuousMillimeters(); // Lectura 2 sensor 3
  int d3_3 = sensor3.readRangeContinuousMillimeters(); // Lectura 3 sensor 3
  // Usar la mediana de las 3 lecturas para mayor estabilidad
  int d3;
  if (d3_1 <= d3_2 && d3_2 <= d3_3) d3 = d3_2;
  else if (d3_2 <= d3_1 && d3_1 <= d3_3) d3 = d3_1;
  else d3 = d3_3;

  const int TH_MM = 200; // Umbral de detección en mm (20 cm)

  // Marcar obstáculo frontal
  if (d1 > 0 && d1 < TH_MM) {
    markObstacleCellFromSensor(1, d1);
  }
  
  // Marcar obstáculos laterales con validación de consistencia
  bool lateralObstacleMarked = false;
  
  if (d2 > 0 && d2 < TH_MM) {
    int d2_diff = abs(d2_1 - d2_2) + abs(d2_2 - d2_3) + abs(d2_1 - d2_3);
    if (d2_diff < 150) {
      // Calcular posición del obstáculo antes de marcarlo
      int obsX = posX, obsY = posY;
      int hright = (heading + 1) % 4; // Heading derecha
      if (hright == 0) { obsY += 1; }
      else if (hright == 1) { obsX += 1; }
      else if (hright == 2) { obsY -= 1; }
      else { obsX -= 1; }
      
      // Verificar si ya había un obstáculo en esa celda
      bool wasObstacle = false;
      if (obsX >= 0 && obsX < GRID_W && obsY >= 0 && obsY < GRID_H) {
        wasObstacle = (occGrid[obsX][obsY] == 1);
      }
      markObstacleCellFromSensor(3, d2);
      // Si se marcó un nuevo obstáculo, verificar si afecta el path
      if (!wasObstacle && obsX >= 0 && obsX < GRID_W && obsY >= 0 && obsY < GRID_H) {
        if (occGrid[obsX][obsY] == 1) {
          lateralObstacleMarked = true;
        }
      }
    }
  }
  
  if (d3 > 0 && d3 < TH_MM) {
    int d3_diff = abs(d3_1 - d3_2) + abs(d3_2 - d3_3) + abs(d3_1 - d3_3);
    if (d3_diff < 150) { // Si las lecturas son consistentes
      // Calcular posición del obstáculo antes de marcarlo
      int obsX = posX, obsY = posY;
      int hleft = (heading + 3) % 4;
      if (hleft == 0) { obsY += 1; }
      else if (hleft == 1) { obsX += 1; }
      else if (hleft == 2) { obsY -= 1; }
      else { obsX -= 1; }
      
      // Verificar si ya había un obstáculo en esa celda
      bool wasObstacle = false;
      if (obsX >= 0 && obsX < GRID_W && obsY >= 0 && obsY < GRID_H) {
        wasObstacle = (occGrid[obsX][obsY] == 1);
      }
      
      markObstacleCellFromSensor(2, d3);
      
      // Si se marcó un nuevo obstáculo, verificar si afecta el path
      if (!wasObstacle && obsX >= 0 && obsX < GRID_W && obsY >= 0 && obsY < GRID_H) {
        if (occGrid[obsX][obsY] == 1) {
          lateralObstacleMarked = true;
        }
      }
    }
  }
  bool previousObstacleFlag = obstacleFlag;
  if (d1 > 0 && d1 < TH_MM) {
    obstacleFlag = true;
    // Activar efecto LED rojo cuando se detecta un obstáculo
    if (!previousObstacleFlag) {
      ledObstacle = true;
      ledBreathing = ledFlashing = ledRainbow = ledDot = ledCam = false;
    }
  } else {
    obstacleFlag = false;
  }
}

// Marca celda como obstáculo desde lectura de sensor
void markObstacleCellFromSensor(int sensorIndex, int distance_mm) {
  int obsX = posX; // Posición X del obstáculo en celdas
  int obsY = posY; // Posición Y del obstáculo en celdas
  int cellsAway = 1; // Siempre marcar la celda adyacente
  
  if (sensorIndex == 1) {
    if (heading == 0) { obsY += cellsAway; }
    else if (heading == 1) { obsX += cellsAway; }
    else if (heading == 2) { obsY -= cellsAway; }
    else { obsX -= cellsAway; }
  } else if (sensorIndex == 2) {
    int hleft = (heading + 3) % 4; // Heading izquierda
    if (hleft == 0) { obsY += cellsAway; }
    else if (hleft == 1) { obsX += cellsAway; }
    else if (hleft == 2) { obsY -= cellsAway; }
    else { obsX -= cellsAway; }
  } else {
    int hright = (heading + 1) % 4; // Heading derecha
    if (hright == 0) { obsY += cellsAway; }
    else if (hright == 1) { obsX += cellsAway; }
    else if (hright == 2) { obsY -= cellsAway; }
    else { obsX -= cellsAway; }
  }
  if (obsX < 0 || obsX >= GRID_W || obsY < 0 || obsY >= GRID_H) {
    return; // Ignorar detecciones fuera del grid
  }
  if (obsX != posX || obsY != posY) {
    // Solo marcar si no hay ya un obstáculo en esa celda
    if (occGrid[obsX][obsY] == 0) {
      occGrid[obsX][obsY] = 1;
    }
  }
}

// Ejecuta siguiente paso del camino de forma no bloqueante
void executeNextStepFromPathNonBlocking() {
  if (pathIndex >= pathLen - 1) return;
  Cell cur = pathCells[pathIndex]; // Celda actual
  
  int nextIndex = pathIndex + 1; // Índice siguiente
  if (nextIndex >= pathLen) return;
  
  Cell nxt = pathCells[nextIndex]; // Celda siguiente
  int dx = nxt.x - cur.x; // Diferencia X
  int dy = nxt.y - cur.y; // Diferencia Y
  int desiredDir = 0; // Dirección deseada
  if (dx == 0 && dy == 1) desiredDir = 0;
  else if (dx == 1 && dy == 0) desiredDir = 1;
  else if (dx == 0 && dy == -1) desiredDir = 2;
  else if (dx == -1 && dy == 0) desiredDir = 3;

  int diff = (desiredDir - heading + 4) % 4; // Diferencia de dirección
  if (diff != 0) {
    rotateToDirNonBlocking(desiredDir);
    return;
  }
  // Aquí solo verificamos si hay un obstáculo frontal que bloquea el camino
  int dfront1 = sensor1.readRangeContinuousMillimeters(); // Lectura frontal 1
  int dfront2 = sensor1.readRangeContinuousMillimeters(); // Lectura frontal 2
  int dfront = (dfront1 < dfront2) ? dfront1 : dfront2;
  
  if (dfront > 0 && dfront < 200) {
    markObstacleCellFromSensor(1, dfront);
    pathIndex = 0;
    pathLen = 0;
    return;
  }
  queueForwardOneCellNonBlocking();
  pathIndex++;
}

// Rota a dirección deseada
void rotateToDirNonBlocking(int desiredDir) {
  int diff = (desiredDir - heading + 4) % 4; // Diferencia de dirección
  if (diff == 0) return;
  desiredHeadingAfterTurn = desiredDir; // Guardar dirección deseada
  if (diff == 1) {
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, HIGH);
    stepsToDo = turnSteps;
    stepsDone = 0;
    activeMotorCommand = MOTOR_OPPOSITE_D;
    movingFlag = true;
  } else if (diff == 3) {
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, LOW);
    stepsToDo = turnSteps;
    stepsDone = 0;
    activeMotorCommand = MOTOR_OPPOSITE_A;
    movingFlag = true;
  } else if (diff == 2) {
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, HIGH);
    stepsToDo = turnSteps * 2;
    stepsDone = 0;
    activeMotorCommand = MOTOR_OPPOSITE_D;
    movingFlag = true;
  }
}

void queueForwardOneCellNonBlocking() {
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
  stepsToDo = stepsPerCell;
  stepsDone = 0;
  activeMotorCommand = MOTOR_BOTH_CW;
  movingFlag = true;
}

void queueBackwardOneCellNonBlocking() {
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  stepsToDo = stepsPerCell;
  stepsDone = 0;
  activeMotorCommand = MOTOR_BOTH_CCW;
  movingFlag = true;
}

// Calcula stepsPerCell
void calculateStepsPerCell() {
  float stepsPerMeter = (float)stepsPerRev / 0.17f;
  stepsPerCell = (int)round(stepsPerMeter * CELL_SIZE_M);
  if (stepsPerCell < 1) stepsPerCell = 1;
}

// Actualiza pose después de avanzar
void updatePoseAfterForward() {
  if (heading == 0) posY += 1;
  else if (heading == 1) posX += 1;
  else if (heading == 2) posY -= 1;
  else posX -= 1;
  if (posX < 0) posX = 0;
  if (posY < 0) posY = 0;
  if (posX >= GRID_W) posX = GRID_W - 1;
  if (posY >= GRID_H) posY = GRID_H - 1;
}

// Limpia mapa manteniendo pose
void clearMapKeepPose() {
  for (int i=0;i<GRID_W;i++) for (int j=0;j<GRID_H;j++) occGrid[i][j]=0;
  planPathAstar();
}

// Limpia mapa y resetea pose
void clearAllResetPose() {
  for (int i=0;i<GRID_W;i++) for (int j=0;j<GRID_H;j++) occGrid[i][j]=0;
  posX = 0; posY = 0; heading = 0;
  haveGoal = false;
  autoMode = false;
  pathLen = 0;
  pathIndex = 0;
}

// Verifica si hay un obstáculo en la celda al frente
bool checkObstacleInFront() {
  int frontX = posX;
  int frontY = posY;
  
  // Calcular celda al frente basado en heading
  if (heading == 0) { // Norte
    frontY += 1;
  } else if (heading == 1) { // Este
    frontX += 1;
  } else if (heading == 2) { // Sur
    frontY -= 1;
  } else { // Oeste (heading == 3)
    frontX -= 1;
  }
  
  // Verificar límites del grid
  if (frontX < 0 || frontX >= GRID_W || frontY < 0 || frontY >= GRID_H) {
    return true; // Fuera de límites se considera obstáculo
  }
  
  // Verificar si hay obstáculo en esa celda
  return (occGrid[frontX][frontY] == 1);
}

// Función para ejecutar el modo autónomo
void executeAutonomousMode() {
  if (millis() - lastAutoTick < AUTO_STEP_DELAY_MS) {
    return; // Esperar hasta que pase el delay
  }
  lastAutoTick = millis();

  bool previousObstacleFlag = obstacleFlag;
  
  static unsigned long lastTurnCompleteTime = 0;
  static int previousDesiredHeading = -1;
  const unsigned long SENSOR_STABILIZATION_MS = 200; // Delay reducido después de un giro
  
  if (previousDesiredHeading != -1 && desiredHeadingAfterTurn == -1) {
    lastTurnCompleteTime = millis();
  }
  previousDesiredHeading = desiredHeadingAfterTurn;
  
  bool obstaclesDetected = false;
  if (desiredHeadingAfterTurn == -1 && !movingFlag) {
    unsigned long timeSinceTurn = millis() - lastTurnCompleteTime;
    if (timeSinceTurn > SENSOR_STABILIZATION_MS) {
      occupancyMarkFromSensorReadings();
      obstaclesDetected = true; // Se detectaron obstáculos
    }
  }

  bool pathWasRecalculated = false;
  if (obstaclesDetected && !movingFlag && desiredHeadingAfterTurn == -1 && pathLen > 0) {
    bool pathBlocked = false;
    for (int i = pathIndex; i < pathLen; i++) {
      if (occGrid[pathCells[i].x][pathCells[i].y] == 1) {
        pathBlocked = true;
        break;
      }
    }
    if (pathBlocked) {
      pathIndex = 0;
      pathLen = 0;
      planPathAstar();
      pathWasRecalculated = true;
      if (pathLen > 0) {
        // Verificar si la primera celda del nuevo path tiene obstáculo
        if (pathIndex < pathLen && occGrid[pathCells[pathIndex].x][pathCells[pathIndex].y] == 0) {
          obstacleFlag = false;
        }
      }
      return;
    }
  }
  if (obstacleFlag && !previousObstacleFlag) {
    ledObstacle = true;
    ledBreathing = ledFlashing = ledRainbow = ledDot = ledCam = false;
  }
  if (obstacleFlag && !movingFlag && desiredHeadingAfterTurn == -1 && !pathWasRecalculated) {
    int frontX = posX, frontY = posY;
    if (heading == 0) { frontY += 1; }
    else if (heading == 1) { frontX += 1; }
    else if (heading == 2) { frontY -= 1; }
    else { frontX -= 1; }
    
    bool frontCellHasObstacle = false;
    bool frontCellInPath = false;
    if (frontX >= 0 && frontX < GRID_W && frontY >= 0 && frontY < GRID_H) {
      frontCellHasObstacle = (occGrid[frontX][frontY] == 1);
      // Verificar si esa celda está en el path actual
      if (pathLen > 0 && pathIndex < pathLen) {
        if (pathIndex + 1 < pathLen) {
          Cell nextCell = pathCells[pathIndex + 1];
          frontCellInPath = (nextCell.x == frontX && nextCell.y == frontY);
        }
      }
    }
    
    // Solo girar si realmente hay un obstáculo al frente que bloquea el path
    if (!frontCellHasObstacle || !frontCellInPath) {
      obstacleFlag = false;
    } else {
    int dright1 = sensor2.readRangeContinuousMillimeters(); // Sensor derecho
    int dright2 = sensor2.readRangeContinuousMillimeters();
    int dright = (dright1 < dright2) ? dright1 : dright2;
    
    int dleft1 = sensor3.readRangeContinuousMillimeters(); // Sensor izquierdo
    int dleft2 = sensor3.readRangeContinuousMillimeters();
    int dleft = (dleft1 < dleft2) ? dleft1 : dleft2;
    
    const int TH_MM = 150;
    bool leftClear = (dleft == 0 || dleft >= TH_MM);
    bool rightClear = (dright == 0 || dright >= TH_MM);
    
    // Calcular posición del obstáculo lateral izquierdo
    int leftObsX = posX, leftObsY = posY;
    int hleft = (heading + 3) % 4;
    if (hleft == 0) { leftObsY += 1; }
    else if (hleft == 1) { leftObsX += 1; }
    else if (hleft == 2) { leftObsY -= 1; }
    else { leftObsX -= 1; }
    bool leftHasObstacle = false;
    if (leftObsX >= 0 && leftObsX < GRID_W && leftObsY >= 0 && leftObsY < GRID_H) {
      leftHasObstacle = (occGrid[leftObsX][leftObsY] == 1);
    }
    
    // Calcular posición del obstáculo lateral derecho
    int rightObsX = posX, rightObsY = posY;
    int hright = (heading + 1) % 4;
    if (hright == 0) { rightObsY += 1; }
    else if (hright == 1) { rightObsX += 1; }
    else if (hright == 2) { rightObsY -= 1; }
    else { rightObsX -= 1; }
    bool rightHasObstacle = false;
    if (rightObsX >= 0 && rightObsX < GRID_W && rightObsY >= 0 && rightObsY < GRID_H) {
      rightHasObstacle = (occGrid[rightObsX][rightObsY] == 1);
    }
    
    if (leftClear && !rightClear && !leftHasObstacle) {
      // Girar izquierda para evitar obstáculo frontal
      int newDir = (heading + 3) % 4;
      rotateToDirNonBlocking(newDir);
      pathIndex = 0;
      pathLen = 0;
      return; // Retornar inmediatamente para evitar doble giro
    } else if (rightClear && !leftClear && !rightHasObstacle) {
      // Girar derecha para evitar obstáculo frontal
      int newDir = (heading + 1) % 4;
      rotateToDirNonBlocking(newDir);
      pathIndex = 0;
      pathLen = 0;
      return; // Retornar inmediatamente para evitar doble giro
    } else if (leftClear && rightClear) {
      // Ambos lados libres - preferir el que NO tiene obstáculo marcado
      if (!rightHasObstacle) {
        int newDir = (heading + 1) % 4;
        rotateToDirNonBlocking(newDir);
        pathIndex = 0;
        pathLen = 0;
        return; // Retornar inmediatamente para evitar doble giro
      } else if (!leftHasObstacle) {
        int newDir = (heading + 3) % 4;
        rotateToDirNonBlocking(newDir);
        pathIndex = 0;
        pathLen = 0;
        return; // Retornar inmediatamente para evitar doble giro
      } else {
        // Ambos tienen obstáculos marcados, preferir derecha
        int newDir = (heading + 1) % 4;
        rotateToDirNonBlocking(newDir);
        pathIndex = 0;
        pathLen = 0;
        return; // Retornar inmediatamente para evitar doble giro
      }
    } else {
      // No hay espacio en los lados, recalcular path con obstáculos marcados
      pathIndex = 0;
      pathLen = 0;
      if (desiredHeadingAfterTurn == -1) {
        planPathAstar();
      }
      return;
      }
    }
  }
  else if (previousObstacleFlag && !obstacleFlag && !movingFlag && pathLen == 0 && desiredHeadingAfterTurn == -1) {
    planPathAstar();
  }
    // Si evasionMode está activo por alguna razón, desactivarlo y recalcular
  if (evasionMode && !movingFlag && desiredHeadingAfterTurn == -1) {
    evasionMode = false;
    evasionCellsCount = 0;
    planPathAstar();
    return;
  }
  else if (pathLen == 0 && !movingFlag && desiredHeadingAfterTurn == -1 && !evasionMode && !obstacleFlag) {
    planPathAstar();
    if (pathLen == 0) {
      if (obstacleFlag) {
      } else {
        autoMode = false;
        autoPaused = true;
        Serial.println("AUTO_NOPATH");
      }
    }
  } else {
    if (!movingFlag && desiredHeadingAfterTurn == -1 && !evasionMode) {
      if (pathIndex >= pathLen - 1) {
        autoMode = false;
        autoPaused = true;
        haveGoal = false;
        Serial.println("AUTO_ARRIVED");
      } else if (pathLen > 0) {
        executeNextStepFromPathNonBlocking();
      }
    }
  }
}
