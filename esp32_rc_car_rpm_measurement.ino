#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi credentials
const char* ssid = "Ayken";
const char* password = "Jake1044";

// Motor control pins
const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

// Encoder pins
const int encoder1 = 34;
const int encoder2 = 35;

// Tick counters
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;

// RPM tracking
unsigned long lastRPMUpdate = 0;
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;
const int ticksPerRevolution = 20;  // Adjust based on encoder specs

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Function declarations
void movement(int cmdValue);
void Forward();
void Backward();
void Left();
void Right();
void stopbot();
void IRAM_ATTR onEncoder1Tick();
void IRAM_ATTR onEncoder2Tick();
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);

  // Setup SoftAP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);

  // WebSocket setup
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // Motor pins
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);

  // Encoder pins
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder1Tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), onEncoder2Tick, RISING);

  stopbot();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastRPMUpdate >= 5) {
    noInterrupts();
    int ticks1 = encoder1Ticks;
    int ticks2 = encoder2Ticks;
    encoder1Ticks = 0;
    encoder2Ticks = 0;
    interrupts();

    rpmMotor1 = (ticks1 / (float)ticksPerRevolution) * 120.0;
    rpmMotor2 = (ticks2 / (float)ticksPerRevolution) * 120.0;

    Serial.printf("RPM1: %.2f | RPM2: %.2f\n", rpmMotor1, rpmMotor2);

    String rpmData = "RPM," + String(rpmMotor1, 2) + "," + String(rpmMotor2, 2);
    ws.textAll(rpmData);

    lastRPMUpdate = currentTime;
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    int message = atoi((char*)data);
    Serial.print("Received: ");
    Serial.println(message);

    // Send acknowledgment safely
    String ack = "Message received: " + String(message);
    client->text(ack);

    movement(message);
  }
}

// Motor movement logic
void movement(int cmdValue) {
  switch (cmdValue) {
    case 1:
      Serial.println("Executing Forward");
      Forward();
      break;
    case 3:
      Serial.println("Executing Backward");
      Backward();
      break;
    case 2:
      Serial.println("Executing Left Turn");
      Left();
      break;
    case 4:
      Serial.println("Executing Right Turn");
      Right();
      break;
    case 5:
      Serial.println("Stopping");
      stopbot();
      break;
    default:
      Serial.println("Unknown Command ++ stopping");
      stopbot();
      break;
  }
}

void Forward() {
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}

void Backward() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}

void Left() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}

void Right() {
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}

void stopbot() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, LOW);
}

// Encoder ISRs
void IRAM_ATTR onEncoder1Tick() {
  encoder1Ticks++;
}

void IRAM_ATTR onEncoder2Tick() {
  encoder2Ticks++;
}
