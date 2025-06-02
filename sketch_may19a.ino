#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi Credentials
const char* ssid = "Knock";
const char* password = "kgpc2286";

// WebSocket Config
const char* ws_host = "192.168.25.168";
const uint16_t ws_port = 3000;
const char* ws_path = "/";

// WebSocket
WebSocketsClient webSocket;

// GPIO Pins
#define IR_SENSOR_PIN       26
#define IR_SENSOR_OUT_PIN   27
#define SERVO_PIN           25
#define SERVO_OUT_PIN       19
#define IR_A1_PIN           12
#define IR_A2_PIN           13
#define IR_A3_PIN           14
#define IR_A4_PIN           15
#define IR_B1_PIN           16
#define IR_B2_PIN           17

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// State Variables
bool detectionSent = false;
bool detectionOutSent = false;
bool servoInRotated = false;
bool servoOutRotated = false;
unsigned long lastObjectSeenTime = 0;
unsigned long lastObjectOutSeenTime = 0;
unsigned long resetDelay = 2000;
bool isFull = false;

// Slot States
bool a1LastState = false, a2LastState = false, a3LastState = false, a4LastState = false;
bool b1LastState = false, b2LastState = false;

// Servo Motors
Servo servoIn;
Servo servoOut;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  setupPins();
  setupServos();
  connectToWiFi();
  setupWebSocket();
}

void setupPins() {
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_OUT_PIN, INPUT);
  pinMode(IR_A1_PIN, INPUT_PULLUP);
  pinMode(IR_A2_PIN, INPUT_PULLUP);
  pinMode(IR_A3_PIN, INPUT_PULLUP);
  pinMode(IR_A4_PIN, INPUT_PULLUP);
  pinMode(IR_B1_PIN, INPUT_PULLUP);
  pinMode(IR_B2_PIN, INPUT_PULLUP);
}

void setupServos() {
  servoIn.attach(SERVO_PIN, 500, 2400);
  servoOut.attach(SERVO_OUT_PIN, 500, 2400);
  servoIn.write(0);
  servoOut.write(0);
}

void connectToWiFi() {
  Serial.printf("[WiFi] Connecting to %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected.");
  } else {
    Serial.println("\n[WiFi] Failed to connect!");
    while (true) delay(1000);
  }
}

void setupWebSocket() {
  webSocket.begin(ws_host, ws_port, ws_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

void loop() {
  webSocket.loop();
  updateSlotStates();
  checkFullStatus();
  handleDetection();
  handleDetectionOut();
  handleServoReset();
  handleDisplay();
}

// Check bãi đỗ đầy
void checkFullStatus() {
  isFull = a1LastState && a2LastState && a3LastState && a4LastState && b1LastState && b2LastState;
  if (isFull) {
    Serial.println("[SYSTEM] Bãi đỗ FULL.");
  }
}

// Cập nhật trạng thái từng vị trí
void updateSlotStates() {
  checkAndSendSlot(IR_A1_PIN, "A1", a1LastState);
  checkAndSendSlot(IR_A2_PIN, "A2", a2LastState);
  checkAndSendSlot(IR_A3_PIN, "A3", a3LastState);
  checkAndSendSlot(IR_A4_PIN, "A4", a4LastState);
  checkAndSendSlot(IR_B1_PIN, "B1", b1LastState);
  checkAndSendSlot(IR_B2_PIN, "B2", b2LastState);
}

void checkAndSendSlot(int pin, const char* slotName, bool& lastState) {
  bool currentState = (digitalRead(pin) == LOW);
  if (currentState != lastState) {
    lastState = currentState;
    sendSlotStatus(slotName, currentState);
  }
}

void sendSlotStatus(const char* slot, bool inUse) {
  if (webSocket.isConnected()) {
    StaticJsonDocument<128> doc;
    doc["event"] = "slot_status";
    doc["slot"] = slot;
    doc["in_use"] = inUse;
    String jsonStr;
    serializeJson(doc, jsonStr);
    webSocket.sendTXT(jsonStr);
    Serial.printf("[WS] Slot %s: %s\n", slot, inUse ? "IN_USE" : "AVAILABLE");
  }
}

// Phát hiện xe vào
void handleDetection() {
  bool detectionNow = (digitalRead(IR_SENSOR_PIN) == LOW);
  unsigned long currentMillis = millis();
  if (detectionNow) {
    lastObjectSeenTime = currentMillis;
    if (!detectionSent) {
      sendDetection("detection");
      detectionSent = true;
    }
  } else if (currentMillis - lastObjectSeenTime > resetDelay) {
    detectionSent = false;
  }
}

// Phát hiện xe ra
void handleDetectionOut() {
  bool detectionNow = (digitalRead(IR_SENSOR_OUT_PIN) == LOW);
  unsigned long currentMillis = millis();
  if (detectionNow) {
    lastObjectOutSeenTime = currentMillis;
    if (!detectionOutSent) {
      sendDetection("detection_out");
      detectionOutSent = true;
    }
  } else if (currentMillis - lastObjectOutSeenTime > resetDelay) {
    detectionOutSent = false;
  }
}

void sendDetection(const char* eventType) {
  Serial.printf("[WS] Sending event: %s\n", eventType);
  if (webSocket.isConnected()) {
    StaticJsonDocument<128> doc;
    doc["event"] = eventType;
    String jsonStr;
    serializeJson(doc, jsonStr);
    webSocket.sendTXT(jsonStr);
  }
}

// Reset Servo
void handleServoReset() {
  unsigned long currentMillis = millis();
  if (servoInRotated && (currentMillis - lastObjectSeenTime > resetDelay)) {
    Serial.println("[Servo In] Resetting");
    servoIn.write(0);
    servoInRotated = false;
  }
  if (servoOutRotated && (currentMillis - lastObjectOutSeenTime > resetDelay)) {
    Serial.println("[Servo Out] Resetting");
    servoOut.write(0);
    servoOutRotated = false;
  }
}

// OLED Hiển thị
void handleDisplay() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("|"); display.print(a1LastState ? "X" : "O");
  display.print("| A |"); display.print(a2LastState ? "X" : "O"); display.print("|");

  display.setCursor(0, 22);
  display.print("|"); display.print(a3LastState ? "X" : "O");
  display.print("|   |"); display.print(a4LastState ? "X" : "O"); display.print("|");

  display.setCursor(0, 44);
  display.print("|"); display.print(b1LastState ? "X" : "O");
  display.print("| B |"); display.print(b2LastState ? "X" : "O"); display.print("|");

  display.display();
}

// WebSocket Xử lý
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.printf("[WS] Received: %s\n", payload);
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print("[JSON] Parse error: ");
      Serial.println(error.c_str());
      return;
    }

    const char* event = doc["event"];
    const char* command = doc["command"];
    const char* target = doc["target"];

    if (strcmp(command, "rotate") == 0) {
      if (target) {
        if (strcmp(target, "in") == 0 && !servoInRotated) {
          rotateServo(servoIn, servoInRotated, "[Servo In]");
        } else if (strcmp(target, "out") == 0 && !servoOutRotated) {
          rotateServo(servoOut, servoOutRotated, "[Servo Out]");
        } else {
          Serial.println("[WS] Invalid target or already rotated");
        }
      }
    }
  } else if (type == WStype_CONNECTED) {
    Serial.println("[WS] Connected to server");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("[WS] Disconnected");
  }
}

void rotateServo(Servo& servo, bool& rotatedFlag, const char* logPrefix) {
  Serial.printf("%s Rotating 90 degrees\n", logPrefix);
  servo.write(90);
  rotatedFlag = true;
}
