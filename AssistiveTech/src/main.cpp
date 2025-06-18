// === Blynk Configuration ===
#define BLYNK_TEMPLATE_ID "TMPL6YE7TP4yd"
#define BLYNK_TEMPLATE_NAME "Fajr Buddy"
#define BLYNK_AUTH_TOKEN "MUuy_EgQAWb7IJtLAnj937OR7qrf_fF4"

// === Libraries ===
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>

// === Wi-Fi Credentials ===
char ssid[] = "Irf";
char pass[] = "123456777";

// === MQTT Setup ===
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// === Pin Definitions ===
// Analogue Inputs
const uint8_t LDR1_PIN = 36;   // Input: Ambient light
const uint8_t LDR2_PIN = 32;   // Input: Output light for PID

// Outputs
const uint8_t LED_PIN = 23;    // Output: PWM LED
const uint8_t BUZZER_PIN = 22; // Output: Tone

// Inputs (buttons)
const uint8_t BUTTON1_PIN = 21; // Toggle manual mode
const uint8_t BUTTON2_PIN = 19; // Toggle LED in manual mode

// === Mode Flags ===
bool isManual = false;
bool manualLEDState = false;

// === PID Variables ===
float kp = 0.5;
float ki = 0.05;
float kd = 0.1;

float setpoint = 0.0;
float input = 0.0;
float output = 0.0;
float previousError = 0.0;
float integral = 0.0;

const int maxBrightness = 255;

// === Debounce Timing ===
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
const unsigned long debounceDelay = 300; 

// === Time Client (NTP) ===
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8 * 3600, 60000); 

// === Subuh Alarm ===
bool subuhTriggered = false;

// === Demo Mode ===
bool useDemoTimes = true;
String demoSubuhTime = "10:18";

// === MQTT Reconnect Function ===
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// === Blynk Virtual Pin Control ===
BLYNK_WRITE(V0) { // Manual mode switch
  isManual = param.asInt();
  Serial.println(isManual ? "Manual mode ENABLED (Blynk)" : "Auto mode ENABLED (Blynk)");
}

BLYNK_WRITE(V1) { // Manual LED control
  if (isManual) {
    manualLEDState = param.asInt();
    ledcWrite(0, manualLEDState ? maxBrightness : 0);
    Serial.print("Manual LED (Blynk): ");
    Serial.println(manualLEDState ? "ON" : "OFF");
  }
}

// === Setup Function ===
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pin Modes
  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // LED PWM Setup
  ledcSetup(0, 5000, 8);         // Channel 0, 5kHz, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);

  // Wi-Fi Connection
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timeClient.begin();
  client.setServer(mqtt_server, 1883);

  Serial.println("System initialized.");
}

// === Helper: Time Match ===
bool isTimeEqual(String current, String target) {
  return current == target;
}

// === Main Loop ===
void loop() {
  Blynk.run();

  unsigned long currentTime = millis();

  // Update NTP time
  timeClient.update();
  String formattedTime = timeClient.getFormattedTime().substring(0, 5); // HH:MM

  // === BUTTON 1: Manual Mode Toggle ===
  if (digitalRead(BUTTON1_PIN) == LOW && (currentTime - lastButton1Press > debounceDelay)) {
    isManual = !isManual;
    Blynk.virtualWrite(V0, isManual); // Sync with app
    Serial.println(isManual ? "Manual mode ENABLED (Button)" : "Auto mode ENABLED (Button)");
    lastButton1Press = currentTime;
  }

  // === BUTTON 2: Manual LED Toggle ===
  if (isManual && digitalRead(BUTTON2_PIN) == LOW && (currentTime - lastButton2Press > debounceDelay)) {
    manualLEDState = !manualLEDState;
    ledcWrite(0, manualLEDState ? maxBrightness : 0);
    Blynk.virtualWrite(V1, manualLEDState);
    Serial.print("Manual LED (Button): ");
    Serial.println(manualLEDState ? "ON" : "OFF");
    lastButton2Press = currentTime;
  }

  // === Sensor Readings ===
  int ldr1 = analogRead(LDR1_PIN); // ambient light input
  int ldr2 = analogRead(LDR2_PIN); // output light for feedback

  // === Auto Mode (PID Control) ===
  if (!isManual) {
    if (ldr1 >= 4000) {
      setpoint = maxBrightness;
      tone(BUZZER_PIN, 1000); // Alarm ON
    } else {
      setpoint = 0;
      noTone(BUZZER_PIN);     // Alarm OFF
    }

    input = map(ldr2, 0, 4095, 0, maxBrightness);
    float error = setpoint - input;
    integral += error;
    float derivative = error - previousError;
    output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, 0, maxBrightness);
    previousError = error;

    if (ldr1 >= 4000) {
      ledcWrite(0, (int)output);
    } else {
      ledcWrite(0, 0); // LED OFF
    }
  }

  // === Subuh Alarm Trigger ===
  String subuhTime = useDemoTimes ? demoSubuhTime : "05:45";
  if (!subuhTriggered && isTimeEqual(formattedTime, subuhTime)) {
    Serial.println("SUBUH ALARM! Triggering light + buzzer.");
    ledcWrite(0, maxBrightness);
    tone(BUZZER_PIN, 1000);
    subuhTriggered = true;
  }

  // === Serial Debug ===
  Serial.print("Time: "); Serial.print(formattedTime);
  Serial.print(" | LDR1: "); Serial.print(ldr1);
  Serial.print(" | LDR2: "); Serial.print(ldr2);
  Serial.print(" | Setpoint: "); Serial.print(setpoint);
  Serial.print(" | Output: "); Serial.println(output);

  // === Blynk Updates ===
  Blynk.virtualWrite(V2, ldr1);
  Blynk.virtualWrite(V3, ldr2);
  Blynk.virtualWrite(V4, isManual ? "Manual" : "Auto");
  Blynk.virtualWrite(V5, formattedTime);
  Blynk.virtualWrite(V6, subuhTriggered ? 1 : 0);

  // === MQTT ===
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // MQTT Payload Format
  String payload = "{";
  payload += "\"time\":\"" + formattedTime + "\",";
  payload += "\"ldr1\":" + String(ldr1) + ",";
  payload += "\"ldr2\":" + String(ldr2) + ",";
  payload += "\"mode\":\"" + String(isManual ? "Manual" : "Auto") + "\",";
  payload += "\"led\":" + String(isManual ? manualLEDState : (ldr1 >= 4000 ? 1 : 0)) + ",";
  payload += "\"subuh_triggered\":" + String(subuhTriggered ? "true" : "false");
  payload += "}";

  client.publish("esp32/status", payload.c_str());

  delay(500);
}
