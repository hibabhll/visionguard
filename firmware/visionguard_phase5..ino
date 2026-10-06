#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define LED_PIN      15
#define BUZZER_PIN   25
#define TRIG_PIN     5
#define ECHO_PIN     18
#define SDA_PIN      21
#define SCL_PIN      22

#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASS       ""
#define MQTT_BROKER     "broker.hivemq.com"
#define MQTT_PORT       1883
#define TOPIC_TELEMETRY "hiba/visionguard/telemetry"
#define TOPIC_STATUS    "hiba/visionguard/status"
#define TOPIC_ALERT     "hiba/visionguard/alert"
#define TELEMETRY_MS    2000

#define BATTERY_DRAIN_MS 10000

#define MPU_ADDR         0x68
#define FALL_FACTOR      2.2
#define CALM_LOW_FACTOR  0.85
#define CALM_HIGH_FACTOR 1.35
#define RECOVERY_MS      2000

#define DIST_CRITICAL 40
#define DIST_WARNING  100

QueueHandle_t distanceQueue;
QueueHandle_t fallQueue;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

volatile int g_distance = -1;
volatile int g_fall     = 0;
volatile int g_state    = 0;
volatile int g_battery  = 100;

bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

bool mpuReadAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / 16384.0;
  ay = ry / 16384.0;
  az = rz / 16384.0;
  return true;
}

void sensorTask(void *pv) {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  while (true) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    int  distance = (duration == 0) ? 999 : (duration * 0.034 / 2);

    xQueueSend(distanceQueue, &distance, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void fallTask(void *pv) {
  float ax, ay, az;

  if (!mpuInit()) {
    while (true) {
      Serial.println("[FALL] ERROR: MPU6050 not found on I2C. Check VCC, GND, SDA=21, SCL=22.");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  float baseline = 0;
  int   samples  = 0;
  for (int i = 0; i < 20; i++) {
    if (mpuReadAccel(ax, ay, az)) {
      baseline += sqrt(ax*ax + ay*ay + az*az);
      samples++;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (samples == 0) {
    while (true) {
      Serial.println("[FALL] ERROR: I2C reads failing. Check SDA/SCL wires (maybe swapped).");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  baseline /= samples;
  Serial.print("[FALL] Calibration OK. Baseline = ");
  Serial.println(baseline);

  float fallThresh = baseline * FALL_FACTOR;
  float calmLow    = baseline * CALM_LOW_FACTOR;
  float calmHigh   = baseline * CALM_HIGH_FACTOR;

  bool     fall      = false;
  uint32_t calmSince = 0;
  uint32_t lastPrint = 0;

  while (true) {
    if (mpuReadAccel(ax, ay, az)) {
      float mag = sqrt(ax*ax + ay*ay + az*az);

      if (millis() - lastPrint > 1000) {
        Serial.print("[FALL] mag = ");
        Serial.println(mag);
        lastPrint = millis();
      }

      if (!fall && mag > fallThresh) {
        fall   = true;
        g_fall = 1;
        int ev = 1;
        xQueueSend(fallQueue, &ev, 0);
        Serial.println("[FALL] *** FALL DETECTED ***");
      }
      else if (fall) {
        if (mag > calmLow && mag < calmHigh) {
          if (calmSince == 0) calmSince = millis();
          else if (millis() - calmSince > RECOVERY_MS) {
            fall      = false;
            calmSince = 0;
            g_fall    = 0;
            int ev = 0;
            xQueueSend(fallQueue, &ev, 0);
            Serial.println("[FALL] User stable again -> NORMAL");
          }
        } else {
          calmSince = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void alertTask(void *pv) {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  int dist = 999;
  int fall = 0;
  int lastState = -1;

  while (true) {
    int tmp;
    if (xQueueReceive(distanceQueue, &tmp, 0) == pdPASS) dist = tmp;
    if (xQueueReceive(fallQueue,     &tmp, 0) == pdPASS) fall = tmp;

    g_distance = dist;

    int state;
    if (fall)                                              state = 3;
    else if (dist > 0 && dist <= DIST_CRITICAL)            state = 2;
    else if (dist > DIST_CRITICAL && dist <= DIST_WARNING) state = 1;
    else                                                   state = 0;

    g_state = state;

    if (state != lastState) {
      switch (state) {
        case 3:  Serial.println("[ALERT] State -> FALL (siren)"); break;
        case 2:  Serial.println("[ALERT] State -> CRITICAL (continuous beep)"); break;
        case 1:  Serial.println("[ALERT] State -> WARNING (slow beeps)"); break;
        default: Serial.println("[ALERT] State -> NORMAL (silent)"); break;
      }
      lastState = state;
    }

    switch (state) {
      case 3: {
        uint32_t t  = millis() % 800;
        bool     on = (t < 100) || (t >= 200 && t < 300);
        if (on) tone(BUZZER_PIN, 2200); else tone(BUZZER_PIN, 1400);
        digitalWrite(LED_PIN, on ? HIGH : LOW);
        break;
      }
      case 2:
        tone(BUZZER_PIN, 2500);
        digitalWrite(LED_PIN, HIGH);
        break;
      case 1: {
        uint32_t t  = millis() % 1000;
        bool     on = (t < 150);
        if (on) tone(BUZZER_PIN, 1000); else noTone(BUZZER_PIN);
        digitalWrite(LED_PIN, on ? HIGH : LOW);
        break;
      }
      default:
        noTone(BUZZER_PIN);
        digitalWrite(LED_PIN, LOW);
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void heartbeatTask(void *pv) {
  while (true) {
    Serial.print("[HEARTBEAT] up=");
    Serial.print(millis() / 1000);
    Serial.print("s dist=");
    Serial.print(g_distance);
    Serial.print("cm fall=");
    Serial.print(g_fall);
    Serial.print(" batt=");
    Serial.print(g_battery);
    Serial.print("% state=");
    switch (g_state) {
      case 3:  Serial.println("FALL"); break;
      case 2:  Serial.println("CRITICAL"); break;
      case 1:  Serial.println("WARNING"); break;
      default: Serial.println("NORMAL"); break;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void mqttTask(void *pv) {
  Serial.print("[WIFI] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println();
  Serial.print("[WIFI] Connected. IP = ");
  Serial.println(WiFi.localIP());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  String clientId = "visionguard-hiba-" + String(random(0xffff), HEX);

  while (!mqtt.connected()) {
    Serial.print("[MQTT] Connecting... ");
    bool ok = mqtt.connect(clientId.c_str(),
                           TOPIC_STATUS, 1, true, "offline");
    if (ok) {
      Serial.println("connected.");
      mqtt.publish(TOPIC_STATUS, "online", true);
    } else {
      Serial.print("failed rc=");
      Serial.print(mqtt.state());
      Serial.println(" -> retry in 2 s");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  uint32_t lastPub   = 0;
  uint32_t lastDrain = 0;
  int      lastSent  = -1;

  while (true) {
    if (!mqtt.connected()) {
      Serial.println("[MQTT] Link lost, reconnecting...");
      while (!mqtt.connected()) {
        mqtt.connect(clientId.c_str(), TOPIC_STATUS, 1, true, "offline");
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      mqtt.publish(TOPIC_STATUS, "online", true);
    }
    mqtt.loop();

    if ((int)g_state != lastSent) {
      lastSent = g_state;
      char alert[96];
      snprintf(alert, sizeof(alert),
               "{\"state\":%d,\"distance_cm\":%d,\"fall\":%d,\"battery_pct\":%d}",
               lastSent, (int)g_distance, (int)g_fall, (int)g_battery);
      mqtt.publish(TOPIC_ALERT, alert);
      Serial.print("[MQTT] Alert published: ");
      Serial.println(alert);
    }

    if (millis() - lastDrain > BATTERY_DRAIN_MS) {
      lastDrain = millis();
      if (g_battery > 1) g_battery -= 1;
      if (g_state != 0 && g_battery > 1) g_battery -= 1;
    }

    if (millis() - lastPub > TELEMETRY_MS) {
      lastPub = millis();
      char payload[192];
      snprintf(payload, sizeof(payload),
               "{\"distance_cm\":%d,\"fall\":%d,\"state\":%d,\"battery_pct\":%d,\"uptime_s\":%lu,\"rssi\":%d}",
               (int)g_distance, (int)g_fall, (int)g_state, (int)g_battery,
               (unsigned long)(millis() / 1000), WiFi.RSSI());
      mqtt.publish(TOPIC_TELEMETRY, payload);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 2000); digitalWrite(LED_PIN, HIGH); delay(250);
    noTone(BUZZER_PIN);     digitalWrite(LED_PIN, LOW);  delay(250);
  }
  Serial.println("[SELFTEST] Done. Expected: 3 beeps + 3 LED blinks.");

  distanceQueue = xQueueCreate(5, sizeof(int));
  fallQueue     = xQueueCreate(5, sizeof(int));

  xTaskCreate(sensorTask,    "SENSOR",    4096, NULL, 2, NULL);
  xTaskCreate(fallTask,      "FALL",      4096, NULL, 2, NULL);
  xTaskCreate(alertTask,     "ALERT",     4096, NULL, 2, NULL);
  xTaskCreate(heartbeatTask, "HEARTBEAT", 4096, NULL, 1, NULL);
  xTaskCreate(mqttTask,      "CLOUD",     8192, NULL, 2, NULL);

  Serial.println("[VisionGuard] Phase 5 FINAL started - 5 tasks running");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));
}