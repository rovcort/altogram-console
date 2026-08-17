#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <PubSubClient.h>
#include <FlashStorage.h>
#include <math.h>

// ---------------------------------------------------------------
// CONFIGURABLE CREDENTIALS (no longer hardcoded)
// ---------------------------------------------------------------
// Credentials are now stored in flash (via FlashStorage) instead of
// being baked into the sketch. On first boot (or after a factory
// reset) the device will prompt for them once over Serial, then
// remember them across power cycles / re-uploads of this same sketch.
//
// To reconfigure later: open Serial Monitor at 115200 baud, and
// within 5 seconds of boot send the character  r  (lowercase r) plus
// Enter. This clears saved config and re-runs the setup wizard.
//
// Install "FlashStorage" by Arduino via Library Manager if you don't
// already have it (works on SAMD boards, which the Nano 33 IoT is).
// ---------------------------------------------------------------

struct Credentials {
  bool valid;              // set true once populated
  char ssid[33];
  char wifiPass[65];
  char mqttServer[65];
  int  mqttPort;
  char mqttUser[33];
  char mqttPass[33];
};

FlashStorage(credStore, Credentials);
Credentials creds;

WiFiSSLClient wiFiClient;
PubSubClient mqttClient(wiFiClient);

bool lastAlertState = false;

const unsigned long SAMPLE_INTERVAL_MS = 10; // 100 Hz
const float THRESHOLD_RATIO = 2.5;

// STA / LTA Coefficients
const float ALPHA_STA = 0.20;
const float ALPHA_LTA = 0.005;

const int MIN_TRIGGER_SAMPLES = 10;
int triggerCounter = 0;

// Buzzer Alarm
const int buzzer = 4;
unsigned long previousAlarmMillis = 0;
bool alarmState = LOW;

const int relayPin = 8;
// WiFi reconnect tracking (loop() never re-called WiFi.begin() after boot,
// so a dropped connection stayed dropped forever - see loop() below)
unsigned long lastWifiAttempt = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000;

// Single 2nd-order Butterworth Bandpass Filter (0.5Hz - 10Hz at 100Hz)
struct BiquadFilter {
  float b0 = 0.067455, b1 = 0.0, b2 = -0.067455;
  float a1 = -1.224747, a2 = 0.465091;
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  float process(float in) {
    float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = in;
    y2 = y1; y1 = out;
    return out;
  }
} filter3D;

float sta = 0.0, lta = 0.02;
unsigned long lastSampleTime = 0;

// Peak Ground Acceleration (PGA) Tracking Window (100 samples = 1 second)
float pgaWindowMax = 0.0;
int pgaSampleCount = 0;
const int PGA_WINDOW_SAMPLES = 100;
const char* currentIntensityStr = "I (Imperceptible)";

float activeAlertPeakPGA = 0.0;
const char* lastPrintedIntensity = "";

// Maps Peak Ground Acceleration (PGA in g) to Modified Mercalli Intensity (MMI)
const char* estimateMMI(float pga_g) {
  float pga_pct = pga_g * 100.0; // Convert g to %g
  if (pga_pct < 0.17) return "I (Imperceptible)";
  if (pga_pct < 1.4)  return "II-III (Weak)";
  if (pga_pct < 3.9)  return "IV (Light Shaking)";
  if (pga_pct < 9.2)  return "V (Moderate Shaking)";
  if (pga_pct < 18.0) return "VI (Strong Shaking)";
  if (pga_pct < 34.0) return "VII (Very Strong)";
  return "VIII+ (Destructive)";
}

unsigned long getBeepInterval(const char* intensity) {
  if (strcmp(intensity, "VIII+ (Destructive)") == 0) return 50;
  if (strcmp(intensity, "VI (Strong Shaking)") == 0) return 200;
  if (strcmp(intensity, "IV (Light Shaking)") == 0) return 500;

  return 800; // mild shaking II-III
}

// ---------------------------------------------------------------
// Serial config helpers
// ---------------------------------------------------------------

// Blocking read of one line from Serial into buf (max len maxLen-1).
// Trims trailing \r\n. Returns length read.
int readSerialLine(char* buf, int maxLen) {
  int idx = 0;
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (idx > 0) break;   // ignore leading CR/LF
        else continue;
      }
      if (idx < maxLen - 1) {
        buf[idx++] = c;
      }
    }
  }
  buf[idx] = '\0';
  return idx;
}

int readSerialInt(int defaultVal) {
  char tmp[16];
  int len = readSerialLine(tmp, sizeof(tmp));
  if (len == 0) return defaultVal;
  int v = atoi(tmp);
  return v == 0 ? defaultVal : v;
}

void runConfigWizard() {
  Serial.println();
  Serial.println("==================================================");
  Serial.println(" ALTOGRAM CONFIGURATION WIZARD");
  Serial.println(" Enter each value then press Enter.");
  Serial.println("==================================================");

  Serial.print("WiFi SSID: ");
  readSerialLine(creds.ssid, sizeof(creds.ssid));
  Serial.println(creds.ssid);

  Serial.print("WiFi Password: ");
  readSerialLine(creds.wifiPass, sizeof(creds.wifiPass));
  Serial.println("[hidden]");

  Serial.print("MQTT Broker Host (e.g. xxxx.hivemq.cloud): ");
  readSerialLine(creds.mqttServer, sizeof(creds.mqttServer));
  Serial.println(creds.mqttServer);

  Serial.print("MQTT Broker Port [8883]: ");
  creds.mqttPort = readSerialInt(8883);
  Serial.println(creds.mqttPort);

  Serial.print("MQTT Username: ");
  readSerialLine(creds.mqttUser, sizeof(creds.mqttUser));
  Serial.println(creds.mqttUser);

  Serial.print("MQTT Password: ");
  readSerialLine(creds.mqttPass, sizeof(creds.mqttPass));
  Serial.println("[hidden]");

  creds.valid = true;
  credStore.write(creds);

  Serial.println("--------------------------------------------------");
  Serial.println(" Configuration saved to flash. Continuing boot...");
  Serial.println("--------------------------------------------------");
}

// Gives the user a short window at boot to type 'r' + Enter to force
// reconfiguration even if valid credentials are already stored.
bool userRequestedReset() {
  Serial.println("Send 'r' within 5s to reconfigure WiFi/MQTT credentials...");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'r' || c == 'R') {
        // drain rest of line
        while (Serial.available()) Serial.read();
        return true;
      }
    }
  }
  return false;
}

void loadOrConfigureCredentials() {
  creds = credStore.read();

  bool wantsReset = false;
  if (Serial) {
    wantsReset = userRequestedReset();
  }

  if (!creds.valid || wantsReset) {
    if (!creds.valid) {
      Serial.println("No stored credentials found.");
    }
    runConfigWizard();
  } else {
    Serial.println("Loaded saved WiFi/MQTT credentials from flash.");
    Serial.print("  SSID: "); Serial.println(creds.ssid);
    Serial.print("  MQTT Host: "); Serial.println(creds.mqttServer);
    Serial.print("  MQTT Port: "); Serial.println(creds.mqttPort);
  }
}

// Reconnect to MQTT with LWT (Last Will and Testament)
void reconnectMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(creds.mqttServer);
    Serial.print("...");

    const char* clientID = "SeismicSensorClient";
    const char* willTopic = "seismic/system";
    int willQoS = 1;
    bool willRetain = true;
    const char* willMessage = "{\"status\":\"disconnected\"}";

    if (mqttClient.connect(clientID, creds.mqttUser, creds.mqttPass, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("CONNECTED");
      char statusPayload[128];
      const char* status = "connected";
      snprintf(statusPayload, sizeof(statusPayload), "{\"status\":\"%s\"}", status);
      mqttClient.publish("seismic/system", statusPayload, true);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Will retry next loop execution");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  loadOrConfigureCredentials();

  if (!IMU.begin()) while (1);

  WiFi.begin(creds.ssid, creds.wifiPass);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) {
    Serial.println("WiFi Status: NOT CONNECTED");
    Serial.println("Mode: OFFLINE");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Status: CONNECTED");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  mqttClient.setServer(creds.mqttServer, creds.mqttPort);

  Serial.println("System Initialized. Monitoring for seismic activity...");

  pinMode(buzzer, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  if (WiFi.status() != WL_CONNECTED) {
    // Previously: nothing here. If WiFi dropped (or never connected within
    // the 5s boot window), the device stayed offline permanently. Now it
    // retries every WIFI_RETRY_INTERVAL_MS without blocking the sensor loop.
    if (currentTime - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiAttempt = currentTime;
      Serial.println("WiFi disconnected - attempting reconnect...");
      WiFi.begin(creds.ssid, creds.wifiPass);
    }
  } else {
    if(!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
  }

  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;

    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);

      float totalMag = sqrt(ax * ax + ay * ay + az * az);
      float filteredSignal = fabs(filter3D.process(totalMag));

      sta = (ALPHA_STA * filteredSignal) + ((1.0 - ALPHA_STA) * sta);

      float safeLta = (lta < 0.001) ? 0.001 : lta;
      float ratio = sta / safeLta;

      bool isWarmedUp = (currentTime > 3000);
      if (isWarmedUp && ratio < THRESHOLD_RATIO) {
        lta = (ALPHA_LTA * filteredSignal) + ((1.0 - ALPHA_LTA) * lta);
      }

      if (ratio >= THRESHOLD_RATIO) {
        triggerCounter++;
      } else {
        triggerCounter = max(0, triggerCounter - 1);
      }

      bool systemAlert = isWarmedUp && (triggerCounter >= MIN_TRIGGER_SAMPLES);

      if (systemAlert) {
        digitalWrite(relayPin, HIGH);

        if (filteredSignal > activeAlertPeakPGA || !lastAlertState) {
          activeAlertPeakPGA = filteredSignal;
          currentIntensityStr = estimateMMI(activeAlertPeakPGA);
        }

        // Check if event just started or intensity escalated
        if (!lastAlertState) {
          lastAlertState = true;
          lastPrintedIntensity = currentIntensityStr;

          // MQTT Publish Initial Alert
          if (mqttClient.connected()) {
            char alertPayload[128];
            snprintf(alertPayload, sizeof(alertPayload),
            "{\"event\":\"ALERT_TRIGGERED\",\"pga\":%.4f,\"intensity\":\"%s\"}",
            activeAlertPeakPGA, currentIntensityStr);
            mqttClient.publish("seismic/alert", alertPayload);
          }

          // Serial Print Initial Trigger
          Serial.println("================================================");
          Serial.println("[EVENT TRIGGERED] Seismic Activity Detected!");
          Serial.print("Intensity: "); Serial.println(currentIntensityStr);
          Serial.println("================================================");

        } else if (strcmp(currentIntensityStr, lastPrintedIntensity) != 0) {
          lastPrintedIntensity = currentIntensityStr;

          // MQTT Publish Escalation
          if (mqttClient.connected()) {
            char alertPayload[128];
            snprintf(alertPayload, sizeof(alertPayload),
            "{\"event\":\"ALERT_TRIGGERED\",\"pga\":%.4f,\"intensity\":\"%s\"}",
            activeAlertPeakPGA, currentIntensityStr);
            mqttClient.publish("seismic/alert", alertPayload);
          }

          // Serial Print Instant Escalation
          Serial.print("[INTENSITY ESCALATION] Shaking changed to: ");
          Serial.println(currentIntensityStr);
        }

      } else {
        digitalWrite(relayPin, LOW);

        if (lastAlertState) {
          lastAlertState = false;
          activeAlertPeakPGA = 0.0;
          lastPrintedIntensity = "";

          if (mqttClient.connected()) {
            mqttClient.publish("seismic/alert", "{\"event\":\"STATUS_NORMAL\"}");
          }

          Serial.println("========================================");
          Serial.println("[STATUS NORMAL] Shaking has subsided.");
          Serial.println("========================================");
        }
      }

      // Dynamic PGA Window Tracking (1-Second Peak Tracking)
      if (filteredSignal > pgaWindowMax) {
        pgaWindowMax = filteredSignal;
      }
      pgaSampleCount++;

      if (pgaSampleCount >= PGA_WINDOW_SAMPLES) {
        const char* windowIntensityStr = estimateMMI(pgaWindowMax);

        if (mqttClient.connected()){
          char telemetryPayload[128];
          snprintf(telemetryPayload, sizeof(telemetryPayload),
            "{\"pga_g\":%.4f,\"ratio\":%.2f,\"intensity\":\"%s\"}", pgaWindowMax, ratio, windowIntensityStr);
          mqttClient.publish("seismic/telemetry", telemetryPayload);
        }

        pgaWindowMax = 0.0;
        pgaSampleCount = 0;
      }
    }
  }

  // Buzzer Control
  if (triggerCounter >= MIN_TRIGGER_SAMPLES) {
    unsigned long dynamicInterval = getBeepInterval(currentIntensityStr);

    if (currentTime - previousAlarmMillis >= dynamicInterval) {
      previousAlarmMillis = currentTime;
      alarmState = !alarmState;

      if (alarmState) {
        tone(buzzer, 2000);
      } else {
        noTone(buzzer);
      }
    }
  } else {
    noTone(buzzer);
    alarmState = LOW;
  }
}
