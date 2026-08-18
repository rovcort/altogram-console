#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <PubSubClient.h>
#include <FlashStorage.h>
#include <Adafruit_SleepyDog.h>
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
//
// Also install "Adafruit SleepyDog Library" via Library Manager - it wraps
// the SAMD21's built-in hardware watchdog. If setup()/loop() ever hangs
// (a blocked TLS handshake, a stuck library call, etc.), the watchdog
// resets the MCU instead of the whole detector silently going dark.
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

// Built once in setup() from the board's WiFi MAC address so two nodes on
// the same broker never collide on "SeismicSensorClient" and fight for
// the connection (each new connect would kick the other off).
char mqttClientID[24] = "SeismicSensorClient";
bool mqttClientIdSet = false;

bool lastAlertState = false;
bool imuFault = false; // set in setup() if the IMU never responds; loop() reads this to skip sensing safely

const unsigned long SAMPLE_INTERVAL_MS = 10; // 100 Hz
const float THRESHOLD_RATIO = 2.5;

// STA / LTA Coefficients
const float ALPHA_STA = 0.20;
const float ALPHA_LTA = 0.005;

// MIN_TRIGGER_SAMPLES was 10 (100ms sustained above threshold) - short
// enough that a single bump or knock could trip a relay wired to a gas or
// electrical shutoff. 100 samples = 1 second sustained, which is long
// enough to screen out short mechanical transients while still being fast
// relative to real P-wave onset durations.
const int MIN_TRIGGER_SAMPLES = 100;
int triggerCounter = 0;

// Once tripped, the relay LATCHES (stays engaged) regardless of triggerCounter
// dropping back down - this is deliberate: an earthquake's shaking oscillates,
// so if release were tied directly to triggerCounter the relay could chatter
// on/off rapidly mid-event. For a gas or electrical shutoff that's the
// failure mode to avoid most - re-energizing a gas line mid-quake before
// anyone has checked for a leak is exactly what a real seismic shutoff valve
// is designed never to do. Release only happens via an explicit reset
// command (see mqttCallback / seismic/command below), matching how real
// mechanical seismic gas shutoff valves require manual reset.
bool relayLatched = false;
const char* COMMAND_TOPIC = "seismic/command";

// Buzzer Alarm
const int buzzer = 4;
unsigned long previousAlarmMillis = 0;
bool alarmState = LOW;

// Buzzer auto-mutes after BUZZER_DURATION_MS - deliberately independent of
// relayLatched. The relay stays tripped (and requires manual reset) exactly
// as before; only the audible alarm times out, so a real event this long
// doesn't produce an indefinite, escalating siren someone has to physically
// reach the device/dashboard to silence.
unsigned long alertStartMillis = 0;
const unsigned long BUZZER_DURATION_MS = 30000;

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
float pgaWindowPeakRatio = 0.0; // ratio captured at the same instant as pgaWindowMax
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
// Clears a tripped relay latch. Only called from mqttCallback() below in
// response to an explicit {"cmd":"reset"} on seismic/command - never
// automatically, by design (see relayLatched comment above).
void resetRelayLatch() {
  relayLatched = false;
  lastAlertState = false;
  triggerCounter = 0;
  activeAlertPeakPGA = 0.0;
  lastPrintedIntensity = "";
  digitalWrite(relayPin, LOW);

  if (mqttClient.connected()) {
    mqttClient.publish("seismic/alert", "{\"event\":\"STATUS_NORMAL\"}", true);
  }

  Serial.println("========================================");
  Serial.println("[RELAY RESET] Latch cleared via manual command.");
  Serial.println("========================================");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, COMMAND_TOPIC) != 0) return;

  char buf[64];
  unsigned int copyLen = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
  memcpy(buf, payload, copyLen);
  buf[copyLen] = '\0';

  // Deliberately simple string match rather than a JSON parser - this is a
  // single-field control message, and avoiding a JSON dependency here keeps
  // the safety-critical reset path minimal.
  if (strstr(buf, "\"reset\"") != nullptr) {
    resetRelayLatch();
  }
}

void reconnectMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(creds.mqttServer);
    Serial.print("...");

    const char* clientID = mqttClientID;
    const char* willTopic = "seismic/system";
    int willQoS = 1;
    bool willRetain = true;
    const char* willMessage = "{\"status\":\"disconnected\"}";

    if (mqttClient.connect(clientID, creds.mqttUser, creds.mqttPass, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("CONNECTED");
      char statusPayload[128];
      const char* status = "connected";
      snprintf(statusPayload, sizeof(statusPayload),
               "{\"status\":\"%s\",\"imu_fault\":%s}",
               status, imuFault ? "true" : "false");
      mqttClient.publish("seismic/system", statusPayload, true);
      mqttClient.subscribe(COMMAND_TOPIC);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Will retry next loop execution");
    }
  }
}

void assignMqttClientIdFromMac() {
  if (mqttClientIdSet) return;
  byte mac[6];
  WiFi.macAddress(mac);
  snprintf(mqttClientID, sizeof(mqttClientID), "SeismicSensor-%02X%02X%02X",
           mac[2], mac[1], mac[0]);
  mqttClientIdSet = true;
  Serial.print("MQTT Client ID: "); Serial.println(mqttClientID);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  loadOrConfigureCredentials();

  // Enabled here (not earlier) because runConfigWizard() above blocks
  // waiting on user Serial input with no timeout - enabling the watchdog
  // before that would reset the board mid-configuration.
  int wdtMs = Watchdog.enable(16000);
  Serial.print("Watchdog enabled, timeout ~"); Serial.print(wdtMs); Serial.println("ms");

  // Previously: if (!IMU.begin()) while (1); - a wiring/hardware fault here
  // hung the MCU forever with zero indication of why over Serial or MQTT.
  // Now: retry a few times, and if it still fails, keep booting in a
  // degraded mode so the fault can be reported once connectivity is up,
  // rather than looking indistinguishable from a dead/unplugged device.
  const int IMU_INIT_RETRIES = 5;
  bool imuOk = false;
  for (int attempt = 1; attempt <= IMU_INIT_RETRIES; attempt++) {
    if (IMU.begin()) { imuOk = true; break; }
    Serial.print("IMU init failed (attempt ");
    Serial.print(attempt);
    Serial.println(") - retrying...");
    delay(500);
  }
  imuFault = !imuOk;
  if (imuFault) {
    Serial.println("IMU FAULT: sensor not responding after retries. Booting in degraded mode.");
  }

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
    assignMqttClientIdFromMac();
  }

  mqttClient.setServer(creds.mqttServer, creds.mqttPort);
  mqttClient.setCallback(mqttCallback);

  Serial.println("System Initialized. Monitoring for seismic activity...");

  pinMode(buzzer, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
}

void loop() {
  Watchdog.reset();

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
    assignMqttClientIdFromMac();
    if(!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
  }

  if (!imuFault && currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
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

      bool crossedTriggerThreshold = isWarmedUp && (triggerCounter >= MIN_TRIGGER_SAMPLES);

      // Trip the latch once; triggerCounter dropping back down afterward
      // (shaking is oscillatory, not a steady signal) no longer releases it.
      if (crossedTriggerThreshold && !relayLatched) {
        relayLatched = true;
      }

      if (relayLatched) {
        digitalWrite(relayPin, HIGH);

        // Peak/intensity tracking continues for as long as the relay stays
        // latched, independent of triggerCounter, so escalation is still
        // reported correctly even between oscillation troughs.
        if (filteredSignal > activeAlertPeakPGA || !lastAlertState) {
          activeAlertPeakPGA = filteredSignal;
          currentIntensityStr = estimateMMI(activeAlertPeakPGA);
        }

        // Check if event just started or intensity escalated
        if (!lastAlertState) {
          lastAlertState = true;
          lastPrintedIntensity = currentIntensityStr;
          alertStartMillis = currentTime; // buzzer countdown starts here, not on later escalations

          // MQTT Publish Initial Alert
          // retain=true so a dashboard that connects (or reconnects) mid-event
          // immediately sees the current alert state instead of nothing until
          // the next escalation or all-clear message happens to fire.
          if (mqttClient.connected()) {
            char alertPayload[160];
            snprintf(alertPayload, sizeof(alertPayload),
            "{\"event\":\"ALERT_TRIGGERED\",\"pga\":%.4f,\"intensity\":\"%s\",\"latched\":true}",
            activeAlertPeakPGA, currentIntensityStr);
            mqttClient.publish("seismic/alert", alertPayload, true);
          }

          // Serial Print Initial Trigger
          Serial.println("================================================");
          Serial.println("[EVENT TRIGGERED] Seismic Activity Detected! Relay LATCHED - requires manual reset.");
          Serial.print("Intensity: "); Serial.println(currentIntensityStr);
          Serial.println("================================================");

        } else if (strcmp(currentIntensityStr, lastPrintedIntensity) != 0) {
          lastPrintedIntensity = currentIntensityStr;

          // MQTT Publish Escalation
          if (mqttClient.connected()) {
            char alertPayload[160];
            snprintf(alertPayload, sizeof(alertPayload),
            "{\"event\":\"ALERT_TRIGGERED\",\"pga\":%.4f,\"intensity\":\"%s\",\"latched\":true}",
            activeAlertPeakPGA, currentIntensityStr);
            mqttClient.publish("seismic/alert", alertPayload, true);
          }

          // Serial Print Instant Escalation
          Serial.print("[INTENSITY ESCALATION] Shaking changed to: ");
          Serial.println(currentIntensityStr);
        }

      } else {
        digitalWrite(relayPin, LOW);
      }

      // Dynamic PGA Window Tracking (1-Second Peak Tracking)
      // ratio is captured at the same instant as the new peak so the two
      // published numbers describe the same moment - previously "ratio"
      // was just whatever the STA/LTA ratio happened to be when the
      // 1-second window closed, which could be a different instant than
      // when pgaWindowMax actually occurred.
      if (filteredSignal > pgaWindowMax) {
        pgaWindowMax = filteredSignal;
        pgaWindowPeakRatio = ratio;
      }
      pgaSampleCount++;

      if (pgaSampleCount >= PGA_WINDOW_SAMPLES) {
        const char* windowIntensityStr = estimateMMI(pgaWindowMax);

        // Device-side NTP timestamp (epoch seconds), when available, so
        // telemetry can be correlated across multiple sensor nodes without
        // relying solely on each dashboard browser's local clock. Returns
        // 0 if the NINA module hasn't completed an NTP sync yet.
        unsigned long epochTime = WiFi.getTime();

        // Progress toward tripping, as a percentage of the sustained window
        // required (MIN_TRIGGER_SAMPLES). triggerCounter isn't capped at
        // MIN_TRIGGER_SAMPLES (it keeps counting up while shaking continues
        // even after the latch has already engaged), so clamp for display.
        int cappedCounter = min(triggerCounter, MIN_TRIGGER_SAMPLES);
        float triggerPct = (cappedCounter * 100.0) / MIN_TRIGGER_SAMPLES;

        if (mqttClient.connected()){
          char telemetryPayload[192];
          snprintf(telemetryPayload, sizeof(telemetryPayload),
            "{\"pga_g\":%.4f,\"ratio\":%.2f,\"intensity\":\"%s\",\"ts\":%lu,\"trigger_pct\":%.1f}",
            pgaWindowMax, pgaWindowPeakRatio, windowIntensityStr, epochTime, triggerPct);
          mqttClient.publish("seismic/telemetry", telemetryPayload);
        }

        pgaWindowMax = 0.0;
        pgaWindowPeakRatio = 0.0;
        pgaSampleCount = 0;
      }
    }
  }

  // Buzzer Control
  if (imuFault) {
    // Slow, distinct pattern (different pitch/cadence from the quake alarm)
    // so a sensor fault is observable locally even with no MQTT connection.
    const unsigned long FAULT_BEEP_INTERVAL_MS = 2000;
    if (currentTime - previousAlarmMillis >= FAULT_BEEP_INTERVAL_MS) {
      previousAlarmMillis = currentTime;
      alarmState = !alarmState;
      if (alarmState) tone(buzzer, 600); else noTone(buzzer);
    }
  } else if (relayLatched && (currentTime - alertStartMillis < BUZZER_DURATION_MS)) {
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
