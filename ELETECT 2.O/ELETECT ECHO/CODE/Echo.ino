/*
  Eletect Echo - XIAO ESP32S3 (Audio-only) with Meshtastic + Ember
   - Audio inference: gunshot, saw, vehicle -> map to poaching/tree_cutting/vehicle
  - Probe EEPROM 0x50 to detect Ember addon; forward Ember alert lines if present
  - Send JSON TEXTMSG lines over Meshtastic 
  - Log all forwarded events to /events.csv with timestamp,event,src,conf
*/

#define MODEL_NAME "EleTect_Echo"

#include <Arduino.h>
#include <Wire.h>

#define EIDSP_QUANTIZE_FILTERBANK 0
#include <EleTect_Echo_inferencing.h> // Edge Impulse glue (your model)
#include <I2S.h>

#include <SD.h>
#include <RTClib.h>

// ----------------- Hardware / Pins -----------------
// Meshtastic serial (Serial2)
const int MESHTASTIC_RX_PIN = 16; // XIAO RX (connect to Wio TX)
const int MESHTASTIC_TX_PIN = 17; // XIAO TX (connect to Wio RX)
const unsigned long MESHTASTIC_BAUD = 38400UL;

// Ember serial (Serial1) - if Ember is attached we listen here
const int EMBER_RX_PIN = 4;  // XIAO RX (connect to Ember TX)
const int EMBER_TX_PIN = 5;  // XIAO TX (connect to Ember RX)
const unsigned long EMBER_BAUD = 115200UL;

// I2S pins for the XIAO microphone
#define I2S_BCK_PIN  -1
#define I2S_WS_PIN   42
#define I2S_DIN_PIN  41
#define I2S_DOUT_PIN -1

// I2C EEPROM address to probe for Ember presence
#define EEPROM_I2C_ADDR 0x50

// SD card & RTC
const int SD_CS_PIN = 5;            // change if your SD CS is on another pin
RTC_DS3231 rtc;
bool rtc_ok = false;
bool sd_ok = false;
const char *EVENTS_CSV = "/events.csv";

// ----------------- Detection settings -----------------
const float CONFIDENCE_THRESHOLD = 0.60f; // audio model threshold (tune)
const unsigned long PER_EVENT_COOLDOWN_MS = 10UL * 1000UL; // 10 s per-type cooldown
const unsigned long EMBER_MESSAGE_COOLDOWN_MS = 5UL * 1000UL; // 5 s min between forwarded ember messages

// ----------------- Globals -----------------
bool emberPresent = false;
unsigned long lastEEPROMCheckMs = 0;
const unsigned long EEPROM_CHECK_INTERVAL_MS = 5000UL; // re-check every 5s

// Meshtastic Serial2 buffer
String meshtasticBuf = "";
// Ember Serial1 buffer
String emberBuf = "";

// last send times
unsigned long lastSentPoaching = 0;
unsigned long lastSentTreeCutting = 0;
unsigned long lastSentVehicle = 0;
unsigned long lastSentEmber = 0;

// ---------- Forward declarations ----------
bool probeEEPROMonce();
void sendMeshtasticJson(const String &json); // send JSON over Serial2
String classifyAudioToEvent(const String &label); // maps model label to event name
void processAudioInference();
void processEmberSerial();
void processMeshtasticSerial(); // optional
String timestampNow(); // uses RTC if available; otherwise uptime
void appendEventLog(const String &timestamp, const String &src, const String &event, const String &details);

// ------------- Audio inference structures (Edge Impulse style) -------------
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;

// ----------------- Audio capture callbacks -----------------
static void audio_inference_callback(uint32_t n_bytes)
{
    for(int i = 0; i < (n_bytes >> 1); i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];
        if(inference.buf_count >= inference.n_samples) {
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {
    esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
    if (bytes_read <= 0) {
      ei_printf("Error in I2S read : %d", (int)bytes_read);
    } else {
        for (int x = 0; x < i2s_bytes_to_read/2; x++) {
            sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        }
        if (record_status) {
            audio_inference_callback(i2s_bytes_to_read);
        } else {
            break;
        }
    }
  }
  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    ei_sleep(100);
    record_status = true;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

static bool microphone_inference_record(void)
{
    while (inference.buf_ready == 0) {
        delay(10);
    }
    inference.buf_ready = 0;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void)
{
    if (inference.buffer) {
        ei_free(inference.buffer);
        inference.buffer = NULL;
    }
}

// ----------------- Implementation -----------------

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println();
  Serial.println("Eletect Echo (audio-only) starting...");
  Serial.printf("Model label: %s\n", MODEL_NAME);

  // I2C for EEPROM probe
  Wire.begin(); // default SDA/SCL

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("RTC DS3231 not found - timestamps will use uptime.");
    rtc_ok = false;
  } else {
    rtc_ok = true;
    if (rtc.lostPower()) {
      Serial.println("RTC lost power - setting to compile time.");
      rtc.adjust(DateTime(F(_DATE), F(TIME_)));
    }
    Serial.println("RTC initialized.");
  }

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    sd_ok = false;
    Serial.println("SD init failed; continuing without SD logging.");
  } else {
    sd_ok = true;
    Serial.println("SD initialized.");
    // Create events file with header if missing
    if (!SD.exists(EVENTS_CSV)) {
      File f = SD.open(EVENTS_CSV, FILE_WRITE);
      if (f) {
        f.println("timestamp,src,event,details");
        f.close();
        Serial.println("Created events.csv");
      } else {
        Serial.println("Failed to create events.csv");
      }
    }
  }

  // Start Meshtastic UART on Serial2
  Serial2.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_RX_PIN, MESHTASTIC_TX_PIN);
  delay(50);
  Serial.println("Meshtastic Serial2 started.");

  // Start Ember UART (Serial1) - only used if Ember is attached, but starting is harmless
  Serial1.begin(EMBER_BAUD, SERIAL_8N1, EMBER_RX_PIN, EMBER_TX_PIN);
  delay(20);
  Serial.println("Ember Serial1 started (listening).");

  // Start I2S for microphone
  I2S.setAllPins(I2S_BCK_PIN, I2S_WS_PIN, I2S_DIN_PIN, I2S_DOUT_PIN, -1);
  if (!I2S.begin(PDM_MONO_MODE, 16000U, 16)) {
    Serial.println("Failed to init I2S for microphone!");
  } else {
    Serial.println("I2S microphone started.");
    if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
      Serial.println("ERR: audio buffer allocation failed");
    } else {
      Serial.println("Audio inference ready.");
    }
  }

  // initial EEPROM probe
  emberPresent = probeEEPROMonce();
  Serial.printf("Ember present at 0x%02X: %s\n", EEPROM_I2C_ADDR, emberPresent ? "YES" : "NO");
  lastEEPROMCheckMs = millis();
}

// Main loop
void loop() {
  // Periodically probe EEPROM (so ember can be plugged/unplugged at runtime)
  if (millis() - lastEEPROMCheckMs >= EEPROM_CHECK_INTERVAL_MS) {
    bool nowPresent = probeEEPROMonce();
    if (nowPresent != emberPresent) {
      emberPresent = nowPresent;
      Serial.printf("Ember presence changed: %s\n", emberPresent ? "ATTACHED" : "DETACHED");
    }
    lastEEPROMCheckMs = millis();
  }

  // Process Ember serial (if any)
  if (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\r') { /* ignore */ }
    else if (c == '\n') {
      String line = emberBuf;
      emberBuf = "";
      line.trim();
      if (line.length() > 0) {
        String lower = line;
        lower.toLowerCase();
        if (lower.indexOf("alert") >= 0 || lower.indexOf("wildfire") >= 0 || lower.indexOf("fire") >= 0) {
          unsigned long now = millis();
          if (now - lastSentEmber >= EMBER_MESSAGE_COOLDOWN_MS) {
            lastSentEmber = now;
            String payload = String("{\"src\":\"ember\",\"type\":\"alert\",\"msg\":\"") + line + String("\"}");
            sendMeshtasticJson(payload);
            Serial.print("Forwarded Ember alert: ");
            Serial.println(line);
            // log to SD
            appendEventLog(timestampNow(), "ember", "alert", line);
          } else {
            Serial.println("Ember alert suppressed due to cooldown.");
          }
        } else {
          Serial.print("Ember line ignored: ");
          Serial.println(line);
        }
      }
    } else {
      emberBuf += c;
      if (emberBuf.length() > 1024) emberBuf = emberBuf.substring(emberBuf.length() - 512);
    }
  }

  // Process audio inference blocking-per-frame
  processAudioInference();

  // Optionally echo or handle Serial2 incoming lines (not required)
  processMeshtasticSerial();

  delay(10);
}

/* ---------- helpers ---------- */

// Probe EEPROM once for presence at 0x50
bool probeEEPROMonce() {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  byte err = Wire.endTransmission();
  return (err == 0);
}

// Send a line of JSON to meshtastic via Serial2
void sendMeshtasticJson(const String &json) {
  Serial.print("-> Meshtastic: ");
  Serial.println(json);
  Serial2.println(json);
}

// Process one audio inference cycle and send/log messages if detected
void processAudioInference() {
  // record buffer ready (blocks until a frame is recorded)
  if (!microphone_inference_record()) {
    // recording failed
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  // find best prediction
  int pred_index = -1;
  float pred_value = 0.0f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > pred_value) {
      pred_index = ix;
      pred_value = result.classification[ix].value;
    }
  }

  if (pred_index < 0 || pred_value < CONFIDENCE_THRESHOLD) {
    // no confident audio detection
    return;
  }

  const char *label = result.classification[pred_index].label;
  String sLabel = String(label);
  sLabel.toLowerCase();

  String eventName = classifyAudioToEvent(sLabel);
  if (eventName == "none") {
    return; // not relevant
  }

  unsigned long now = millis();
  bool allowedToSend = false;
  if (eventName == "poaching" && now - lastSentPoaching >= PER_EVENT_COOLDOWN_MS) {
    allowedToSend = true; lastSentPoaching = now;
  } else if (eventName == "tree_cutting" && now - lastSentTreeCutting >= PER_EVENT_COOLDOWN_MS) {
    allowedToSend = true; lastSentTreeCutting = now;
  } else if (eventName == "vehicle" && now - lastSentVehicle >= PER_EVENT_COOLDOWN_MS) {
    allowedToSend = true; lastSentVehicle = now;
  }

  if (!allowedToSend) {
    Serial.printf("Detected %s but suppressed by cooldown.\n", eventName.c_str());
    return;
  }

  // build JSON payload: include confidence
  float conf = pred_value; // 0..1
  String payload = String("{\"src\":\"audio\",\"evt\":\"") + eventName + String("\",\"conf\":") + String(conf, 3) + String(",\"t\":") + String(now) + String("}");
  sendMeshtasticJson(payload);

  Serial.printf("Audio detected: label=%s event=%s conf=%.3f -> sent\n", label, eventName.c_str(), conf);

  // Log to SD if available: timestamp,src,event,details(confidence)
  appendEventLog(timestampNow(), "audio", eventName, String(conf, 3));
}

// map audio label strings to high-level events
String classifyAudioToEvent(const String &label) {
  if (label.indexOf("gun") >= 0 || label.indexOf("shot") >= 0) {
    return "poaching";
  }
  if (label.indexOf("saw") >= 0 || label.indexOf("chainsaw") >= 0) {
    return "tree_cutting";
  }
  if (label.indexOf("vehicle") >= 0 || label.indexOf("car") >= 0 || label.indexOf("truck") >= 0) {
    return "vehicle";
  }
  return "none";
}

// Optional: read/echo any messages from Serial2 (Meshtastic replies)
void processMeshtasticSerial() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    Serial.write(c);
  }
}

// Return timestamp string: prefer RTC; otherwise uptime
String timestampNow() {
  if (rtc_ok) {
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    return String(buf);
  } else {
    unsigned long s = millis() / 1000;
    char buf[32];
    snprintf(buf, sizeof(buf), "uptime_%lus", s);
    return String(buf);
  }
}

// Append CSV row: timestamp,src,event,details
void appendEventLog(const String &timestamp, const String &src, const String &event, const String &details) {
  if (!sd_ok) return;
  File f = SD.open(EVENTS_CSV, FILE_WRITE); // FILE_WRITE appends
  if (!f) {
    Serial.println("Failed to open events.csv for append");
    return;
  }
  String safeDetails = details;
  safeDetails.replace("\"", "'"); // avoid breaking CSV if double quotes present
  safeDetails.replace(",", " ");  // make sure comma doesn't shift columns
  f.print("\""); f.print(timestamp); f.print("\",");
  f.print("\""); f.print(src); f.print("\",");
  f.print("\""); f.print(event); f.print("\",");
  f.print("\""); f.print(safeDetails); f.println("\"");
  f.close();
  Serial.printf("Logged event: %s,%s,%s,%s\n", timestamp.c_str(), src.c_str(), event.c_str(), safeDetails.c_str());
}