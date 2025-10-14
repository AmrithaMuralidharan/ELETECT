/**
  Eletect Link
  - Receives alerts from EleTect2.0, Echo and Ember over Meshtastic Serial Module (Serial2)
  - Displays alerts on SSD1306 OLED (I2C)
  - Activates red LED (MOSFET) + vibration motor on alert
  - SOS push-button sends {"sos":"<USERNAME>","loc":"<LOCATION>","t":<uptime_sec>} via Meshtastic
  - Non-blocking timers and debounced button
*/

/* === CONFIG - edit to suit your wiring & preferences === */

// Meshtastic UART (Serial2)
const int MESHTASTIC_RX_PIN = 16; // XIAO RX (connect to Wio TX)
const int MESHTASTIC_TX_PIN = 17; // XIAO TX (connect to Wio RX)
const unsigned long MESHTASTIC_BAUD = 38400UL;

// OLED (I2C) - use default SDA/SCL pins for XIAO
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Visual / Haptic outputs (MOSFET-driven)
const int SIGN_LED_PIN = 25;    // Red LED via MOSFET gate
const int VIBE_PIN = 26;        // Vibration motor driver (MOSFET)

// SOS push-button
const int SOS_BTN_PIN = 27;     // digital input with pullup
const unsigned long SOS_DEBOUNCE_MS = 50;
const unsigned long SOS_PRESS_COOLDOWN_MS = 5000; // don't resend SOS more than once per 5s

// Alert behaviour
const unsigned long ALERT_DURATION_MS = 15000UL;   // how long we vibrate+blink on each alert
const unsigned long LED_BLINK_ON_MS  = 300UL;
const unsigned long LED_BLINK_OFF_MS = 300UL;

// UI / history
const int ALERT_HISTORY_MAX = 6;

// User identity / location to send with SOS
const char *USER_NAME = "Ranger_Anil";
const char *LOCATION = "10.1234,76.1234";  // lat,lon — replace with your coordinates or GPS later

/* === Libraries === */
#include <ArduinoJson.h>

/* === State & structures === */

struct Alert {
  String atype;   // e.g., "elephant", "poaching", "tree_cutting", "vehicle", "wildfire", "unknown"
  String src;     // source: "audio","ember","vision", etc.
  String details; // message text or JSON excerpt
  unsigned long time_ms; // millis() when received
};

Alert alertHistory[ALERT_HISTORY_MAX];
int alertHistoryCount = 0;

// Active alert state (last incoming alert triggers this)
bool alertActive = false;
unsigned long alertActivatedAt = 0;

// LED blink state machine
bool ledStateOn = false;
unsigned long ledStateChangeAt = 0;

// Vibration state - we just turn on for ALERT_DURATION_MS but could blink
bool vibeOn = false;

// Meshtastic Serial buffer
String incomingBuf = "";

// SOS button state
unsigned long lastBtnChangeAt = 0;
bool lastBtnState = HIGH;
unsigned long lastSOSSentAt = 0;

// Meshtastic serial object uses Serial2 (hardware)
HardwareSerial &meshSerial = Serial2;

/* === Helper functions === */

void pushAlert(const Alert &a) {
  // keep history as FIFO
  if (alertHistoryCount < ALERT_HISTORY_MAX) {
    alertHistory[alertHistoryCount++] = a;
  } else {
    // shift left and append
    for (int i = 1; i < ALERT_HISTORY_MAX; ++i) alertHistory[i-1] = alertHistory[i];
    alertHistory[ALERT_HISTORY_MAX-1] = a;
  }
}

void triggerAlert(const Alert &a) {
  Serial.print("Trigger alert: "); Serial.println(a.atype);
  pushAlert(a);
  alertActive = true;
  alertActivatedAt = millis();
  // start LED/vibe
  ledStateOn = true;
  digitalWrite(SIGN_LED_PIN, HIGH);
  ledStateChangeAt = millis();
  digitalWrite(VIBE_PIN, HIGH);
  vibeOn = true;
  // update display immediately
  drawActiveAlert(a);
}

/* Draw functions */
void drawActiveAlert(const Alert &a) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println(a.atype); // big
  display.setTextSize(1);
  display.print("src: ");
  display.println(a.src);
  display.print("t: ");
  unsigned long sec = a.time_ms / 1000UL;
  display.println(sec);
  display.println();
  display.println(a.details);
  display.display();
}

void drawHistoryScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Eletect Link - Alerts");
  display.println("---------------------");
  int start = max(0, alertHistoryCount - 4);
  for (int i = start; i < alertHistoryCount; ++i) {
    unsigned long sec = alertHistory[i].time_ms / 1000UL;
    // show "[t] type src"
    String line = String("[") + String(sec) + String("s] ") + alertHistory[i].atype + String(" ") + alertHistory[i].src;
    if (line.length() > 27) line = line.substring(0, 27);
    display.println(line);
  }
  display.display();
}

/* Send SOS function */
void sendSOS() {
  unsigned long now = millis();
  if (now - lastSOSSentAt < SOS_PRESS_COOLDOWN_MS) {
    Serial.println("SOS suppressed due to cooldown");
    return;
  }
  lastSOSSentAt = now;

  // Create JSON: {"sos":"<user>","loc":"<lat,lon>","t":<uptime_s>}
  StaticJsonDocument<256> doc;
  doc["sos"] = USER_NAME;
  doc["loc"] = LOCATION;
  doc["t"] = now / 1000UL;
  char buf[256];
  size_t n = serializeJson(doc, buf);
  // send via Meshtastic Serial2 (TEXTMSG)
  meshSerial.println(buf);
  Serial.print("Sent SOS: "); Serial.println(buf);

  // immediate feedback
  // short vibration + LED blink
  digitalWrite(VIBE_PIN, HIGH);
  digitalWrite(SIGN_LED_PIN, HIGH);
  delay(300);
  digitalWrite(VIBE_PIN, LOW);
  digitalWrite(SIGN_LED_PIN, LOW);
}

/* Try to parse incoming line: prefer JSON; fallback to keyword matching */
void handleIncomingLine(const String &line) {
  String l = line;
  l.trim();
  if (l.length() == 0) return;

  Serial.print("RX: "); Serial.println(l);

  // First try to parse as JSON
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, l);
  Alert a;
  a.time_ms = millis();
  a.src = "unknown";
  a.atype = "unknown";
  a.details = l;

  if (!err) {
    // JSON parsed - extract likely fields
    if (doc.containsKey("src")) a.src = String((const char*)doc["src"]);
    if (doc.containsKey("evt")) a.atype = String((const char*)doc["evt"]);
    else if (doc.containsKey("event")) a.atype = String((const char*)doc["event"]);
    else if (doc.containsKey("type")) a.atype = String((const char*)doc["type"]);
    // details: include msg or entire json string truncated
    if (doc.containsKey("msg")) a.details = String((const char*)doc["msg"]);
    else {
      // keep a short serialized form
      char tmp[200];
      size_t n = serializeJson(doc, tmp, sizeof(tmp));
      a.details = String(tmp).substring(0, 120);
    }
    // normalize some known cases
    String lowerType = a.atype;
    lowerType.toLowerCase();
    if (lowerType.indexOf("gun") >= 0 || lowerType.indexOf("poach") >= 0) a.atype = "poaching";
    if (lowerType.indexOf("saw") >= 0 || lowerType.indexOf("chainsaw") >= 0) a.atype = "tree_cutting";
    if (lowerType.indexOf("eleph") >= 0) a.atype = "elephant";
    if (lowerType.indexOf("wild") >= 0 || lowerType.indexOf("fire") >= 0 || lowerType.indexOf("alert") >= 0) a.atype = "wildfire";
    // Good parse -> trigger
    triggerAlert(a);
    return;
  }

  // Not JSON — fallback to substring keyword checks
  String lower = l;
  lower.toLowerCase();
  if (lower.indexOf("elephant") >= 0) {
    a.atype = "elephant"; a.src = "vision"; a.details = l; triggerAlert(a); return;
  }
  if (lower.indexOf("gun") >= 0 || lower.indexOf("shot") >= 0 || lower.indexOf("poach") >= 0) {
    a.atype = "poaching"; a.src = "audio"; a.details = l; triggerAlert(a); return;
  }
  if (lower.indexOf("saw") >= 0 || lower.indexOf("chainsaw") >= 0) {
    a.atype = "tree_cutting"; a.src = "audio"; a.details = l; triggerAlert(a); return;
  }
  if (lower.indexOf("vehicle") >= 0 || lower.indexOf("car") >= 0 || lower.indexOf("truck") >= 0) {
    a.atype = "vehicle"; a.src = "vision"; a.details = l; triggerAlert(a); return;
  }
  if (lower.indexOf("wildfire") >= 0 || lower.indexOf("alert") >= 0 || lower.indexOf("fire") >= 0) {
    a.atype = "wildfire"; a.src = "ember"; a.details = l; triggerAlert(a); return;
  }

  // otherwise, keep on history but don't trigger
  a.atype = "misc";
  pushAlert(a);
  // update small summary display
  drawHistoryScreen();
}

/* Read Serial2 (Meshtastic) lines */
void pollMeshtastic() {
  while (meshSerial.available() > 0) {
    char c = (char)meshSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = incomingBuf;
      incomingBuf = "";
      handleIncomingLine(line);
    } else {
      incomingBuf += c;
      if (incomingBuf.length() > 1024) incomingBuf = incomingBuf.substring(incomingBuf.length() - 512);
    }
  }
}

/* Manage blinking & vibration state (non-blocking) */
void updateAlertState() {
  unsigned long now = millis();
  if (!alertActive) {
    // ensure outputs off
    if (ledStateOn) { digitalWrite(SIGN_LED_PIN, LOW); ledStateOn = false; }
    if (vibeOn) { digitalWrite(VIBE_PIN, LOW); vibeOn = false; }
    return;
  }

  // If alert duration over, stop
  if (now - alertActivatedAt >= ALERT_DURATION_MS) {
    alertActive = false;
    digitalWrite(SIGN_LED_PIN, LOW);
    digitalWrite(VIBE_PIN, LOW);
    ledStateOn = false;
    vibeOn = false;
    // show history when no active alert
    drawHistoryScreen();
    return;
  }

  // Blink LED
  if (ledStateOn && now - ledStateChangeAt >= LED_BLINK_ON_MS) {
    ledStateOn = false; ledStateChangeAt = now; digitalWrite(SIGN_LED_PIN, LOW);
  } else if (!ledStateOn && now - ledStateChangeAt >= LED_BLINK_OFF_MS) {
    ledStateOn = true; ledStateChangeAt = now; digitalWrite(SIGN_LED_PIN, HIGH);
  }

  // Keep vibration on for the duration (or you could pulse it)
  if (!vibeOn) { digitalWrite(VIBE_PIN, HIGH); vibeOn = true; }
}

/* SOS button polling + debouncing */
void pollSOSButton() {
  bool raw = digitalRead(SOS_BTN_PIN);
  unsigned long now = millis();
  if (raw != lastBtnState) {
    lastBtnChangeAt = now;
    lastBtnState = raw;
  } else {
    if ((now - lastBtnChangeAt) > SOS_DEBOUNCE_MS) {
      // stable
      if (raw == LOW) { // active low pushbutton
        // button pressed
        // ensure we only send once per physical press by checking lastSOSSentAt
        if (now - lastSOSSentAt >= SOS_PRESS_COOLDOWN_MS) {
          sendSOS(); // sends via meshSerial and gives feedback
          lastSOSSentAt = now;
        }
        // wait until released (simple blocking until release with small delay)
        // but to remain responsive, we just debounce and let user release
      }
    }
  }
}

/* Build and send SOS JSON */
void sendSOS() {
  unsigned long now = millis();
  StaticJsonDocument<256> doc;
  doc["sos"] = USER_NAME;
  doc["loc"] = LOCATION;
  doc["t"] = now / 1000UL;
  char buf[256];
  size_t len = serializeJson(doc, buf, sizeof(buf));
  meshSerial.println(buf);
  Serial.print("SOS sent: "); Serial.println(buf);
  // immediate local feedback: quick vibration + LED blink
  digitalWrite(VIBE_PIN, HIGH);
  digitalWrite(SIGN_LED_PIN, HIGH);
  delay(200);
  digitalWrite(VIBE_PIN, LOW);
  digitalWrite(SIGN_LED_PIN, LOW);
  // show confirmation on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("SOS sent!");
  display.print("User: "); display.println(USER_NAME);
  display.print("Loc: "); display.println(LOCATION);
  display.display();
}

/* === Setup === */
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Eletect Link starting...");

  // pins
  pinMode(SIGN_LED_PIN, OUTPUT);
  digitalWrite(SIGN_LED_PIN, LOW);
  pinMode(VIBE_PIN, OUTPUT);
  digitalWrite(VIBE_PIN, LOW);
  pinMode(SOS_BTN_PIN, INPUT_PULLUP);

  // init OLED
  Wire.begin(); // default I2C pins
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common
    Serial.println("SSD1306 allocation failed");
    // continue but OLED won't work
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Eletect Link");
    display.println("Listening for alerts...");
    display.display();
  }

  // init Serial2 (meshtastic)
  meshSerial.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_RX_PIN, MESHTASTIC_TX_PIN);
  delay(50);
  Serial.println("Meshtastic Serial2 ready");

  // initial UI
  drawHistoryScreen();
}

/* === Main loop === */
void loop() {
  pollMeshtastic();
  pollSOSButton();
  updateAlertState();
  // short delay to avoid busy-looping
  delay(10);
}