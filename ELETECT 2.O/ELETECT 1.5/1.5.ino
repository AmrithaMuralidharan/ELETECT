/*
  Signage node for XIAO ESP32S3
  - Listens for "elephant" messages from EleTect node over Meshtastic
  - When elephant reported: blink a red signage LED (MOSFET-driven) for 40s after last report
  - Uses Grove Vision AI V2 to detect "vehicle" on road and reports vehicle events to EleTect node
    via Meshtastic  ]
  Pin & behavior configuration near top.
*/

#include <Arduino.h>
#include <Seeed_Arduino_SSCMA.h> // Grove Vision AI V2

// ----------------- CONFIGURATION -----------------
// Meshtastic serial (to Wio SX1262 Serial Module)
const int MESHTASTIC_RX_PIN = 16; // XIAO RX (connect to Wio TX)
const int MESHTASTIC_TX_PIN = 17; // XIAO TX (connect to Wio RX)
const unsigned long MESHTASTIC_BAUD = 38400UL; // must match Serial Module baud

// Grove Vision AI V2 UART - use a second serial port
// On XIAO ESP32S3 we will use atSerial (HardwareSerial(0)) for Grove Vision if desired.
// Seeed library uses a HardwareSerial pointer when calling begin(). Keep as is.
#ifdef ESP32
  #include <HardwareSerial.h>
  HardwareSerial atSerial(0); // Grove Vision AI UART (example)
#else
  #define atSerial Serial1
#endif

Seeed_Arduino_SSCMA AI; // Grove Vision AI object

// LED signage output (MOSFET gate)
const int SIGN_LED_PIN = 25; // change to the MOSFET gate pin you connected

// Blink pattern configuration
const unsigned long SIGNAGE_TIMEOUT_MS = 40UL * 1000UL; // 40 seconds after last elephant message
const unsigned long BLINK_ON_MS = 300;   // LED on duration in blink cycle
const unsigned long BLINK_OFF_MS = 300;  // LED off duration in blink cycle

// Vision detection threshold and interval
const int VISION_SCORE_THRESHOLD = 80; // Grove Vision AI score threshold (0..100)
const unsigned long VISION_POLL_MS = 800; // how often to invoke vision model

// Vehicle reporting cooldown so we don't spam the network
const unsigned long VEHICLE_SEND_COOLDOWN_MS = 10000UL; // 10s between vehicle reports

// -------------------------------------------------

// State variables for signage
volatile unsigned long lastElephantMs = 0; // last time an elephant message was received
bool signageActive = false;

// Blink state machine
bool blinkStateOn = false;
unsigned long blinkStateChangeAt = 0;

// Vision timing
unsigned long lastVisionPollAt = 0;
unsigned long lastVehicleSentAt = 0;

// Serial buffer for incoming meshtastic lines
String incomingLine = "";

// Helper: send JSON line to meshtastic serial (TEXTMSG mode)
void sendMeshtastic(const String &payload) {
  // send as a simple line; Meshtastic Serial Module will handle sending as TEXTMSG
  Serial.print("-> meshtastic: ");
  Serial.println(payload);
  Serial2.println(payload);
}

// Parse incoming serial line(s) from Serial2 for elephant messages
void processIncomingLine(const String &line) {
  String lower = line;
  lower.toLowerCase();
  if (lower.indexOf("elephant") >= 0) {
    // Elephant spotted by remote node
    lastElephantMs = millis();
    signageActive = true;
    // Reset blink state so LED turns on immediately
    blinkStateOn = true;
    blinkStateChangeAt = millis();
    digitalWrite(SIGN_LED_PIN, HIGH);
    Serial.println("Elephant message received -> signage activated (40s timeout).");
  } else {
    // Not elephant; optionally inspect other incoming messages if needed
    Serial.print("Incoming (ignored): ");
    Serial.println(line);
  }
}

// Read any available bytes from Serial2 and accumulate lines
void pollMeshtasticSerial() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = incomingLine;
      line.trim();
      incomingLine = "";
      if (line.length() > 0) {
        processIncomingLine(line);
      }
    } else {
      incomingLine += c;
      // Keep buffer bounded
      if (incomingLine.length() > 1024) {
        incomingLine = incomingLine.substring(incomingLine.length() - 512);
      }
    }
  }
}

// Non-blocking blink update; keep LED toggling while signageActive
void updateBlinkState() {
  if (!signageActive) {
    // ensure LED off
    if (blinkStateOn) {
      blinkStateOn = false;
      digitalWrite(SIGN_LED_PIN, LOW);
    }
    return;
  }

  unsigned long now = millis();
  if (blinkStateOn) {
    if (now - blinkStateChangeAt >= BLINK_ON_MS) {
      blinkStateOn = false;
      blinkStateChangeAt = now;
      digitalWrite(SIGN_LED_PIN, LOW);
    }
  } else {
    if (now - blinkStateChangeAt >= BLINK_OFF_MS) {
      blinkStateOn = true;
      blinkStateChangeAt = now;
      digitalWrite(SIGN_LED_PIN, HIGH);
    }
  }

  // Deactivate signage if timeout elapsed since last elephant message
  if (now - lastElephantMs >= SIGNAGE_TIMEOUT_MS) {
    signageActive = false;
    blinkStateOn = false;
    digitalWrite(SIGN_LED_PIN, LOW);
    Serial.println("Signage timeout expired -> signage deactivated.");
  }
}

// Vision: poll Grove Vision AI V2 and send vehicle messages via meshtastic if vehicles found
void pollVisionAndReport() {
  // The SSCMA library uses AI.invoke(1,false,false) returning 0 on success in earlier examples.
  // We will follow that pattern; adjust if your library differs.
  if (!AI.invoke(1, false, false)) { // single invoke, no filter, no image
    for (int i = 0; i < AI.boxes().size(); i++) {
      int score = AI.boxes()[i].score;
      if (score > VISION_SCORE_THRESHOLD) {
        String label = AI.boxes()[i].label;
        String lower = label;
        lower.toLowerCase();
        if (lower.indexOf("vehicle") >= 0 || lower.indexOf("car") >= 0 || lower.indexOf("truck") >= 0 || lower.indexOf("bus") >= 0) {
          unsigned long now = millis();
          if (now - lastVehicleSentAt >= VEHICLE_SEND_COOLDOWN_MS) {
            lastVehicleSentAt = now;
            // Create a compact JSON payload; adjust fields as your EleTect node expects
            String payload = String("{\"event\":\"vehicle\",\"score\":") + String(score) + String(",\"t\":") + String(now) + String("}");
            sendMeshtastic(payload);
            Serial.printf("Vehicle detected (score %d) -> sent vehicle message\n", score);
          } else {
            Serial.println("Vehicle detected but on cooldown (skip sending).");
          }
        }
      }
    }
  } else {
    // AI.invoke returned non-zero according to earlier API — nothing or error
    // You can print debug here if needed
    // Serial.println("Vision: no boxes or invoke returned non-zero");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println();
  Serial.println("Signage node booting...");

  // LED pin
  pinMode(SIGN_LED_PIN, OUTPUT);
  digitalWrite(SIGN_LED_PIN, LOW);

  // init meshtastic serial on Serial2 (hardware serial) with provided pins
  // Use Serial2 for XIAO ESP32S3 (change to Serial1 if you wired differently)
  Serial2.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_RX_PIN, MESHTASTIC_TX_PIN);
  delay(50);
  Serial.println("Meshtastic UART (Serial2) initialized.");

  // init vision ai - wire to hardware serial or the same Serial? Use atSerial (HardwareSerial(0)) as in examples
  #ifdef ESP32
    atSerial.begin(115200);
    AI.begin(&atSerial); // Grove Vision AI using hardware serial 0
  #else
    atSerial.begin(115200);
    AI.begin(&atSerial);
  #endif
  Serial.println("Grove Vision AI v2 UART initialized.");

  lastElephantMs = 0;
  signageActive = false;
  blinkStateOn = false;
  blinkStateChangeAt = millis();

  lastVisionPollAt = 0;
  lastVehicleSentAt = 0;

  Serial.println("Setup complete.");
}

// Main loop: poll meshtastic input, update blink state, and periodically poll vision
void loop() {
  // 1) Handle incoming meshtastic messages from other nodes
  pollMeshtasticSerial();

  // 2) Update blink state machine (non-blocking)
  updateBlinkState();

  // 3) Poll vision at intervals and optionally report vehicles
  if (millis() - lastVisionPollAt >= VISION_POLL_MS) {
    lastVisionPollAt = millis();
    pollVisionAndReport();
  }

  // small yield
  delay(10);
}