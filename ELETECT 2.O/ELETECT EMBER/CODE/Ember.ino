/* Ember - wildfire detector
   - XIAO ESP32S3
   - Sensors: SHT10 (soil T/H), BME680 (air T/H/pressure/gas), MQ-2 (smoke)
   - When wildfire risk score > threshold, send ASCII alert over Serial1:
       e.g. "alert probable wildfire: score=0.82 t=36.1 hum=12 soilT=32 smoke=0.72"
  - UART alerts for Echo to forward.
*/

/* ------------- CONFIG ------------- */
#define ALERT_THRESHOLD 0.70f      // send alert when risk score >= this
#define ALERT_COOLDOWN_MS (5 * 60 * 1000UL) // 5 minutes between alerts
#define SAMPLE_INTERVAL_MS 5000UL  // sample sensors every 5s (tune as needed)

/* Sensor pins - change to match wiring */
#define SHT_DATA_PIN 21   // SHT10 data
#define SHT_CLK_PIN  22   // SHT10 clock

// MQ-2 analog pin (smoke)
#define MQ2_PIN A1        // change if needed

// Serial1 TX pin (Ember -> Echo). Echo listens on its Serial1 RX pin (4 by previous code).
// So wire Ember TX -> Echo RX (pin 4). We set rxPin = -1 and txPin = EMBER_TX_PIN below.
#define EMBER_TX_PIN 4
#define EMBER_BAUD 115200UL

/* I2C for BME680 (default SDA/SCL) - no pins needed unless using non-default pins */

/* ------------- LIBRARIES ------------- */
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SHT1x.h>   // SHT1x library (for SHT10)
                   // Install via Library Manager or from https://github.com/PaulStoffregen/SHT1x

/* ------------- SENSOR OBJECTS ------------- */
Adafruit_BME680 bme;            // I2C BME680
SHT1x sht(SHT_DATA_PIN, SHT_CLK_PIN); // SHT1x object (dataPin, clockPin)

/* ------------- STATE & TUNING ------------- */
unsigned long lastSampleAt = 0;
unsigned long lastAlertAt = 0;

/* Weights for scoring (sum to 1.0 ideally) */
const float W_AIR_TEMP     = 0.30f;  // higher air temp increases risk
const float W_SOIL_TEMP    = 0.25f;  // higher soil temp increases risk
const float W_REL_HUMIDITY = 0.25f;  // lower humidity increases risk (we invert it)
const float W_SMOKE        = 0.20f;  // higher smoke/gas increases risk

/* Expected normalization ranges  - tune for your environment */
const float AIR_TEMP_MIN = 10.0f;  // degC (maps to 0)
const float AIR_TEMP_MAX = 50.0f;  // degC (maps to 1)
const float SOIL_TEMP_MIN = 5.0f;
const float SOIL_TEMP_MAX = 60.0f;
const float RELHUM_MIN = 0.0f;     // percent
const float RELHUM_MAX = 100.0f;
const int   MQ2_ADC_MIN = 200;     // ADC baseline for "clean air" (tune)
const int   MQ2_ADC_MAX = 3500;    // ADC value for heavy smoke (tune)

/* ------------- Helper functions ------------- */
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// normalize value v from [minVal,maxVal] to 0..1
float normalize(float v, float minVal, float maxVal) {
  if (maxVal <= minVal) return 0.0f;
  return clampf((v - minVal) / (maxVal - minVal), 0.0f, 1.0f);
}

/* ------------- Setup ------------- */
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  Serial.println();
  Serial.println("Ember node booting...");

  // Init I2C & BME680
  Wire.begin();
  if (! bme.begin()) {
    Serial.println("BME680 not found - check wiring");
  } else {
    // recommended settings (oversampling/time)
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    // lower heater profile for gas sensor stability (tune if needed)
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    Serial.println("BME680 initialized.");
  }

  // Init SHT10 (SHT1x)
  // SHT1x constructor already set pin modes; no begin() required for many libs
  Serial.println("SHT1x ready (soil sensor) - check wiring.");

  // MQ-2 analog - no init required; optionally calibrate baseline
  analogReadResolution(12); // 0..4095 on ESP32
  Serial.println("MQ-2 analog ready.");

  // Init Serial1 for transmitting to Echo (we only TX)
  // note: rxPin = -1 because we don't read on Ember from Serial1
  Serial1.begin(EMBER_BAUD, SERIAL_8N1, -1, EMBER_TX_PIN);
  Serial.println("Serial1 (to Echo) started.");

  lastSampleAt = millis() - 1000;
  lastAlertAt = 0;
}

/* ------------- Main loop ------------- */
void loop() {
  unsigned long now = millis();
  if (now - lastSampleAt < SAMPLE_INTERVAL_MS) {
    delay(10);
    return;
  }
  lastSampleAt = now;

  // Read sensors
  // 1) BME680 - perform reading
  float airTemp = NAN, airHum = NAN, pressure = NAN;
  float gasRes = NAN;
  if (bme.performReading()) {
    airTemp = bme.temperature;      // degC
    airHum = bme.humidity;         // %
    pressure = bme.pressure / 100.0; // hPa
    gasRes = bme.gas_resistance;   // ohms (or raw units) - higher usually means cleaner air
  } else {
    Serial.println("BME680 reading failed.");
  }

  // 2) SHT10
  float soilTemp = NAN, soilHum = NAN;
  // SHT1x returns temperature in C and humidity in %
  // For soil temp/hum sensor module ensure probe is appropriate; SHT10 often used for air,
  // but some Robu modules are soil-capable — adjust if required.
  soilTemp = sht.readTemperatureC();
  soilHum  = sht.readHumidity();

  // 3) MQ-2 analog
  int mqRaw = analogRead(MQ2_PIN); // 0..4095
  // Map MQ-2: higher adc => more smoke (module output depends on driver circuit).
  // We normalize between MQ2_ADC_MIN..MQ2_ADC_MAX
  float smokeNorm = normalize((float)mqRaw, (float)MQ2_ADC_MIN, (float)MQ2_ADC_MAX);

  // Normalize features
  float airTempN = normalize(airTemp, AIR_TEMP_MIN, AIR_TEMP_MAX);      // 0..1
  float soilTempN = normalize(soilTemp, SOIL_TEMP_MIN, SOIL_TEMP_MAX);  // 0..1
  float relHumN = 1.0f - normalize(airHum, RELHUM_MIN, RELHUM_MAX);    // invert: low humidity -> high risk

  // For gas: we used MQ2 analog (smokeNorm). Also BME680 gas_resistance — lower means more gas.
  // We combine both: bmeGasN (where high means more VOC) = 1 - norm(gasRes)
  float bmeGasN = 0.0f;
  if (!isnan(gasRes)) {
    // choose reasonable min/max for gasRes (tune in field). Here we assume 100..200000 ohms
    const float BME_GAS_MIN = 100.0f;
    const float BME_GAS_MAX = 200000.0f;
    bmeGasN = 1.0f - normalize(gasRes, BME_GAS_MIN, BME_GAS_MAX); // higher -> more gas -> closer to 1
    bmeGasN = clampf(bmeGasN, 0.0f, 1.0f);
  }

  // Combine gas indicators: use max of MQ-2 and BME680 gas reading for robust detection
  float gasIndicator = max(smokeNorm, bmeGasN);

  // Compute weighted risk score
  float score = W_AIR_TEMP * airTempN + W_SOIL_TEMP * soilTempN + W_REL_HUMIDITY * relHumN + W_SMOKE * gasIndicator;
  score = clampf(score, 0.0f, 1.0f);

  // Debug print
  Serial.printf("Sensors: airT=%.2fC airH=%.2f%% soilT=%.2fC smokeRaw=%d smokeN=%.2f bmeGasN=%.2f score=%.3f\n",
                airTemp, airHum, soilTemp, mqRaw, smokeNorm, bmeGasN, score);

  // Decide: alert if score >= threshold and not in cooldown
  if (score >= ALERT_THRESHOLD) {
    unsigned long tnow = millis();
    if (tnow - lastAlertAt >= ALERT_COOLDOWN_MS) {
      lastAlertAt = tnow;
      // Build alert line expected by Echo
      // Keep it simple: "alert probable wildfire: score=0.82 airT=36.1 airH=12 soilT=32 smoke=0.72"
      char buf[256];
      snprintf(buf, sizeof(buf),
               "alert probable wildfire: score=%.2f airT=%.2f airH=%.2f soilT=%.2f smoke=%.2f",
               score, (isnan(airTemp)?-999.0f:airTemp), (isnan(airHum)?-999.0f:airHum),
               (isnan(soilTemp)?-999.0f:soilTemp), gasIndicator);
      String alertLine = String(buf);
      // Send over Serial1 to Echo
      Serial1.println(alertLine);
      // Also print locally for debug
      Serial.print("ALERT SENT -> ");
      Serial.println(alertLine);
    } else {
      Serial.println("High risk detected but in cooldown — alert suppressed.");
    }
  } else {
    // optionally send periodic status messages (disabled by default)
    // If you want to send periodic non-alert telemetry, implement with a different cooldown and format.
  }

  // end loop iteration
}