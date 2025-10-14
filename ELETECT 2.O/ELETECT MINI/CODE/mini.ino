/*
  EleTect Mini 
  --------------------------------
  - Camera: SSCMA_Micro_Core (DefaultCameraConfigXIAOS3)
  - Audio: Edge Impulse model on XIAO ESP32S3 mic
  - DFPlayer: deterrent sounds
  - “Eye” LED strobes for visual deterrence
  - Rotary selector (potentiometer on A0):
      OFF / AUTO / BIRD / DOG / MONKEY / SQUIRREL
*/

#define MODEL_NAME "EleTect_Mini"

SET_LOOP_TASK_STACK_SIZE(40 * 1024);

#include <Arduino.h>
#include <SSCMA_Micro_Core.h>
#include <esp_camera.h>
#include <Wire.h>
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>

#define EIDSP_QUANTIZE_FILTERBANK 0
#include <EleTect_Mini_inferencing.h>
#include <I2S.h>

/* --- Camera Micro Core objects --- */
SSCMAMicroCore instance;
SSCMAMicroCore::VideoCapture capture;

/* --- Hardware pins --- */
#define DFPLAYER_SOFT_RX 10 // DFPlayer TX -> XIAO pin 10
#define DFPLAYER_SOFT_TX 11 // DFPlayer RX -> XIAO pin 11
const int EYE_LEFT_PIN  = 2;
const int EYE_RIGHT_PIN = 4;
#define SELECT_POT_PIN A0

#define I2S_BCK_PIN  -1
#define I2S_WS_PIN   42
#define I2S_DIN_PIN  41
#define I2S_DOUT_PIN -1

/* --- DFPlayer folders --- */
const uint8_t FOLDER_BIRD     = 1;
const uint8_t FOLDER_DOG      = 2;
const uint8_t FOLDER_MONKEY   = 3;
const uint8_t FOLDER_SQUIRREL = 4;
const uint8_t NUM_FILES_PER_FOLDER = 20;

/* --- Timing --- */
const unsigned long PLAYBACK_COOLDOWN_MS = 8000UL;
unsigned long lastBirdAt = 0, lastDogAt = 0, lastMonkeyAt = 0, lastSquirrelAt = 0;
const unsigned long VISION_INTERVAL_MS = 800;
const unsigned long AUDIO_INTERVAL_MS  = 800;

/* --- State --- */
bool df_ok = false;

/* --- DFPlayer --- */
SoftwareSerial dfSoft(DFPLAYER_SOFT_RX, DFPLAYER_SOFT_TX);
DFRobotDFPlayerMini dfPlayer;

/* --- Audio (Edge Impulse) --- */
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
const float CONFIDENCE_THRESHOLD = 0.60f;

/* --- ADC and Random --- */
const int ADC_MAX = 4095;

/* --- Class mapping --- */
const char *classNames[] = {"bird", "dog", "monkey", "squirrel"};
const int NUM_CLASSES = 4;

/* --- Rotary selector options --- */
enum SelectorOption { SEL_OFF = 0, SEL_AUTO, SEL_BIRD, SEL_DOG, SEL_MONKEY, SEL_SQUIRREL, SEL_COUNT };
const char* selectorNames[SEL_COUNT] = { "OFF", "AUTO", "BIRD", "DOG", "MONKEY", "SQUIRREL" };
SelectorOption currentSelection = SEL_AUTO;
SelectorOption lastPrintedSelection = (SelectorOption)255;

/* ---------- Function Prototypes ---------- */
String detectVision();
String detectAudio();
void playRandomFromFolder(uint8_t folderNumber);
void strobeEyesRandom();
SelectorOption readPotSelector();

/* --- Audio Capture (Edge Impulse style) --- */
static void audio_inference_callback(uint32_t n_bytes) {
    for (int i = 0; i < (n_bytes >> 1); i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];
        if (inference.buf_count >= inference.n_samples) {
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
    const int32_t bytes_to_read = (uint32_t)arg;
    size_t bytes_read = bytes_to_read;
    while (record_status) {
        esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, (void*)sampleBuffer, bytes_to_read, &bytes_read, 100);
        if (bytes_read > 0) {
            for (int x = 0; x < bytes_to_read/2; x++)
                sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
            if (record_status)
                audio_inference_callback(bytes_to_read);
        }
    }
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (!inference.buffer) return false;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;
    record_status = true;
    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

static bool microphone_inference_record(void) {
    while (inference.buf_ready == 0) delay(10);
    inference.buf_ready = 0;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

/* --- Camera Detection --- */
String detectVision() {
    auto frame = capture.getManagedFrame();
    if (instance.invoke(frame) != MA_STATUS_OK)
        return "none";

    auto classes = instance.getClasses();
    for (const auto &cls : classes) {
        int target = cls.target;
        float score = cls.score;
        if (score > 0.5f && target >= 0 && target < NUM_CLASSES) {
            String label = String(classNames[target]);
            Serial.printf("Vision detected: %s (%.2f)\n", label.c_str(), score);
            return label;
        }
    }
    return "none";
}

/* --- Audio Detection --- */
String detectAudio() {
    if (!microphone_inference_record()) return "none";
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };
    if (run_classifier(&signal, &result, debug_nn) != EI_IMPULSE_OK)
        return "none";

    int pred_index = -1; float pred_value = 0.0f;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > pred_value) {
            pred_index = ix; pred_value = result.classification[ix].value;
        }
    }
    if (pred_index < 0 || pred_value < CONFIDENCE_THRESHOLD) return "none";
    const char *label = result.classification[pred_index].label;
    Serial.printf("Audio detected: %s (%.2f)\n", label, pred_value);
    return String(label);
}

/* --- DFPlayer --- */
void playRandomFromFolder(uint8_t folderNumber) {
    if (!df_ok) return;
    int fileCount = dfPlayer.readFileCountsInFolder(folderNumber);
    if (fileCount <= 0) fileCount = NUM_FILES_PER_FOLDER;
    int fileIndex = random(1, fileCount + 1);
    int vol = 28 + (random(0, 3));
    if (vol > 30) vol = 30;
    dfPlayer.volume(vol);
    Serial.printf("Playing folder %d file %d\n", folderNumber, fileIndex);
    dfPlayer.playFolder(folderNumber, fileIndex);
}

/* --- LED Eye Strobers --- */
void strobeEyesRandom() {
    int pattern = random(0,4);
    switch (pattern) {
        case 0:
            for (int i=0;i<6;i++) {
                digitalWrite(EYE_LEFT_PIN,HIGH); digitalWrite(EYE_RIGHT_PIN,HIGH);
                delay(80); digitalWrite(EYE_LEFT_PIN,LOW); digitalWrite(EYE_RIGHT_PIN,LOW); delay(80);
            } break;
        case 1:
            for (int i=0;i<8;i++) {
                digitalWrite(EYE_LEFT_PIN,(i%2)==0);
                digitalWrite(EYE_RIGHT_PIN,(i%2)!=0);
                delay(120);
            } break;
        case 2:
            for (int i=0;i<4;i++) {
                digitalWrite(EYE_LEFT_PIN,HIGH); delay(200); digitalWrite(EYE_LEFT_PIN,LOW); delay(80);
                digitalWrite(EYE_RIGHT_PIN,HIGH); delay(200); digitalWrite(EYE_RIGHT_PIN,LOW); delay(80);
            } break;
        case 3:
            for (int i=0;i<10;i++) {
                int p=random(0,3);
                digitalWrite(EYE_LEFT_PIN,p==1);
                digitalWrite(EYE_RIGHT_PIN,p==2);
                delay(random(40,180));
            } break;
    }
    digitalWrite(EYE_LEFT_PIN,LOW);
    digitalWrite(EYE_RIGHT_PIN,LOW);
}

/* --- Rotary Selector --- */
SelectorOption readPotSelector() {
    int raw = analogRead(SELECT_POT_PIN);
    int idx = map(raw, 0, ADC_MAX, 0, SEL_COUNT - 1);
    if (idx < 0) idx = 0;
    if (idx >= SEL_COUNT) idx = SEL_COUNT - 1;
    return (SelectorOption)idx;
}

/* --- Camera Setup --- */
bool cameraMicroCoreSetup() {
    if (capture.begin(SSCMAMicroCore::VideoCapture::DefaultCameraConfigXIAOS3) != MA_STATUS_OK)
        return false;
    if (instance.begin(SSCMAMicroCore::Config::DefaultConfig) != MA_STATUS_OK)
        return false;
    Serial.println("Camera ready.");
    return true;
}

/* --- Setup --- */
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Booting EleTect Mini (no RTC/SD)");

    // DFPlayer
    dfSoft.begin(9600);
    delay(200);
    if (dfPlayer.begin(dfSoft)) {
        df_ok = true;
        dfPlayer.volume(30);
        Serial.println("DFPlayer OK");
    } else {
        df_ok = false;
        Serial.println("DFPlayer missing");
    }

    // Eyes
    pinMode(EYE_LEFT_PIN, OUTPUT);
    pinMode(EYE_RIGHT_PIN, OUTPUT);
    digitalWrite(EYE_LEFT_PIN, LOW);
    digitalWrite(EYE_RIGHT_PIN, LOW);

    // Camera
    if (!cameraMicroCoreSetup())
        Serial.println("Camera init failed!");

    // Audio
    I2S.setAllPins(I2S_BCK_PIN, I2S_WS_PIN, I2S_DIN_PIN, I2S_DOUT_PIN, -1);
    if (!I2S.begin(PDM_MONO_MODE, 16000U, 16))
        Serial.println("I2S init failed!");
    else if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT))
        Serial.println("Audio buffer alloc failed!");

    // Rotary selector
    pinMode(SELECT_POT_PIN, INPUT);
    currentSelection = readPotSelector();

    randomSeed(micros());
    Serial.println("Setup complete.");
}

/* --- Main Loop --- */
unsigned long lastVisionMs = 0, lastAudioMs = 0;

void loop() {
    String vlabel = "none", alabel = "none";

    if (millis() - lastVisionMs >= VISION_INTERVAL_MS) {
        lastVisionMs = millis();
        vlabel = detectVision();
    }

    if (millis() - lastAudioMs >= AUDIO_INTERVAL_MS) {
        lastAudioMs = millis();
        alabel = detectAudio();
    }

    // Update selector
    SelectorOption sel = readPotSelector();
    if (sel != lastPrintedSelection) {
        Serial.print("Selector -> "); Serial.println(selectorNames[sel]);
        lastPrintedSelection = sel;
    }
    currentSelection = sel;

    // Determine detected animal
    String animal = (vlabel != "none") ? vlabel : ((alabel != "none") ? alabel : "none");

    // Skip if OFF
    if (currentSelection == SEL_OFF) {
        delay(50);
        return;
    }

    // Decide action
    bool shouldAct = false;
    if (currentSelection == SEL_AUTO) shouldAct = (animal != "none");
    else {
        const char* target = selectorNames[currentSelection];
        shouldAct = animal.equalsIgnoreCase(target);
    }

    if (!shouldAct || animal == "none") {
        delay(50);
        return;
    }

    // Perform deterrent action
    if (animal == "bird" && millis() - lastBirdAt >= PLAYBACK_COOLDOWN_MS) {
        lastBirdAt = millis();
        playRandomFromFolder(FOLDER_BIRD);
        strobeEyesRandom();
    } else if (animal == "dog" && millis() - lastDogAt >= PLAYBACK_COOLDOWN_MS) {
        lastDogAt = millis();
        playRandomFromFolder(FOLDER_DOG);
        strobeEyesRandom();
    } else if (animal == "monkey" && millis() - lastMonkeyAt >= PLAYBACK_COOLDOWN_MS) {
        lastMonkeyAt = millis();
        playRandomFromFolder(FOLDER_MONKEY);
        strobeEyesRandom();
    } else if (animal == "squirrel" && millis() - lastSquirrelAt >= PLAYBACK_COOLDOWN_MS) {
        lastSquirrelAt = millis();
        playRandomFromFolder(FOLDER_SQUIRREL);
        strobeEyesRandom();
    }

    delay(20);
}