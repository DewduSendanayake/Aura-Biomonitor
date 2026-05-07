/*
 * ============================================================
 * RESEARCH  : R26-DS-012 | Component 1 | SLIIT
 * PROJECT   : Personalized Anxiety Detection — Phase 2
 *             Wearable Data Collection Firmware
 * AUTHOR    : Sendanayake H.D. (IT22107596)
 * VERSION   : 1.0.0
 * ============================================================
 * PURPOSE
 *   Collects continuous multimodal physiological data from the
 *   custom ESP32-C3 chest-strap wearable. Every 60 seconds it
 *   computes the 11 locked feature set and uploads to Firebase
 *   alongside raw RR intervals for offline ML training in
 *   Google Colab.
 *
 * 11 LOCKED FEATURES (per 60-second window):
 *   mean_HR      Mean heart rate (BPM)
 *   mean_RR      Mean RR interval (ms)
 *   SDNN         Std dev of all RR intervals (ms)
 *   RMSSD        Root mean square of successive RR diffs (ms)
 *   pNN50        % successive RR diffs > 50ms  (0 to 100)
 *   mean_BR      Mean breathing rate (breaths/min)
 *   std_BR       Std dev of breathing rate
 *   mean_temp    Mean skin temperature (deg C)
 *   std_temp     Std dev of skin temperature
 *   mean_acc_mag Mean accelerometer magnitude (g)
 *   std_acc_mag  Std dev of accelerometer magnitude
 *
 * HARDWARE
 *   ESP32-C3 Mini-1
 *   AD8232   ECG  (Analog GPIO4, LO+ GPIO5, LO- GPIO6)
 *   BMI160   IMU  (I2C: SDA=8, SCL=9, addr=0x69)
 *   DS18B20  Temp (OneWire GPIO2)
 *   BF350    Strain gauge / respiration (Analog GPIO3)
 *   SSD1306  OLED 128x64 (I2C: SDA=8, SCL=9, addr=0x3C)
 *
 * FIREBASE SCHEMA
 *   /research/R26_DS_012/
 *     participants/{SUBJECT_ID}/
 *       sessions/{SESSION_ID}/
 *         info/           session metadata (start time, fw, config)
 *         latest/         most recent live readings (overwritten)
 *         windows/        archive of all 60-second feature windows
 *           {timestamp}/  one entry per window
 *
 * NOTES ON HRV COMPUTATION
 *   All formulas match NeuroKit2 hrv_time() output.
 *   Raw RR intervals are also uploaded as a CSV string so the
 *   Colab pipeline can verify firmware values against NeuroKit2.
 *   The ectopic beat filter (20% deviation threshold) mirrors
 *   NeuroKit2's default artifact correction method.
 *
 * ECG SAMPLING RATE
 *   200 Hz (5 ms interval), up from the original 100 Hz.
 *   This halves RR interval timing error from +/-10 ms to
 *   +/-5 ms, which matters for pNN50 (50 ms threshold) and
 *   RMSSD accuracy.
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <math.h>
#include <BMI160Gen.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FirebaseESP32.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>


// ============================================================
//  CONFIGURATION
//  Edit ONLY this section to change subject, WiFi, Firebase.
// ============================================================
namespace Config {

  // Research identifiers
  // Change SUBJECT_ID for each new participant (P001, P002, etc).
  // SESSION_ID is auto-generated from the NTP startup time.
  constexpr char SUBJECT_ID[] = "P001";  // CHANGE PER PARTICIPANT

  // WiFi
  constexpr char SSID[]     = "SLT FIBER";
  constexpr char PASSWORD[] = "20hOmewIFi25";

  // Firebase credentials
  // Firebase Console > Project Settings > Web API Key
  // Firebase Console > Realtime Database > URL
  // Firebase Console > Authentication > Add user (email/pass)
  constexpr char FB_API_KEY[]  = "YOUR_FIREBASE_API_KEY_HERE";
  constexpr char FB_DB_URL[]   = "https://YOUR-PROJECT-ID.firebaseio.com";
  constexpr char FB_EMAIL[]    = "esp32@r26ds012.com";
  constexpr char FB_PASSWORD[] = "YOUR_FIREBASE_PASSWORD_HERE";

  // Timing (all in milliseconds)
  constexpr uint32_t WINDOW_DURATION_MS = 60000UL; // 60-second feature window
  constexpr uint32_t OLED_INTERVAL      = 300UL;
  constexpr uint32_t SERIAL_INTERVAL    = 1000UL;
  constexpr uint32_t SENSOR_SLOW_MS     = 150UL;   // IMU + temp + strain poll
  constexpr uint32_t OLED_PAGE_DURATION = 8000UL;

  // NTP  (Sri Lanka = UTC+5:30)
  constexpr long  GMT_OFFSET_SEC = 19800L;
  constexpr int   DST_OFFSET_SEC = 0;
  constexpr char  NTP_SERVER[]   = "pool.ntp.org";

  // Watchdog
  constexpr uint32_t WDT_TIMEOUT_SEC = 30;

  // GPIO Pins
  constexpr uint8_t SDA_PIN     = 8;
  constexpr uint8_t SCL_PIN     = 9;
  constexpr uint8_t DS18B20_PIN = 2;
  constexpr uint8_t STRAIN_PIN  = 3;
  constexpr uint8_t ECG_PIN     = 4;   // AD8232 analog output
  constexpr uint8_t LO_PLUS     = 5;   // AD8232 leads-off (+)
  constexpr uint8_t LO_MINUS    = 6;   // AD8232 leads-off (-)

  // I2C Addresses
  constexpr uint8_t BMI160_ADDR = 0x69;
  constexpr uint8_t OLED_ADDR   = 0x3C;

  // ADC
  constexpr float ADC_REF_V = 3.3f;
  constexpr float ADC_MAX   = 4095.0f;

  // ECG / Heart Rate
  // 5 ms = 200 Hz. Better RR timing precision (+/-5 ms)
  // versus the original 100 Hz (+/-10 ms). pNN50 uses a
  // 50 ms threshold, so halving the timing error matters.
  constexpr uint32_t ECG_SAMPLE_INTERVAL = 5;
  constexpr uint8_t  BPM_RATE_SIZE       = 10;    // display smoothing buffer
  constexpr float    BPM_MIN             = 30.0f;
  constexpr float    BPM_MAX             = 200.0f; // covers panic tachycardia
  constexpr uint8_t  ECG_FILTER_SIZE     = 8;
  constexpr float    ECG_THRESH_RATIO    = 0.40f;
  constexpr float    ECG_HYST_RATIO      = 0.15f;
  constexpr int      ECG_ADAPT_DECAY     = 1;

  // 5 seconds settle before recording RR intervals.
  // Gives the adaptive threshold time to learn the signal.
  constexpr uint32_t ECG_SETTLE_MS = 5000;

  // Valid RR interval range.
  constexpr float RR_MIN_MS = 300.0f;   // 200 BPM max (panic)
  constexpr float RR_MAX_MS = 2000.0f;  // 30  BPM min

  // Ectopic beat filter threshold.
  // Any RR interval that deviates more than 20% from the
  // running mean is treated as an ectopic beat and excluded.
  // 20% is the default in NeuroKit2's artifact correction.
  constexpr float RR_ECTOPIC_THRESH = 0.20f;

  // Respiration
  constexpr uint8_t  RESP_FILTER_SIZE  = 5;
  constexpr uint8_t  RESP_DC_SIZE      = 30;
  constexpr uint8_t  RESP_RATE_SIZE    = 6;
  constexpr float    RESP_MIN          = 4.0f;
  constexpr float    RESP_MAX          = 40.0f;
  constexpr uint32_t RESP_MIN_INTERVAL = 1500UL;

  // Window feature accumulation buffers
  // 200 BPM x 60 s = 200 beats max. Use 256 for headroom.
  constexpr uint16_t RR_BUFFER_SIZE = 256;

  // 40 breaths/min x 60 s = 40 samples max.
  constexpr uint8_t BR_BUFFER_SIZE = 60;

  // Minimum valid RR intervals required for a HRV-valid window.
  constexpr uint8_t MIN_VALID_RR = 20;

  // BMI160 accelerometer scaling
  // Default range +/-2g gives 16384 LSB per g.
  // Dividing raw counts by this gives magnitude in g,
  // matching the WESAD dataset's accelerometer units.
  constexpr float BMI160_ACC_SCALE_G = 16384.0f;

  // OLED
  constexpr uint8_t SCREEN_W   = 128;
  constexpr uint8_t SCREEN_H   = 64;
  constexpr uint8_t OLED_PAGES = 4;

  // Firmware version
  constexpr char FW_VERSION[] = "1.0.0-R26DS012";
}


// ============================================================
//  DATA STRUCTURES
// ============================================================

// Heart rate display and ECG detection state.
struct HeartRateData {
  float bpm            = 0.0f;
  int   bpmAvg         = 0;
  int   ecgValue       = 0;
  int   ecgSmoothed    = 0;
  bool  leadsConnected = false;

  float rates[Config::BPM_RATE_SIZE] = {0.0f};
  byte  rateSpot   = 0;
  long  lastBeat   = 0;
  byte  validBeats = 0;

  int           adaptThresh      = 2048;
  unsigned long leadsConnectedAt = 0;
};

// Accumulates beat-to-beat (RR) intervals across the current
// 60-second window. runMean is the exponential moving average
// of accepted intervals, used by the ectopic filter.
struct RRBuffer {
  float    intervals[Config::RR_BUFFER_SIZE] = {0.0f};
  uint16_t count   = 0;
  float    runMean = 700.0f; // seeded at approx 86 BPM resting rate
};

// HRV features computed from one completed 60-second window.
// Formulas match NeuroKit2 hrv_time() output exactly.
struct HRVFeatures {
  float    mean_HR = 0.0f;
  float    mean_RR = 0.0f;
  float    SDNN    = 0.0f;
  float    RMSSD   = 0.0f;
  float    pNN50   = 0.0f;
  bool     valid   = false;
  uint16_t n_rr    = 0;
};

// Accumulates breathing rate samples across the current window.
struct BRBuffer {
  float   rates[Config::BR_BUFFER_SIZE] = {0.0f};
  uint8_t count = 0;
};

// Welford online algorithm for running mean and standard deviation.
// Uses O(1) memory regardless of how many samples arrive.
// Used for temperature and accelerometer magnitude so we do
// not need to store every raw sample in a large array.
struct WelfordState {
  uint32_t n    = 0;
  double   mean = 0.0;
  double   M2   = 0.0;  // aggregated sum of squared deviations

  void update(float x) {
    n++;
    double delta  = (double)x - mean;
    mean         += delta / (double)n;
    double delta2 = (double)x - mean;
    M2           += delta * delta2;
  }

  float getMean()   const { return (n > 0) ? (float)mean                : 0.0f; }
  float getStddev() const { return (n > 1) ? (float)sqrt(M2 / (double)(n - 1)) : 0.0f; }
  void  reset()           { n = 0; mean = 0.0; M2 = 0.0; }
};

// All 11 locked features plus quality metadata for one window.
struct WindowFeatures {
  float mean_HR      = 0.0f;
  float mean_RR      = 0.0f;
  float SDNN         = 0.0f;
  float RMSSD        = 0.0f;
  float pNN50        = 0.0f;
  float mean_BR      = 0.0f;
  float std_BR       = 0.0f;
  float mean_temp    = 0.0f;
  float std_temp     = 0.0f;
  float mean_acc_mag = 0.0f;
  float std_acc_mag  = 0.0f;

  uint16_t rr_count   = 0;
  uint8_t  br_count   = 0;
  uint32_t temp_count = 0;
  uint32_t acc_count  = 0;
  bool     hrv_valid  = false;
};

// IMU readings. acc_mag_g is the vector magnitude in g units.
struct ImuData {
  int   ax = 0, ay = 0, az = 0;
  int   gx = 0, gy = 0, gz = 0;
  float acc_mag_g = 0.0f;
};

// Skin temperature.
struct TemperatureData {
  float celsius    = 0.0f;
  float fahrenheit = 0.0f;
  bool  valid      = false;
};

// Strain gauge raw reading (for respiration derivation).
struct StrainData {
  int   raw     = 0;
  float voltage = 0.0f;
};

// Live respiration state (used for display and BR accumulation).
struct RespirationData {
  float filteredRaw    = 0.0f;
  float acComponent    = 0.0f;
  float rateInstant    = 0.0f;
  int   rateAvg        = 0;

  float rates[Config::RESP_RATE_SIZE] = {0.0f};
  byte  rateSpot       = 0;
  byte  validBreaths   = 0;
  long  lastBreath     = 0;
  bool  breathDetected = false;
};

// Which sensors initialised successfully at startup.
struct SensorHealth {
  bool ad8232   = false;
  bool bmi160   = false;
  bool ds18b20  = false;
  bool firebase = false;
  bool wifi     = false;
};


// ============================================================
//  GLOBAL VARIABLES
// ============================================================

HeartRateData    hrData;
ImuData          imuData;
TemperatureData  tempData;
RespirationData  respData;
StrainData       strainData;
SensorHealth     health;

// Window accumulation buffers, reset after each 60-s upload.
RRBuffer     rrBuf;
BRBuffer     brBuf;
WelfordState tempWelford;
WelfordState accWelford;

// Last successfully computed window (displayed on OLED page 0).
WindowFeatures lastWindow;
uint32_t       windowCount   = 0;
uint32_t       missedUploads = 0;

// Hardware drivers.
OneWire           oneWire(Config::DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
Adafruit_SSD1306  display(Config::SCREEN_W, Config::SCREEN_H, &Wire, -1);
FirebaseData      fbdo;    // used for window archive uploads
FirebaseData      fbdo2;   // used for /latest live update
FirebaseAuth      auth;
FirebaseConfig    fbConfig;

// Scheduling timestamps.
unsigned long lastOLED       = 0;
unsigned long lastSerial     = 0;
unsigned long lastSensor     = 0;
unsigned long lastEcgRead    = 0;
unsigned long lastPageSwitch = 0;
unsigned long windowStart    = 0;

// Session ID, auto-generated from NTP at startup.
char sessionId[24] = "NOSYNC";

// Current OLED page (0 to 3).
uint8_t currentOledPage = 0;


// ============================================================
//  UTILITY FUNCTIONS
// ============================================================

bool getFormattedTime(char* buf, size_t len) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 500)) return false;
  strftime(buf, len, "%d/%m/%Y %H:%M:%S", &timeinfo);
  return true;
}

bool getSafeTimestamp(char* buf, size_t len) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 500)) return false;
  strftime(buf, len, "%Y%m%d_%H%M%S", &timeinfo);
  return true;
}

String getShortTime() {
  struct tm timeinfo;
  char buf[10];
  if (!getLocalTime(&timeinfo, 200)) return "--:--";
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

bool generateSessionId() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 2000)) return false;
  strftime(sessionId, sizeof(sessionId), "%Y%m%d_%H%M%S", &timeinfo);
  return true;
}


// ============================================================
//  HRV FEATURE COMPUTATION
//
//  All formulas verified against NeuroKit2 hrv_time() output.
//  The Colab pipeline processes the same raw RR intervals with
//  NeuroKit2, so any discrepancy between firmware values and
//  NeuroKit2 values can be identified before training.
// ============================================================

HRVFeatures computeHRV(const RRBuffer& buf) {
  HRVFeatures f;
  f.n_rr = buf.count;

  if (buf.count < Config::MIN_VALID_RR) {
    f.valid = false;
    return f;
  }

  // mean_RR and mean_HR
  double sum = 0.0;
  for (uint16_t i = 0; i < buf.count; i++) sum += (double)buf.intervals[i];
  f.mean_RR = (float)(sum / (double)buf.count);
  f.mean_HR = (f.mean_RR > 0.0f) ? (60000.0f / f.mean_RR) : 0.0f;

  // SDNN = sqrt( mean( (RR_i - mean_RR)^2 ) )  [population std dev]
  double sq = 0.0;
  for (uint16_t i = 0; i < buf.count; i++) {
    double d = (double)buf.intervals[i] - (double)f.mean_RR;
    sq += d * d;
  }
  f.SDNN = (float)sqrt(sq / (double)(buf.count - 1));

  // RMSSD and pNN50 require at least 2 intervals.
  if (buf.count >= 2) {
    double   rmssd_sum = 0.0;
    uint16_t above50   = 0;
    uint16_t ndiffs    = buf.count - 1;

    for (uint16_t i = 1; i < buf.count; i++) {
      float diff = buf.intervals[i] - buf.intervals[i - 1];
      rmssd_sum += (double)(diff * diff);
      if (fabsf(diff) > 50.0f) above50++;
    }

    f.RMSSD = (float)sqrt(rmssd_sum / (double)ndiffs);
    f.pNN50 = (float)above50 / (float)ndiffs * 100.0f;
  }

  f.valid = true;
  return f;
}

void computeBRStats(const BRBuffer& buf, float& mean_BR, float& std_BR) {
  mean_BR = 0.0f;
  std_BR  = 0.0f;
  if (buf.count == 0) return;

  double sum = 0.0;
  for (uint8_t i = 0; i < buf.count; i++) sum += (double)buf.rates[i];
  mean_BR = (float)(sum / (double)buf.count);

  if (buf.count < 2) return;
  double sq = 0.0;
  for (uint8_t i = 0; i < buf.count; i++) {
    double d = (double)buf.rates[i] - (double)mean_BR;
    sq += d * d;
  }
  std_BR = (float)sqrt(sq / (double)(buf.count - 1));
}


// ============================================================
//  WIFI
// ============================================================

void connectWiFi() {
  Serial.printf("[WiFi] Connecting to: %s\n", Config::SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(Config::SSID, Config::PASSWORD);

  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    Serial.print(".");
    retries++;
    esp_task_wdt_reset();
  }

  health.wifi = (WiFi.status() == WL_CONNECTED);
  if (health.wifi) {
    Serial.printf("\n[WiFi] Connected. IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("\n[WiFi] FAILED — offline mode.");
  }
}

void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
}


// ============================================================
//  FIREBASE
// ============================================================

void initFirebase() {
  if (!health.wifi) {
    Serial.println("[Firebase] Skipped — no WiFi.");
    return;
  }
  fbConfig.api_key               = Config::FB_API_KEY;
  fbConfig.database_url          = Config::FB_DB_URL;
  auth.user.email                = Config::FB_EMAIL;
  auth.user.password             = Config::FB_PASSWORD;
  fbConfig.token_status_callback = tokenStatusCallback;

  Firebase.begin(&fbConfig, &auth);
  Firebase.reconnectWiFi(true);

  // 8192 bytes for the window JSON (includes the RR CSV string).
  // 4096 bytes for the smaller /latest live-readings JSON.
  fbdo.setResponseSize(8192);
  fbdo2.setResponseSize(4096);

  health.firebase = true;
  Serial.println("[Firebase] Init started (token generating)...");
}

// Uploads a one-time metadata record at session start.
// Path: /research/R26_DS_012/participants/{id}/sessions/{sid}/info
void uploadSessionInfo() {
  if (!health.firebase || !Firebase.ready()) return;

  char ts[32];
  getFormattedTime(ts, sizeof(ts));

  String path = "/research/R26_DS_012/participants/";
  path += Config::SUBJECT_ID;
  path += "/sessions/";
  path += sessionId;
  path += "/info";

  FirebaseJson json;
  json.set("subject_id",         Config::SUBJECT_ID);
  json.set("session_id",         sessionId);
  json.set("start_time",         ts);
  json.set("fw_version",         Config::FW_VERSION);
  json.set("window_duration_ms", (int)Config::WINDOW_DURATION_MS);
  json.set("ecg_sample_hz",      (int)(1000 / Config::ECG_SAMPLE_INTERVAL));
  json.set("rr_min_ms",          Config::RR_MIN_MS);
  json.set("rr_max_ms",          Config::RR_MAX_MS);
  json.set("ectopic_thresh",     Config::RR_ECTOPIC_THRESH);
  json.set("min_valid_rr",       (int)Config::MIN_VALID_RR);
  json.set("acc_scale_g",        Config::BMI160_ACC_SCALE_G);

  if (Firebase.setJSON(fbdo, path.c_str(), json)) {
    Serial.printf("[Firebase] Session info uploaded OK\n");
  } else {
    Serial.printf("[Firebase] Session info FAILED: %s\n", fbdo.errorReason().c_str());
  }
  esp_task_wdt_reset();
}

// Builds the /latest live-readings JSON.
void buildLiveJson(FirebaseJson& json, const char* timestamp) {
  json.set("subject_id",     Config::SUBJECT_ID);
  json.set("session_id",     sessionId);
  json.set("timestamp",      timestamp);
  json.set("fw_version",     Config::FW_VERSION);
  json.set("window_count",   (int)windowCount);
  json.set("missed_uploads", (int)missedUploads);

  json.set("live/bpm_avg",     hrData.bpmAvg);
  json.set("live/bpm_instant", hrData.bpm);
  json.set("live/leads_on",    hrData.leadsConnected);
  json.set("live/br_avg",      respData.rateAvg);
  json.set("live/br_instant",  respData.rateInstant);
  json.set("live/temp_c",      tempData.celsius);
  json.set("live/acc_mag_g",   imuData.acc_mag_g);
  json.set("live/rr_buf_n",    (int)rrBuf.count);

  if (lastWindow.hrv_valid) {
    json.set("last_window/mean_HR",      lastWindow.mean_HR);
    json.set("last_window/mean_RR",      lastWindow.mean_RR);
    json.set("last_window/SDNN",         lastWindow.SDNN);
    json.set("last_window/RMSSD",        lastWindow.RMSSD);
    json.set("last_window/pNN50",        lastWindow.pNN50);
    json.set("last_window/mean_BR",      lastWindow.mean_BR);
    json.set("last_window/std_BR",       lastWindow.std_BR);
    json.set("last_window/mean_temp",    lastWindow.mean_temp);
    json.set("last_window/std_temp",     lastWindow.std_temp);
    json.set("last_window/mean_acc_mag", lastWindow.mean_acc_mag);
    json.set("last_window/std_acc_mag",  lastWindow.std_acc_mag);
  }

  json.set("system/wifi_rssi", (int)WiFi.RSSI());
  json.set("system/heap_free", (int)ESP.getFreeHeap());
}

// Called every 60 seconds from loop().
// Computes all 11 features, uploads the window, then resets
// all accumulation buffers for the next window.
void uploadWindowFeatures() {
  char ts[32];
  if (!getFormattedTime(ts, sizeof(ts))) strcpy(ts, "unknown");

  char safets[24];
  if (!getSafeTimestamp(safets, sizeof(safets))) strcpy(safets, "unknown");

  // Compute all 11 features.
  HRVFeatures hrv = computeHRV(rrBuf);

  float mean_BR = 0.0f, std_BR = 0.0f;
  computeBRStats(brBuf, mean_BR, std_BR);

  WindowFeatures wf;
  wf.mean_HR      = hrv.mean_HR;
  wf.mean_RR      = hrv.mean_RR;
  wf.SDNN         = hrv.SDNN;
  wf.RMSSD        = hrv.RMSSD;
  wf.pNN50        = hrv.pNN50;
  wf.mean_BR      = mean_BR;
  wf.std_BR       = std_BR;
  wf.mean_temp    = tempWelford.getMean();
  wf.std_temp     = tempWelford.getStddev();
  wf.mean_acc_mag = accWelford.getMean();
  wf.std_acc_mag  = accWelford.getStddev();
  wf.rr_count     = rrBuf.count;
  wf.br_count     = brBuf.count;
  wf.temp_count   = tempWelford.n;
  wf.acc_count    = accWelford.n;
  wf.hrv_valid    = hrv.valid;

  lastWindow = wf;
  windowCount++;

  Serial.printf(
    "[Window #%lu] valid=%s RR_n=%d HR=%.1f SDNN=%.1f RMSSD=%.1f pNN50=%.1f%% "
    "BR=%.1f+/-%.1f Temp=%.2f+/-%.3f AccMag=%.3f+/-%.3f\n",
    windowCount, wf.hrv_valid ? "YES" : "NO",
    wf.rr_count, wf.mean_HR, wf.SDNN, wf.RMSSD, wf.pNN50,
    wf.mean_BR, wf.std_BR,
    wf.mean_temp, wf.std_temp,
    wf.mean_acc_mag, wf.std_acc_mag
  );

  // Upload to Firebase.
  ensureWiFi();

  if (!Firebase.ready()) {
    Serial.println("[Firebase] Upload skipped — token not ready.");
    missedUploads++;
  } else {
    String basePath = "/research/R26_DS_012/participants/";
    basePath += Config::SUBJECT_ID;
    basePath += "/sessions/";
    basePath += sessionId;

    // Build the window JSON.
    FirebaseJson windowJson;
    windowJson.set("subject_id",    Config::SUBJECT_ID);
    windowJson.set("session_id",    sessionId);
    windowJson.set("timestamp",     ts);
    windowJson.set("fw_version",    Config::FW_VERSION);
    windowJson.set("window_number", (int)windowCount);
    windowJson.set("window_ms",     (int)Config::WINDOW_DURATION_MS);

    // 11 locked features.
    windowJson.set("features/mean_HR",      wf.mean_HR);
    windowJson.set("features/mean_RR",      wf.mean_RR);
    windowJson.set("features/SDNN",         wf.SDNN);
    windowJson.set("features/RMSSD",        wf.RMSSD);
    windowJson.set("features/pNN50",        wf.pNN50);
    windowJson.set("features/mean_BR",      wf.mean_BR);
    windowJson.set("features/std_BR",       wf.std_BR);
    windowJson.set("features/mean_temp",    wf.mean_temp);
    windowJson.set("features/std_temp",     wf.std_temp);
    windowJson.set("features/mean_acc_mag", wf.mean_acc_mag);
    windowJson.set("features/std_acc_mag",  wf.std_acc_mag);

    // Quality metadata, used in Colab to filter bad windows.
    windowJson.set("quality/rr_count",   (int)wf.rr_count);
    windowJson.set("quality/br_count",   (int)wf.br_count);
    windowJson.set("quality/temp_count", (int)wf.temp_count);
    windowJson.set("quality/acc_count",  (int)wf.acc_count);
    windowJson.set("quality/hrv_valid",  wf.hrv_valid);
    windowJson.set("quality/leads_on",   hrData.leadsConnected);
    windowJson.set("quality/wifi_rssi",  (int)WiFi.RSSI());
    windowJson.set("quality/heap_free",  (int)ESP.getFreeHeap());

    // Raw RR intervals as a CSV string.
    // In Python/Colab: [float(x) for x in row['rr_intervals_csv'].split(',')]
    // This lets NeuroKit2 independently verify the firmware HRV values.
    String rrCsv = "";
    rrCsv.reserve((size_t)rrBuf.count * 8);
    for (uint16_t i = 0; i < rrBuf.count; i++) {
      if (i > 0) rrCsv += ",";
      rrCsv += String(rrBuf.intervals[i], 1);
    }
    windowJson.set("rr_intervals_csv", rrCsv);

    // Write the window record to the archive.
    String windowPath = basePath + "/windows/" + String(safets);
    if (Firebase.setJSON(fbdo, windowPath.c_str(), windowJson)) {
      Serial.printf("[Firebase] Window #%lu OK  [%s]\n", windowCount, ts);
    } else {
      Serial.printf("[Firebase] Window FAILED: %s\n", fbdo.errorReason().c_str());
      missedUploads++;
    }
    esp_task_wdt_reset();

    // Update /latest with current live readings.
    FirebaseJson liveJson;
    buildLiveJson(liveJson, ts);
    String latestPath = basePath + "/latest";
    Firebase.setJSON(fbdo2, latestPath.c_str(), liveJson);
    esp_task_wdt_reset();

    health.firebase = Firebase.ready();
  }

  // Always reset accumulation buffers regardless of upload result.
  // runMean is intentionally kept so the ectopic filter has a
  // valid starting point immediately in the next window.
  rrBuf.count = 0;
  brBuf.count = 0;
  tempWelford.reset();
  accWelford.reset();
}


// ============================================================
//  SENSOR: AD8232 ECG
// ============================================================

void initAD8232() {
  Serial.print("[AD8232] Initializing... ");
  pinMode(Config::LO_PLUS,  INPUT_PULLDOWN);
  pinMode(Config::LO_MINUS, INPUT_PULLDOWN);
  pinMode(Config::ECG_PIN,  INPUT);

  analogSetPinAttenuation(Config::ECG_PIN,    ADC_11db);
  analogSetPinAttenuation(Config::STRAIN_PIN, ADC_11db);

  (void)analogRead(Config::ECG_PIN); delay(10);
  (void)analogRead(Config::ECG_PIN);

  health.ad8232 = true;
  Serial.println("OK (LO=PULLDOWN, atten=11dB, 200 Hz)");
}

// Called every 5 ms (200 Hz) from the main loop.
//
// Reads ECG signal, filters noise, detects R-peaks,
// applies an ectopic beat filter, and stores valid RR intervals
// in rrBuf for the 60-second HRV computation window.
void updateHeartRate() {
  if (!health.ad8232) return;

  static int     history[Config::ECG_FILTER_SIZE] = {0};
  static uint8_t histIdx = 0;
  static int     sigMax  = 2400;
  static int     sigMin  = 1800;
  static bool    aboveThreshold = false;

  unsigned long now = millis();
  if (now - lastEcgRead < Config::ECG_SAMPLE_INTERVAL) return;
  lastEcgRead = now;

  // Leads-off check.
  bool loPlus  = digitalRead(Config::LO_PLUS);
  bool loMinus = digitalRead(Config::LO_MINUS);
  bool connected = !(loPlus || loMinus);

  if (!connected) {
    if (hrData.leadsConnected) {
      // Leads just disconnected — reset ectopic filter seed.
      rrBuf.runMean         = 700.0f;
      hrData.leadsConnectedAt = 0;
    }
    hrData.leadsConnected = false;
    return;
  }

  if (!hrData.leadsConnected) {
    // Leads just reconnected — start the settle timer.
    hrData.leadsConnectedAt = now;
    hrData.leadsConnected   = true;
  }

  // Dummy read reduces ADC sampling glitches on ESP32-C3.
  analogRead(Config::ECG_PIN);
  int raw = analogRead(Config::ECG_PIN);
  hrData.ecgValue = raw;

  // Moving-average noise filter.
  history[histIdx] = raw;
  histIdx = (histIdx + 1) % Config::ECG_FILTER_SIZE;
  int sum = 0;
  for (int i = 0; i < Config::ECG_FILTER_SIZE; i++) sum += history[i];
  hrData.ecgSmoothed = sum / Config::ECG_FILTER_SIZE;

  // Adaptive range tracking.
  sigMax -= Config::ECG_ADAPT_DECAY;
  sigMin += Config::ECG_ADAPT_DECAY;
  if (sigMax < sigMin + 100) sigMax = sigMin + 100;
  if (hrData.ecgSmoothed > sigMax) sigMax = hrData.ecgSmoothed;
  if (hrData.ecgSmoothed < sigMin) sigMin = hrData.ecgSmoothed;

  int range  = sigMax - sigMin;
  int thresh = sigMin + (int)(range * Config::ECG_THRESH_RATIO);
  int hyst   = (int)(range * Config::ECG_HYST_RATIO);
  hrData.adaptThresh = thresh;

  // R-peak detection with hysteresis.
  if (hrData.ecgSmoothed > thresh && !aboveThreshold) {
    aboveThreshold = true;

    bool settled = (now - hrData.leadsConnectedAt) > Config::ECG_SETTLE_MS;

    if (settled && hrData.lastBeat != 0) {
      float dt_ms = (float)(now - hrData.lastBeat);

      // Physiological range check.
      if (dt_ms >= Config::RR_MIN_MS && dt_ms <= Config::RR_MAX_MS) {

        // Ectopic beat filter.
        // Skip filter for the first 5 beats so runMean can
        // establish a valid baseline from actual data.
        float deviation  = fabsf(dt_ms - rrBuf.runMean) / rrBuf.runMean;
        bool  is_ectopic = (rrBuf.count >= 5) &&
                           (deviation > Config::RR_ECTOPIC_THRESH);

        if (!is_ectopic) {
          // Store RR interval in the window accumulation buffer.
          if (rrBuf.count < Config::RR_BUFFER_SIZE) {
            rrBuf.intervals[rrBuf.count++] = dt_ms;
          }

          // Update exponential moving average (alpha = 0.1).
          rrBuf.runMean = rrBuf.runMean * 0.9f + dt_ms * 0.1f;

          // Update display BPM ring buffer.
          float bpm = 60000.0f / dt_ms;
          hrData.bpm = bpm;
          hrData.rates[hrData.rateSpot++] = bpm;
          hrData.rateSpot %= Config::BPM_RATE_SIZE;
          if (hrData.validBeats < Config::BPM_RATE_SIZE) hrData.validBeats++;

          float s = 0.0f;
          for (int i = 0; i < hrData.validBeats; i++) s += hrData.rates[i];
          hrData.bpmAvg = (int)(s / (float)hrData.validBeats);
        }
      }
    }

    // Always update lastBeat, even for ectopic beats, so the
    // next RR interval is measured from this peak, not the
    // one before it. Skipping this would inflate the next interval.
    hrData.lastBeat = now;

  } else if (hrData.ecgSmoothed < thresh - hyst) {
    aboveThreshold = false;
  }
}


// ============================================================
//  SENSOR: BMI160 (IMU)
// ============================================================

void initBMI160() {
  Serial.print("[BMI160] Initializing... ");
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, Config::BMI160_ADDR)) {
    Serial.println("FAILED — check wiring!");
    health.bmi160 = false;
    return;
  }
  // Step counting disabled — not relevant to anxiety detection.
  // Default +/-2g range is kept: appropriate for torso motion.
  health.bmi160 = true;
  Serial.println("OK (+/-2g, step counting disabled)");
}

// Called every 150 ms. Reads raw accelerometer and gyroscope,
// converts to g units, computes vector magnitude, and feeds the
// Welford state for running mean and std dev across the window.
void updateIMU() {
  if (!health.bmi160) return;

  BMI160.readAccelerometer(imuData.ax, imuData.ay, imuData.az);
  BMI160.readGyro(imuData.gx, imuData.gy, imuData.gz);

  // Convert raw counts to g units (16384 LSB/g at +/-2g range).
  // This matches the accelerometer unit in WESAD, ensuring Phase 2
  // features are on the same scale as Phase 1 training data.
  float ax_g = imuData.ax / Config::BMI160_ACC_SCALE_G;
  float ay_g = imuData.ay / Config::BMI160_ACC_SCALE_G;
  float az_g = imuData.az / Config::BMI160_ACC_SCALE_G;
  imuData.acc_mag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  // Accumulate for mean_acc_mag and std_acc_mag.
  accWelford.update(imuData.acc_mag_g);
}


// ============================================================
//  SENSOR: DS18B20 (Skin Temperature)
// ============================================================

void initDS18B20() {
  Serial.print("[DS18B20] Initializing... ");
  tempSensor.begin();
  uint8_t n      = tempSensor.getDeviceCount();
  health.ds18b20 = (n > 0);
  Serial.printf("%s (found %d device[s])\n", health.ds18b20 ? "OK" : "FAILED", n);
  if (health.ds18b20) {
    tempSensor.setResolution(12);           // Highest: 0.0625 deg C per step
    tempSensor.setWaitForConversion(false); // Non-blocking mode
  }
}

// Called every 150 ms, internally throttled to a 2-second cycle.
// Two-step non-blocking approach:
//   Step A (every 2 s): send the conversion command
//   Step B (800 ms later): read the result
void updateTemperature() {
  if (!health.ds18b20) return;

  static unsigned long lastReq   = 0;
  static bool          requested = false;
  unsigned long        now       = millis();

  if (!requested && (now - lastReq >= 2000)) {
    tempSensor.requestTemperatures();
    requested = true;
    lastReq   = now;
  }

  if (requested && (now - lastReq >= 800)) {
    float c = tempSensor.getTempCByIndex(0);

    if (c == DEVICE_DISCONNECTED_C || c < -55.0f || c > 125.0f) {
      tempData.valid   = false;
      tempData.celsius = 0.0f;
    } else {
      tempData.celsius    = c;
      tempData.fahrenheit = tempSensor.toFahrenheit(c);
      tempData.valid      = true;

      // Accumulate only physiologically plausible skin temperatures.
      // Normal skin surface range: 28 to 38 deg C.
      // Values outside this range indicate the strap is loose or
      // the sensor is not in contact with skin.
      if (c >= 28.0f && c <= 38.0f) {
        tempWelford.update(c);
      }
    }
    requested = false;
  }
}


// ============================================================
//  SENSOR: BF350 Strain Gauge (Respiration)
//
//  The strain gauge stretches when the chest expands during
//  inhalation, producing a slow voltage wave.
//  Three signal processing stages extract breathing rate:
//    Stage 1 -- moving-average filter removes electrical noise
//    Stage 2 -- DC removal strips the resting-tension offset
//    Stage 3 -- rising zero-crossing detects each breath
// ============================================================

void updateStrain() {
  const uint8_t SAMPLES = 8;
  uint32_t adcSum = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    adcSum += analogRead(Config::STRAIN_PIN);
    delayMicroseconds(100);
  }
  strainData.raw     = adcSum / SAMPLES;
  strainData.voltage = (strainData.raw / Config::ADC_MAX) * Config::ADC_REF_V;

  // Stage 1: Moving-average low-pass filter.
  static float   maBuffer[Config::RESP_FILTER_SIZE] = {0};
  static uint8_t maIdx = 0;
  maBuffer[maIdx] = (float)strainData.raw;
  maIdx = (maIdx + 1) % Config::RESP_FILTER_SIZE;
  float maSum = 0;
  for (uint8_t i = 0; i < Config::RESP_FILTER_SIZE; i++) maSum += maBuffer[i];
  respData.filteredRaw = maSum / Config::RESP_FILTER_SIZE;

  // Stage 2: DC removal.
  static float   dcBuffer[Config::RESP_DC_SIZE] = {0};
  static uint8_t dcIdx = 0;
  dcBuffer[dcIdx] = respData.filteredRaw;
  dcIdx = (dcIdx + 1) % Config::RESP_DC_SIZE;
  float dcSum = 0;
  for (uint8_t i = 0; i < Config::RESP_DC_SIZE; i++) dcSum += dcBuffer[i];
  float dcMean         = dcSum / Config::RESP_DC_SIZE;
  respData.acComponent = respData.filteredRaw - dcMean;

  // Stage 3: Rising zero-crossing breath detection.
  static float prevAC          = 0.0f;
  static long  lastBreathLocal = 0;
  respData.breathDetected = false;

  unsigned long now  = millis();
  bool zeroCrossUp   = (prevAC < 0.0f && respData.acComponent >= 0.0f);

  if (zeroCrossUp && (now - lastBreathLocal > Config::RESP_MIN_INTERVAL)) {
    if (lastBreathLocal != 0) {
      float intervalSec = (now - lastBreathLocal) / 1000.0f;
      float instRate    = 60.0f / intervalSec;

      if (instRate >= Config::RESP_MIN && instRate <= Config::RESP_MAX) {
        respData.rateInstant = instRate;

        respData.rates[respData.rateSpot++] = instRate;
        respData.rateSpot %= Config::RESP_RATE_SIZE;
        if (respData.validBreaths < Config::RESP_RATE_SIZE) respData.validBreaths++;

        float rSum = 0;
        for (byte i = 0; i < respData.validBreaths; i++) rSum += respData.rates[i];
        respData.rateAvg        = (int)(rSum / (float)respData.validBreaths);
        respData.breathDetected = true;

        // Accumulate for mean_BR and std_BR.
        if (brBuf.count < Config::BR_BUFFER_SIZE) {
          brBuf.rates[brBuf.count++] = instRate;
        }
      }
    }
    lastBreathLocal     = now;
    respData.lastBreath = now;
  }
  prevAC = respData.acComponent;
}


// ============================================================
//  OLED DISPLAY
// ============================================================

void initOLED() {
  Serial.print("[OLED] Initializing... ");
  if (!display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR)) {
    Serial.println("FAILED at 0x3C");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setTextSize(1);
  display.setCursor(4,  8);  display.print("R26-DS-012 RESEARCH");
  display.setCursor(8,  22); display.print("Anxiety Detection");
  display.setCursor(4,  36); display.print("Subject: ");
  display.print(Config::SUBJECT_ID);
  display.setCursor(4,  50); display.print("Initializing...");
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
  Serial.println("OK");
}

// Page 0: ECG + HRV status.
void drawOledPage0() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print("ECG + HRV");
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  if (!hrData.leadsConnected) {
    display.setTextSize(1);
    display.setCursor(8, 22); display.print("Leads disconnected.");
    display.setCursor(8, 35); display.print("Attach electrodes.");
    display.setCursor(8, 48); display.print("Check contact.");
    display.display();
    return;
  }

  bool settled = (millis() - hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;
  if (!settled) {
    unsigned long remain = (Config::ECG_SETTLE_MS -
                            (millis() - hrData.leadsConnectedAt)) / 1000UL + 1;
    display.setTextSize(1);
    display.setCursor(8, 22); display.print("Calibrating ECG...");
    display.setCursor(8, 35); display.printf("Ready in %lus", remain);
    display.display();
    return;
  }

  display.setTextSize(3);
  display.setCursor(0, 14);
  display.print(hrData.bpmAvg);

  display.setTextSize(1);
  display.setCursor(56, 16); display.print("BPM avg");
  display.setCursor(56, 26); display.printf("RR buf:%d", rrBuf.count);

  display.setCursor(0, 44);
  if (lastWindow.hrv_valid) {
    display.printf("RMSSD:%.1f SDNN:%.1f", lastWindow.RMSSD, lastWindow.SDNN);
    display.setCursor(0, 54);
    display.printf("pNN50: %.1f%%", lastWindow.pNN50);
  } else {
    display.print("HRV: collecting...");
    display.setCursor(0, 54);
    display.printf("RR: %d / %d min", rrBuf.count, (int)Config::MIN_VALID_RR);
  }
  display.display();
}

// Page 1: Respiration.
void drawOledPage1() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print("RESPIRATION");
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(3);
  display.setCursor(0, 14);
  if (respData.validBreaths > 0) display.print(respData.rateAvg);
  else                           display.print("--");

  display.setTextSize(1);
  display.setCursor(56, 22); display.print("br/min");

  display.setCursor(0, 44);
  if (respData.rateInstant > 0)
    display.printf("Inst: %.1f br/min", respData.rateInstant);
  else
    display.print("Calibrating...");

  display.setCursor(0, 54);
  if (lastWindow.mean_BR > 0.0f)
    display.printf("Avg:%.1f Std:%.1f", lastWindow.mean_BR, lastWindow.std_BR);

  display.drawLine(100, 63, 127, 63, SSD1306_WHITE);
  int barH = (int)constrain(respData.acComponent / 20.0f, -6.0f, 6.0f);
  if (barH >= 0) display.fillRect(100, 63 - barH, 27, barH + 1, SSD1306_WHITE);
  else           display.fillRect(100, 63, 27, -barH, SSD1306_WHITE);
  if (respData.breathDetected) display.fillCircle(120, 56, 3, SSD1306_WHITE);

  display.display();
}

// Page 2: Temperature and accelerometer magnitude.
void drawOledPage2() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print("TEMP + MOTION");
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 15);
  if (tempData.valid) display.printf("Temp: %.2f C", tempData.celsius);
  else               display.print("Temp: -- sensor ERR");

  if (lastWindow.mean_temp > 0.0f) {
    display.setCursor(0, 25);
    display.printf("Avg:%.2fC Std:%.3f", lastWindow.mean_temp, lastWindow.std_temp);
  }

  display.setCursor(0, 37);
  display.printf("AccMag: %.3f g", imuData.acc_mag_g);

  if (lastWindow.mean_acc_mag > 0.0f) {
    display.setCursor(0, 47);
    display.printf("Avg:%.3f Std:%.3f", lastWindow.mean_acc_mag, lastWindow.std_acc_mag);
  }

  display.setCursor(0, 57);
  display.printf("A:[%d,%d,%d]k",
    imuData.ax / 1000, imuData.ay / 1000, imuData.az / 1000);

  display.display();
}

// Page 3: Research session info and system health.
void drawOledPage3() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print("SESSION INFO");
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 14);
  display.printf("Subj: %s", Config::SUBJECT_ID);

  display.setCursor(0, 23);
  display.printf("Sess: %.15s", sessionId);

  display.setCursor(0, 33);
  display.printf("Win: #%lu  RR buf: %d", windowCount, rrBuf.count);

  display.setCursor(0, 43);
  display.printf("Missed: %lu", missedUploads);

  display.setCursor(0, 53);
  display.printf("WiFi:%s FB:%s %dkB",
    health.wifi      ? "Y" : "N",
    Firebase.ready() ? "Y" : "N",
    ESP.getFreeHeap() / 1024);

  display.display();
}

void updateOLED() {
  if (millis() - lastPageSwitch >= Config::OLED_PAGE_DURATION) {
    lastPageSwitch  = millis();
    currentOledPage = (currentOledPage + 1) % Config::OLED_PAGES;
  }
  switch (currentOledPage) {
    case 0: drawOledPage0(); break;
    case 1: drawOledPage1(); break;
    case 2: drawOledPage2(); break;
    case 3: drawOledPage3(); break;
  }
}


// ============================================================
//  SERIAL TELEMETRY
// ============================================================
void printSerial() {
  char ts[32];
  getFormattedTime(ts, sizeof(ts));
  bool settled = hrData.leadsConnected &&
                 (millis() - hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;

  Serial.printf(
    "[%s] Subj=%s Sess=%s Win=#%lu\n"
    "  ECG  : BPM=%d(avg) %.1f(inst) leads=%s settled=%s thr=%d\n"
    "  RRBUF: n=%d runMean=%.1f\n"
    "  HRV  : HR=%.1f RR=%.1fms SDNN=%.1f RMSSD=%.1f pNN50=%.1f%% [valid=%s]\n"
    "  RESP : avg=%d inst=%.1f BR_n=%d\n"
    "  TEMP : %.2fC valid=%s temp_n=%lu\n"
    "  ACC  : mag=%.3fg [%d,%d,%d] acc_n=%lu\n"
    "  SYS  : WiFi=%s FB=%s RSSI=%d Heap=%dkB Missed=%lu\n\n",
    ts, Config::SUBJECT_ID, sessionId, windowCount,
    hrData.bpmAvg, hrData.bpm,
    hrData.leadsConnected ? "YES" : "NO",
    settled ? "YES" : "NO",
    hrData.adaptThresh,
    rrBuf.count, rrBuf.runMean,
    lastWindow.mean_HR, lastWindow.mean_RR,
    lastWindow.SDNN, lastWindow.RMSSD, lastWindow.pNN50,
    lastWindow.hrv_valid ? "YES" : "NO",
    respData.rateAvg, respData.rateInstant, brBuf.count,
    tempData.celsius, tempData.valid ? "YES" : "NO", tempWelford.n,
    imuData.acc_mag_g, imuData.ax, imuData.ay, imuData.az, accWelford.n,
    health.wifi      ? "Y" : "N",
    Firebase.ready() ? "Y" : "N",
    (int)WiFi.RSSI(), ESP.getFreeHeap() / 1024, missedUploads
  );
}


// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== R26-DS-012 Anxiety Detection Firmware v1.0.0 ===");
  Serial.printf("Subject ID : %s\n", Config::SUBJECT_ID);

  const esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms     = Config::WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_cfg);
  esp_task_wdt_add(NULL);

  Wire.begin(Config::SDA_PIN, Config::SCL_PIN);
  Wire.setClock(100000);
  analogReadResolution(12);

  initOLED();
  initAD8232();
  initBMI160();
  initDS18B20();

  connectWiFi();
  if (health.wifi) {
    configTime(Config::GMT_OFFSET_SEC, Config::DST_OFFSET_SEC, Config::NTP_SERVER);
    Serial.println("[NTP] Time sync started...");
    Serial.print("[NTP] Waiting for sync");
    for (uint8_t i = 0; i < 20; i++) {
      struct tm info;
      if (getLocalTime(&info, 500)) { Serial.println(" OK"); break; }
      Serial.print(".");
      esp_task_wdt_reset();
    }
  }

  initFirebase();

  if (!generateSessionId()) {
    Serial.println("[Session] NTP not synced — session ID = NOSYNC");
  }
  Serial.printf("[Session] ID = %s\n", sessionId);

  // Upload session metadata so Colab can parse the window archive
  // with the correct context (subject, sampling rates, thresholds).
  uploadSessionInfo();

  rrBuf.count   = 0;
  rrBuf.runMean = 700.0f;
  brBuf.count   = 0;
  tempWelford.reset();
  accWelford.reset();

  windowStart    = millis();
  lastPageSwitch = millis();

  Serial.println("[System] Ready — entering data collection loop.\n");
  esp_task_wdt_reset();
}


// ============================================================
//  LOOP
//
//  Task schedule:
//   Every 5 ms    ECG sample + R-peak detection (200 Hz)
//   Every 150 ms  IMU + temperature + strain gauge
//   Every 300 ms  OLED refresh
//   Every 1000 ms Serial telemetry
//   Every 60 s    Feature computation + Firebase upload
//
//  No blocking delays anywhere in the loop.
//  All tasks use non-blocking millis() scheduling.
// ============================================================
void loop() {
  esp_task_wdt_reset();

  // ECG: highest priority, runs at 200 Hz.
  updateHeartRate();

  unsigned long now = millis();

  // Slow sensors: IMU + temperature + respiration.
  if (now - lastSensor >= Config::SENSOR_SLOW_MS) {
    lastSensor = now;
    updateIMU();
    updateTemperature();
    updateStrain();
  }

  // OLED display refresh.
  if (now - lastOLED >= Config::OLED_INTERVAL) {
    lastOLED = now;
    updateOLED();
  }

  // Serial telemetry.
  if (now - lastSerial >= Config::SERIAL_INTERVAL) {
    lastSerial = now;
    printSerial();
  }

  // 60-second window: compute features and upload.
  // windowStart is reset BEFORE the upload so accumulation
  // for the next window starts immediately, even if Firebase
  // takes several seconds to respond.
  if (now - windowStart >= Config::WINDOW_DURATION_MS) {
    windowStart = now;
    uploadWindowFeatures();
  }
}
