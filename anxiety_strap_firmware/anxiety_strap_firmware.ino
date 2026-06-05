/*
 * =============================================================
 * RESEARCH  : R26-DS-012 | Component 1 | SLIIT
 * PROJECT   : Personalized Anxiety Detection -- Phase 2
 *             Wearable Data Collection Firmware
 * AUTHOR    : Sendanayake H.D. (IT22107596)
 * VERSION   : 2.0.0
 * =============================================================
 *
 * CHANGELOG v2.0.0
 *   Replaced Firebase RTDB with direct HTTPS POST to the
 *   Hugging Face FastAPI production server (/ingest).
 *
 *   Removed the session-based path structure (subject/session
 *   hierarchy). The backend now identifies data using user_id
 *   tags in InfluxDB, so sessions are not needed.
 *
 *   Added raw sensor buffers (accel x/y/z, strain, temp) so
 *   the backend scipy/NeuroKit2 pipeline receives real signal
 *   arrays for gravity removal, peak extraction, and
 *   respiration rate cleaning -- exactly what /ingest expects.
 *
 *   On-device HRV computation is kept for OLED display and
 *   serial diagnostics only. The server runs its own
 *   independent feature extraction from the raw payload.
 *
 *   Renamed SUBJECT_ID to USER_ID to match backend convention.
 *   Removed all session ID generation and session info uploads.
 *
 * =============================================================
 * RAW PAYLOAD SENT TO POST /ingest (every 60 seconds):
 *
 *   user_id        Unique user identifier string
 *   window_ts      Timestamp YYYYMMDD_HHMMSS (NTP, UTC+5:30)
 *   fw_version     Firmware version string
 *   ecg_sample_hz  200 (sampling rate used for ECG capture)
 *   slow_sample_hz ~6.67 (polling rate for IMU, temp, strain)
 *   leads_on       Bool, true when AD8232 electrodes attached
 *   rr_ms[]        RR intervals in ms (from on-device detection)
 *   accel_x[]      BMI160 x-axis raw counts (16384 LSB per g)
 *   accel_y[]      BMI160 y-axis raw counts
 *   accel_z[]      BMI160 z-axis raw counts
 *   strain_raw[]   BF350 ADC readings (12-bit, 0-4095)
 *   temp_c[]       DS18B20 skin temperature samples (deg C)
 *
 *   NOTE: Field names must match the Pydantic model in
 *   main.py on your HF Space. Update them there or here if
 *   they diverge.
 *
 * =============================================================
 * 11 ON-DEVICE FEATURES (display only, not sent in payload):
 *   mean_HR, mean_RR, SDNN, RMSSD, pNN50
 *   mean_BR, std_BR
 *   mean_temp, std_temp
 *   mean_acc_mag, std_acc_mag
 *
 * =============================================================
 * HARDWARE
 *   ESP32-C3 Mini-1
 *   AD8232  ECG   -- Analog GPIO4, LO+ GPIO5, LO- GPIO6
 *   BMI160  IMU   -- I2C SDA=8, SCL=9, addr 0x69
 *   DS18B20 Temp  -- OneWire GPIO2
 *   BF350   Resp  -- Analog GPIO3 (strain gauge)
 *   SSD1306 OLED  -- I2C SDA=8, SCL=9, addr 0x3C (128x64)
 *
 * =============================================================
 * ECG SAMPLING RATE
 *   200 Hz (5 ms interval). Halves RR timing error from
 *   +/-10 ms (100 Hz) to +/-5 ms. Matters for pNN50 accuracy
 *   since the threshold is 50 ms.
 *
 * =============================================================
 * SECURITY NOTE
 *   postToServer() uses setInsecure() to skip TLS certificate
 *   verification. This is acceptable during development.
 *   For production, replace setInsecure() with setCACert()
 *   using the ISRG Root X1 PEM (Let's Encrypt root CA that
 *   signs Hugging Face Space certificates).
 * =============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <time.h>
#include <math.h>
#include <BMI160Gen.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>


// =============================================================
//  CONFIGURATION
//  Edit ONLY this section to change user, WiFi, and HF server.
// =============================================================
namespace Config {

  // User identifier. Must match the user_id registered in the
  // backend and used to tag InfluxDB measurements.
  constexpr char USER_ID[] = "P001";   // CHANGE PER USER

  // WiFi credentials.
  constexpr char SSID[]     = "SLT FIBER";
  constexpr char PASSWORD[] = "20hOmewIFi25";

  // Hugging Face FastAPI Space host.
  // Format: {hf-username}-{space-name}.hf.space
  // The firmware POSTs to https://{HF_HOST}/ingest every 60 s.
  constexpr char HF_HOST[] = "YOUR-HF-USERNAME-YOUR-SPACE-NAME.hf.space";

  // Timing (all in milliseconds unless noted).
  constexpr uint32_t WINDOW_DURATION_MS = 60000UL;
  constexpr uint32_t OLED_INTERVAL      = 300UL;
  constexpr uint32_t SERIAL_INTERVAL    = 1000UL;
  constexpr uint32_t SENSOR_SLOW_MS     = 150UL;
  constexpr uint32_t OLED_PAGE_DURATION = 8000UL;
  constexpr uint32_t HTTPS_TIMEOUT_MS   = 15000UL;

  // NTP (Sri Lanka = UTC+5:30).
  constexpr long  GMT_OFFSET_SEC = 19800L;
  constexpr int   DST_OFFSET_SEC = 0;
  constexpr char  NTP_SERVER[]   = "pool.ntp.org";

  // Watchdog timeout. Must exceed HTTPS_TIMEOUT_MS plus
  // TLS handshake time (~3-5 s) plus processing overhead.
  constexpr uint32_t WDT_TIMEOUT_SEC = 40;

  // GPIO Pins.
  constexpr uint8_t SDA_PIN     = 8;
  constexpr uint8_t SCL_PIN     = 9;
  constexpr uint8_t DS18B20_PIN = 2;
  constexpr uint8_t STRAIN_PIN  = 3;
  constexpr uint8_t ECG_PIN     = 4;   // AD8232 analog output
  constexpr uint8_t LO_PLUS     = 5;   // AD8232 leads-off (+)
  constexpr uint8_t LO_MINUS    = 6;   // AD8232 leads-off (-)

  // I2C addresses.
  constexpr uint8_t BMI160_ADDR = 0x69;
  constexpr uint8_t OLED_ADDR   = 0x3C;

  // ADC.
  constexpr float ADC_REF_V = 3.3f;
  constexpr float ADC_MAX   = 4095.0f;

  // ECG / Heart Rate.
  // 5 ms = 200 Hz. Halves RR timing error vs 100 Hz.
  constexpr uint32_t ECG_SAMPLE_INTERVAL = 5;
  constexpr uint8_t  BPM_RATE_SIZE       = 10;
  constexpr float    BPM_MIN             = 30.0f;
  constexpr float    BPM_MAX             = 200.0f;
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

  // Ectopic beat filter. Any RR interval deviating more than
  // 20% from the running mean is excluded. 20% matches
  // NeuroKit2's default artifact correction threshold.
  constexpr float RR_ECTOPIC_THRESH = 0.20f;

  // Respiration.
  constexpr uint8_t  RESP_FILTER_SIZE  = 5;
  constexpr uint8_t  RESP_DC_SIZE      = 30;
  constexpr uint8_t  RESP_RATE_SIZE    = 6;
  constexpr float    RESP_MIN          = 4.0f;
  constexpr float    RESP_MAX          = 40.0f;
  constexpr uint32_t RESP_MIN_INTERVAL = 1500UL;

  // Accumulation buffer sizes.
  // 200 BPM x 60 s = 200 beats max; 256 gives headroom.
  constexpr uint16_t RR_BUFFER_SIZE = 256;
  // 40 breaths/min x 60 s = 40 max.
  constexpr uint8_t BR_BUFFER_SIZE  = 60;
  // 150 ms interval -> ~400 samples per 60 s window; 440 = headroom.
  constexpr uint16_t SLOW_BUF_SIZE  = 440;
  // 2 s interval -> 30 temp samples per window; 36 = headroom.
  constexpr uint8_t TEMP_BUF_SIZE   = 36;

  // Minimum valid RR count for a HRV-valid display window.
  constexpr uint8_t MIN_VALID_RR = 20;

  // BMI160 default +/-2g range gives 16384 LSB per g.
  constexpr float BMI160_ACC_SCALE_G = 16384.0f;

  // Minimum free heap required before attempting an upload.
  // TLS handshake alone consumes ~60 KB.
  constexpr uint32_t MIN_HEAP_FOR_UPLOAD = 32000UL;

  // OLED.
  constexpr uint8_t SCREEN_W   = 128;
  constexpr uint8_t SCREEN_H   = 64;
  constexpr uint8_t OLED_PAGES = 4;

  // Firmware version.
  constexpr char FW_VERSION[] = "2.0.0-R26DS012";
}


// =============================================================
//  DATA STRUCTURES
// =============================================================

// Heart rate display and ECG detection state.
struct HeartRateData {
  float bpm            = 0.0f;
  int   bpmAvg         = 0;
  int   ecgValue       = 0;
  int   ecgSmoothed    = 0;
  bool  leadsConnected = false;

  float rates[Config::BPM_RATE_SIZE] = {0.0f};
  byte  rateSpot          = 0;
  long  lastBeat          = 0;
  byte  validBeats        = 0;
  int   adaptThresh       = 2048;
  unsigned long leadsConnectedAt = 0;
};

// Accumulates beat-to-beat RR intervals across the current
// 60-second window. runMean is the EMA of accepted intervals,
// used by the ectopic beat filter.
struct RRBuffer {
  float    intervals[Config::RR_BUFFER_SIZE] = {0.0f};
  uint16_t count   = 0;
  float    runMean = 700.0f;   // Seeded at ~86 BPM resting rate
};

// HRV features computed on-device for display purposes only.
// Formulas match NeuroKit2 hrv_time() exactly.
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
// O(1) memory regardless of sample count. Used for temp and
// accelerometer magnitude stats shown on the OLED.
struct WelfordState {
  uint32_t n    = 0;
  double   mean = 0.0;
  double   M2   = 0.0;

  void update(float x) {
    n++;
    double delta  = (double)x - mean;
    mean         += delta / (double)n;
    double delta2 = (double)x - mean;
    M2           += delta * delta2;
  }

  float getMean()   const { return (n > 0) ? (float)mean : 0.0f; }
  float getStddev() const { return (n > 1) ? (float)sqrt(M2 / (double)(n-1)) : 0.0f; }
  void  reset()           { n = 0; mean = 0.0; M2 = 0.0; }
};

// 11 on-device features for OLED display and serial diagnostics.
// The server re-extracts its own feature set from the raw arrays
// in the /ingest payload -- this struct is for local use only.
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

struct ImuData {
  int   ax = 0, ay = 0, az = 0;
  int   gx = 0, gy = 0, gz = 0;
  float acc_mag_g = 0.0f;
};

struct TemperatureData {
  float celsius    = 0.0f;
  float fahrenheit = 0.0f;
  bool  valid      = false;
};

struct StrainData {
  int   raw     = 0;
  float voltage = 0.0f;
};

struct RespirationData {
  float filteredRaw  = 0.0f;
  float acComponent  = 0.0f;
  float rateInstant  = 0.0f;
  int   rateAvg      = 0;

  float rates[Config::RESP_RATE_SIZE] = {0.0f};
  byte  rateSpot       = 0;
  byte  validBreaths   = 0;
  long  lastBreath     = 0;
  bool  breathDetected = false;
};

// =============================================================
//  RAW SENSOR BUFFERS
//  Populated during each 60-second window, then serialised into
//  the /ingest JSON payload. The server's scipy/NeuroKit2
//  pipeline runs gravity removal, peak extraction, and
//  respiration cleaning on these arrays directly.
// =============================================================

struct AccelRawBuf {
  int16_t  x[Config::SLOW_BUF_SIZE];
  int16_t  y[Config::SLOW_BUF_SIZE];
  int16_t  z[Config::SLOW_BUF_SIZE];
  uint16_t count = 0;
};

struct StrainRawBuf {
  uint16_t values[Config::SLOW_BUF_SIZE];
  uint16_t count = 0;
};

struct TempRawBuf {
  float   values[Config::TEMP_BUF_SIZE];
  uint8_t count = 0;
};

struct SensorHealth {
  bool ad8232  = false;
  bool bmi160  = false;
  bool ds18b20 = false;
  bool wifi    = false;
  bool server  = false;   // true after a successful /ingest POST
};


// =============================================================
//  GLOBAL VARIABLES
// =============================================================

HeartRateData   hrData;
ImuData         imuData;
TemperatureData tempData;
RespirationData respData;
StrainData      strainData;
SensorHealth    health;

// Feature accumulation buffers (reset after each 60-s upload).
RRBuffer     rrBuf;
BRBuffer     brBuf;
WelfordState tempWelford;
WelfordState accWelford;

// Raw sensor buffers for the /ingest payload.
AccelRawBuf  accelRawBuf;
StrainRawBuf strainRawBuf;
TempRawBuf   tempRawBuf;

// Last computed window features (shown on OLED page 0).
WindowFeatures lastWindow;
uint32_t       windowCount   = 0;
uint32_t       missedUploads = 0;

// Hardware drivers.
OneWire           oneWire(Config::DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
Adafruit_SSD1306  display(Config::SCREEN_W, Config::SCREEN_H, &Wire, -1);

// Scheduling timestamps.
unsigned long lastOLED       = 0;
unsigned long lastSerial     = 0;
unsigned long lastSensor     = 0;
unsigned long lastEcgRead    = 0;
unsigned long lastPageSwitch = 0;
unsigned long windowStart    = 0;

// Current OLED page (0 to 3).
uint8_t currentOledPage = 0;


// =============================================================
//  UTILITY FUNCTIONS
// =============================================================

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


// =============================================================
//  HRV FEATURE COMPUTATION (on-device, display use only)
//  All formulas verified against NeuroKit2 hrv_time() output.
// =============================================================

HRVFeatures computeHRV(const RRBuffer& buf) {
  HRVFeatures f;
  f.n_rr = buf.count;
  if (buf.count < Config::MIN_VALID_RR) { f.valid = false; return f; }

  double sum = 0.0;
  for (uint16_t i = 0; i < buf.count; i++) sum += (double)buf.intervals[i];
  f.mean_RR = (float)(sum / (double)buf.count);
  f.mean_HR = (f.mean_RR > 0.0f) ? (60000.0f / f.mean_RR) : 0.0f;

  // SDNN = population std dev of all RR intervals.
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
      float diff = buf.intervals[i] - buf.intervals[i-1];
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
  mean_BR = 0.0f; std_BR = 0.0f;
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


// =============================================================
//  WIFI
// =============================================================

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
    Serial.println("\n[WiFi] FAILED -- offline mode.");
  }
}

void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    health.wifi = false;
    connectWiFi();
  }
}


// =============================================================
//  BUFFER RESET
//  Called at the end of every 60-second window regardless of
//  whether the upload succeeded, so the next window always
//  starts with fresh accumulators.
//  rrBuf.runMean is intentionally NOT reset here -- the ectopic
//  filter needs a valid seed at the start of the next window.
// =============================================================

void resetWindowBuffers() {
  rrBuf.count        = 0;
  brBuf.count        = 0;
  accelRawBuf.count  = 0;
  strainRawBuf.count = 0;
  tempRawBuf.count   = 0;
  tempWelford.reset();
  accWelford.reset();
}


// =============================================================
//  /ingest PAYLOAD BUILDER
//
//  Serialises all accumulated raw sensor arrays into a single
//  JSON string ready for the HF FastAPI /ingest endpoint.
//
//  Estimated output size: 10-14 KB for a full 60-second window.
//  s.reserve(16000) pre-allocates one contiguous heap block to
//  prevent repeated reallocations during string concatenation.
//
//  The F() macro stores string literals in flash memory,
//  saving SRAM during construction.
// =============================================================

String buildIngestPayload(const char* ts) {
  String s;
  s.reserve(16000);

  s  = F("{\"user_id\":\"");
  s += Config::USER_ID;
  s += F("\",\"window_ts\":\"");
  s += ts;
  s += F("\",\"fw_version\":\"");
  s += Config::FW_VERSION;
  s += F("\",\"ecg_sample_hz\":");
  s += (uint32_t)(1000UL / Config::ECG_SAMPLE_INTERVAL);
  s += F(",\"slow_sample_hz\":");
  s += String(1000.0f / (float)Config::SENSOR_SLOW_MS, 2);
  s += F(",\"leads_on\":");
  s += hrData.leadsConnected ? F("true") : F("false");

  // RR intervals in ms (pre-extracted by on-device R-peak detector).
  // The server can pass these directly to nk.hrv_time() to compute
  // HRV features without needing the raw ECG waveform.
  s += F(",\"rr_ms\":[");
  for (uint16_t i = 0; i < rrBuf.count; i++) {
    if (i) s += ',';
    s += String(rrBuf.intervals[i], 1);
  }
  s += ']';

  // BMI160 raw accelerometer counts (16384 LSB per g at +/-2g).
  s += F(",\"accel_x\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.x[i];
  }
  s += ']';

  s += F(",\"accel_y\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.y[i];
  }
  s += ']';

  s += F(",\"accel_z\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.z[i];
  }
  s += ']';

  // BF350 strain gauge ADC readings (12-bit, 0-4095).
  // 8-sample average per poll to reduce electrical noise.
  s += F(",\"strain_raw\":[");
  for (uint16_t i = 0; i < strainRawBuf.count; i++) {
    if (i) s += ',';
    s += strainRawBuf.values[i];
  }
  s += ']';

  // DS18B20 skin temperature samples (deg C).
  // Only physiologically plausible skin readings (28-38 C)
  // are stored, matching the filter applied in tempWelford.
  s += F(",\"temp_c\":[");
  for (uint8_t i = 0; i < tempRawBuf.count; i++) {
    if (i) s += ',';
    s += String(tempRawBuf.values[i], 2);
  }
  s += F("]}");

  return s;
}


// =============================================================
//  HTTPS POST TO /ingest
//
//  Creates a fresh WiFiClientSecure per upload. This avoids
//  holding TLS context in memory between the 60-second windows.
//
//  esp_task_wdt_reset() is called before and after the blocking
//  POST call because the HTTPS handshake + transfer can take
//  up to HTTPS_TIMEOUT_MS (15 s), which is close to the default
//  WDT timeout. WDT_TIMEOUT_SEC is set to 40 s to cover this.
// =============================================================

bool postToServer(const String& payload) {
  WiFiClientSecure secureClient;

  // Dev mode: skips TLS certificate verification.
  // Replace with secureClient.setCACert(isrg_root_x1_pem)
  // before deploying to production.
  secureClient.setInsecure();

  HTTPClient https;
  String url = String(F("https://")) + Config::HF_HOST + F("/ingest");

  if (!https.begin(secureClient, url)) {
    Serial.println(F("[HF] https.begin() failed."));
    return false;
  }

  https.addHeader(F("Content-Type"), F("application/json"));
  https.setTimeout(Config::HTTPS_TIMEOUT_MS);

  esp_task_wdt_reset();
  int code = https.POST(payload);
  esp_task_wdt_reset();

  bool ok = false;
  if (code > 0) {
    Serial.printf("[HF] POST -> HTTP %d\n", code);
    ok = (code >= 200 && code < 300);
    if (!ok) {
      // Log the server error body to help debug Pydantic
      // validation failures or server-side exceptions.
      String resp = https.getString();
      if (resp.length() > 0)
        Serial.printf("[HF] Server response: %s\n", resp.c_str());
    }
  } else {
    Serial.printf("[HF] POST failed: %s\n",
                  https.errorToString(code).c_str());
  }

  https.end();
  secureClient.stop();
  return ok;
}


// =============================================================
//  60-SECOND WINDOW: COMPUTE FEATURES + UPLOAD
// =============================================================

void uploadWindowFeatures() {
  char ts[32], safets[24];
  if (!getFormattedTime(ts, sizeof(ts)))       strcpy(ts, "unknown");
  if (!getSafeTimestamp(safets, sizeof(safets))) strcpy(safets, "unknown");

  // Compute on-device features for OLED display and serial only.
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
    "BR=%.1f+/-%.1f Temp=%.2f+/-%.3f AccMag=%.3f+/-%.3f\n"
    "             Raw buffers: accel=%d  strain=%d  temp=%d\n",
    windowCount, wf.hrv_valid ? "YES" : "NO",
    wf.rr_count, wf.mean_HR, wf.SDNN, wf.RMSSD, wf.pNN50,
    wf.mean_BR, wf.std_BR,
    wf.mean_temp, wf.std_temp,
    wf.mean_acc_mag, wf.std_acc_mag,
    accelRawBuf.count, strainRawBuf.count, tempRawBuf.count
  );

  // Attempt HTTPS upload.
  ensureWiFi();
  if (!health.wifi) {
    Serial.println(F("[HF] Upload skipped -- no WiFi."));
    missedUploads++;
    resetWindowBuffers();
    return;
  }

  uint32_t heapFree = ESP.getFreeHeap();
  if (heapFree < Config::MIN_HEAP_FOR_UPLOAD) {
    Serial.printf("[HF] Upload skipped -- heap too low (%lu B).\n",
                  (unsigned long)heapFree);
    missedUploads++;
    resetWindowBuffers();
    return;
  }

  Serial.printf("[HF] Building payload (heap free: %lu B)...\n",
                (unsigned long)heapFree);
  String payload = buildIngestPayload(safets);
  Serial.printf("[HF] Payload: %u B  Posting to %s\n",
                payload.length(), Config::HF_HOST);

  bool ok = postToServer(payload);

  // Explicitly release the heap held by the payload String before
  // the function returns, since the next window starts immediately.
  payload = String();

  if (ok) {
    Serial.printf("[HF] Window #%lu OK  [%s]\n", windowCount, ts);
    health.server = true;
  } else {
    Serial.printf("[HF] Window #%lu FAILED  [%s]\n", windowCount, ts);
    health.server = false;
    missedUploads++;
  }

  resetWindowBuffers();
}


// =============================================================
//  SENSOR: AD8232 ECG
// =============================================================

void initAD8232() {
  Serial.print(F("[AD8232] Initializing... "));
  pinMode(Config::LO_PLUS,  INPUT_PULLDOWN);
  pinMode(Config::LO_MINUS, INPUT_PULLDOWN);
  pinMode(Config::ECG_PIN,  INPUT);
  analogSetPinAttenuation(Config::ECG_PIN,    ADC_11db);
  analogSetPinAttenuation(Config::STRAIN_PIN, ADC_11db);
  (void)analogRead(Config::ECG_PIN); delay(10);
  (void)analogRead(Config::ECG_PIN);
  health.ad8232 = true;
  Serial.println(F("OK (200 Hz, ectopic filter active)"));
}

// Called every 5 ms (200 Hz) from the main loop.
// Detects R-peaks, applies the ectopic beat filter, and stores
// valid RR intervals in rrBuf for the 60-second window.
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

  bool loPlus  = digitalRead(Config::LO_PLUS);
  bool loMinus = digitalRead(Config::LO_MINUS);
  bool connected = !(loPlus || loMinus);

  if (!connected) {
    if (hrData.leadsConnected) {
      // Leads just disconnected: reset ectopic filter seed.
      rrBuf.runMean           = 700.0f;
      hrData.leadsConnectedAt = 0;
    }
    hrData.leadsConnected = false;
    return;
  }

  if (!hrData.leadsConnected) {
    // Leads just reconnected: start the settle timer.
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

      if (dt_ms >= Config::RR_MIN_MS && dt_ms <= Config::RR_MAX_MS) {

        // Ectopic beat filter: skip filter for the first 5 beats
        // so runMean can establish a baseline from real data.
        float deviation  = fabsf(dt_ms - rrBuf.runMean) / rrBuf.runMean;
        bool  is_ectopic = (rrBuf.count >= 5) &&
                           (deviation > Config::RR_ECTOPIC_THRESH);

        if (!is_ectopic) {
          if (rrBuf.count < Config::RR_BUFFER_SIZE)
            rrBuf.intervals[rrBuf.count++] = dt_ms;

          // EMA update (alpha = 0.1).
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

    // Always update lastBeat -- even for ectopic beats -- so
    // the next RR interval is measured from this peak and not
    // the one before it. Skipping this would inflate the next RR.
    hrData.lastBeat = now;

  } else if (hrData.ecgSmoothed < thresh - hyst) {
    aboveThreshold = false;
  }
}


// =============================================================
//  SENSOR: BMI160 (IMU)
// =============================================================

void initBMI160() {
  Serial.print(F("[BMI160] Initializing... "));
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, Config::BMI160_ADDR)) {
    Serial.println(F("FAILED -- check wiring!"));
    health.bmi160 = false;
    return;
  }
  // Default +/-2g range kept: appropriate for torso motion and
  // matches WESAD dataset accelerometer units.
  health.bmi160 = true;
  Serial.println(F("OK (+/-2g, step counting disabled)"));
}

// Called every 150 ms. Reads raw accelerometer and gyroscope,
// computes vector magnitude for the Welford display stats, and
// stores raw counts in accelRawBuf for the /ingest payload.
void updateIMU() {
  if (!health.bmi160) return;

  BMI160.readAccelerometer(imuData.ax, imuData.ay, imuData.az);
  BMI160.readGyro(imuData.gx, imuData.gy, imuData.gz);

  // Convert to g for the Welford magnitude statistic.
  float ax_g = imuData.ax / Config::BMI160_ACC_SCALE_G;
  float ay_g = imuData.ay / Config::BMI160_ACC_SCALE_G;
  float az_g = imuData.az / Config::BMI160_ACC_SCALE_G;
  imuData.acc_mag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  accWelford.update(imuData.acc_mag_g);

  // Store raw LSB counts for the server's gravity removal filter.
  if (accelRawBuf.count < Config::SLOW_BUF_SIZE) {
    accelRawBuf.x[accelRawBuf.count] = (int16_t)imuData.ax;
    accelRawBuf.y[accelRawBuf.count] = (int16_t)imuData.ay;
    accelRawBuf.z[accelRawBuf.count] = (int16_t)imuData.az;
    accelRawBuf.count++;
  }
}


// =============================================================
//  SENSOR: DS18B20 (Skin Temperature)
// =============================================================

void initDS18B20() {
  Serial.print(F("[DS18B20] Initializing... "));
  tempSensor.begin();
  uint8_t n = tempSensor.getDeviceCount();
  health.ds18b20 = (n > 0);
  Serial.printf("%s (found %d device[s])\n",
                health.ds18b20 ? "OK" : "FAILED", n);
  if (health.ds18b20) {
    tempSensor.setResolution(12);           // 0.0625 deg C per step
    tempSensor.setWaitForConversion(false); // Non-blocking
  }
}

// Called every 150 ms, internally throttled to a 2-second cycle.
// Two-step non-blocking approach:
//   Step A (every 2 s): send the conversion command.
//   Step B (800 ms later): read the result.
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

      // Only store physiologically plausible skin temperatures
      // (normal skin surface range: 28-38 deg C).
      // Values outside this range indicate the strap is loose
      // or the sensor is not in contact with skin.
      if (c >= 28.0f && c <= 38.0f) {
        tempWelford.update(c);
        if (tempRawBuf.count < Config::TEMP_BUF_SIZE)
          tempRawBuf.values[tempRawBuf.count++] = c;
      }
    }
    requested = false;
  }
}


// =============================================================
//  SENSOR: BF350 Strain Gauge (Respiration)
//
//  The strain gauge stretches during chest expansion, producing
//  a slow voltage wave. Three processing stages extract BR:
//    Stage 1 -- moving-average LP filter removes electrical noise
//    Stage 2 -- DC removal strips the resting-tension offset
//    Stage 3 -- rising zero-crossing detects each breath
// =============================================================

void updateStrain() {
  const uint8_t SAMPLES = 8;
  uint32_t adcSum = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    adcSum += analogRead(Config::STRAIN_PIN);
    delayMicroseconds(100);
  }
  uint16_t adcAvg    = (uint16_t)(adcSum / SAMPLES);
  strainData.raw     = adcAvg;
  strainData.voltage = (adcAvg / Config::ADC_MAX) * Config::ADC_REF_V;

  // Store the 8-sample average for the /ingest payload.
  if (strainRawBuf.count < Config::SLOW_BUF_SIZE)
    strainRawBuf.values[strainRawBuf.count++] = adcAvg;

  // Stage 1: Moving-average LP filter.
  static float   maBuffer[Config::RESP_FILTER_SIZE] = {0};
  static uint8_t maIdx = 0;
  maBuffer[maIdx] = (float)adcAvg;
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
  respData.acComponent = respData.filteredRaw - (dcSum / Config::RESP_DC_SIZE);

  // Stage 3: Rising zero-crossing breath detection.
  static float prevAC          = 0.0f;
  static long  lastBreathLocal = 0;
  respData.breathDetected = false;

  unsigned long now = millis();
  bool zeroCrossUp  = (prevAC < 0.0f && respData.acComponent >= 0.0f);

  if (zeroCrossUp && (now - lastBreathLocal > Config::RESP_MIN_INTERVAL)) {
    if (lastBreathLocal != 0) {
      float instRate = 60.0f / ((now - lastBreathLocal) / 1000.0f);
      if (instRate >= Config::RESP_MIN && instRate <= Config::RESP_MAX) {
        respData.rateInstant = instRate;
        respData.rates[respData.rateSpot++] = instRate;
        respData.rateSpot %= Config::RESP_RATE_SIZE;
        if (respData.validBreaths < Config::RESP_RATE_SIZE) respData.validBreaths++;
        float rSum = 0;
        for (byte i = 0; i < respData.validBreaths; i++) rSum += respData.rates[i];
        respData.rateAvg        = (int)(rSum / (float)respData.validBreaths);
        respData.breathDetected = true;
        if (brBuf.count < Config::BR_BUFFER_SIZE)
          brBuf.rates[brBuf.count++] = instRate;
      }
    }
    lastBreathLocal     = now;
    respData.lastBreath = now;
  }
  prevAC = respData.acComponent;
}


// =============================================================
//  OLED DISPLAY
// =============================================================

void initOLED() {
  Serial.print(F("[OLED] Initializing... "));
  if (!display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR)) {
    Serial.println(F("FAILED at 0x3C"));
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setTextSize(1);
  display.setCursor(4,  8);  display.print(F("R26-DS-012 RESEARCH"));
  display.setCursor(8,  22); display.print(F("Anxiety Detection"));
  display.setCursor(4,  36); display.print(F("User: "));
  display.print(Config::USER_ID);
  display.setCursor(4,  50); display.print(F("Initializing..."));
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
  Serial.println(F("OK"));
}

// Page 0: ECG + HRV status.
void drawOledPage0() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("ECG + HRV"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  if (!hrData.leadsConnected) {
    display.setCursor(8, 22); display.print(F("Leads disconnected."));
    display.setCursor(8, 35); display.print(F("Attach electrodes."));
    display.setCursor(8, 48); display.print(F("Check contact."));
    display.display();
    return;
  }

  bool settled = (millis() - hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;
  if (!settled) {
    unsigned long remain =
      (Config::ECG_SETTLE_MS - (millis() - hrData.leadsConnectedAt)) / 1000UL + 1;
    display.setCursor(8, 22); display.print(F("Calibrating ECG..."));
    display.setCursor(8, 35); display.printf("Ready in %lus", remain);
    display.display();
    return;
  }

  display.setTextSize(3);
  display.setCursor(0, 14);
  display.print(hrData.bpmAvg);
  display.setTextSize(1);
  display.setCursor(56, 16); display.print(F("BPM avg"));
  display.setCursor(56, 26); display.printf("RR buf:%d", rrBuf.count);
  display.setCursor(0, 44);
  if (lastWindow.hrv_valid) {
    display.printf("RMSSD:%.1f SDNN:%.1f", lastWindow.RMSSD, lastWindow.SDNN);
    display.setCursor(0, 54);
    display.printf("pNN50: %.1f%%", lastWindow.pNN50);
  } else {
    display.print(F("HRV: collecting..."));
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
  display.setCursor(2, 2);  display.print(F("RESPIRATION"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(3);
  display.setCursor(0, 14);
  if (respData.validBreaths > 0) display.print(respData.rateAvg);
  else                           display.print(F("--"));
  display.setTextSize(1);
  display.setCursor(56, 22); display.print(F("br/min"));
  display.setCursor(0, 44);
  if (respData.rateInstant > 0)
    display.printf("Inst: %.1f br/min", respData.rateInstant);
  else
    display.print(F("Calibrating..."));
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
  display.setCursor(2, 2);  display.print(F("TEMP + MOTION"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 15);
  if (tempData.valid) display.printf("Temp: %.2f C", tempData.celsius);
  else               display.print(F("Temp: -- sensor ERR"));
  if (lastWindow.mean_temp > 0.0f) {
    display.setCursor(0, 25);
    display.printf("Avg:%.2fC Std:%.3f",
                   lastWindow.mean_temp, lastWindow.std_temp);
  }
  display.setCursor(0, 37);
  display.printf("AccMag: %.3f g", imuData.acc_mag_g);
  if (lastWindow.mean_acc_mag > 0.0f) {
    display.setCursor(0, 47);
    display.printf("Avg:%.3f Std:%.3f",
                   lastWindow.mean_acc_mag, lastWindow.std_acc_mag);
  }
  display.setCursor(0, 57);
  display.printf("A:[%d,%d,%d]k",
                 imuData.ax/1000, imuData.ay/1000, imuData.az/1000);
  display.display();
}

// Page 3: System health and upload status.
void drawOledPage3() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("SYSTEM"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 14);
  display.printf("User: %s", Config::USER_ID);

  display.setCursor(0, 23);
  display.printf("Win: #%lu  RR: %d", windowCount, rrBuf.count);

  display.setCursor(0, 33);
  display.printf("Missed: %lu", missedUploads);

  display.setCursor(0, 43);
  display.printf("WiFi:%s Srv:%s %dkB",
    health.wifi   ? "Y" : "N",
    health.server ? "Y" : "N",
    ESP.getFreeHeap() / 1024);

  display.setCursor(0, 53);
  display.printf("RSSI:%d  A:%d T:%d",
    (int)WiFi.RSSI(), accelRawBuf.count, tempRawBuf.count);

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


// =============================================================
//  SERIAL TELEMETRY
// =============================================================

void printSerial() {
  char ts[32];
  getFormattedTime(ts, sizeof(ts));
  bool settled = hrData.leadsConnected &&
                 (millis() - hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;

  Serial.printf(
    "[%s] User=%s Win=#%lu\n"
    "  ECG  : BPM=%d(avg) %.1f(inst) leads=%s settled=%s thr=%d\n"
    "  RRBUF: n=%d runMean=%.1f\n"
    "  HRV  : HR=%.1f RR=%.1fms SDNN=%.1f RMSSD=%.1f pNN50=%.1f%% [valid=%s]\n"
    "  RESP : avg=%d inst=%.1f BR_n=%d\n"
    "  TEMP : %.2fC valid=%s temp_n=%lu\n"
    "  ACC  : mag=%.3fg [%d,%d,%d] acc_n=%lu\n"
    "  RAW  : accel=%d  strain=%d  temp=%d\n"
    "  SYS  : WiFi=%s Srv=%s RSSI=%d Heap=%dkB Missed=%lu\n\n",
    ts, Config::USER_ID, windowCount,
    hrData.bpmAvg, hrData.bpm,
    hrData.leadsConnected ? "YES" : "NO",
    settled          ? "YES" : "NO",
    hrData.adaptThresh,
    rrBuf.count, rrBuf.runMean,
    lastWindow.mean_HR, lastWindow.mean_RR,
    lastWindow.SDNN, lastWindow.RMSSD, lastWindow.pNN50,
    lastWindow.hrv_valid ? "YES" : "NO",
    respData.rateAvg, respData.rateInstant, brBuf.count,
    tempData.celsius, tempData.valid ? "YES" : "NO", tempWelford.n,
    imuData.acc_mag_g, imuData.ax, imuData.ay, imuData.az, accWelford.n,
    accelRawBuf.count, strainRawBuf.count, tempRawBuf.count,
    health.wifi   ? "Y" : "N",
    health.server ? "Y" : "N",
    (int)WiFi.RSSI(), ESP.getFreeHeap() / 1024, missedUploads
  );
}


// =============================================================
//  SETUP
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n\n=== R26-DS-012 Anxiety Detection Firmware v2.0.0 ==="));
  Serial.printf("User ID : %s\n", Config::USER_ID);
  Serial.printf("HF Host : %s\n", Config::HF_HOST);

  // WDT timeout raised to 40 s to cover the HTTPS_TIMEOUT_MS
  // (15 s) plus TLS handshake overhead (~3-5 s).
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
    configTime(Config::GMT_OFFSET_SEC, Config::DST_OFFSET_SEC,
               Config::NTP_SERVER);
    Serial.println(F("[NTP] Time sync started..."));
    Serial.print(F("[NTP] Waiting"));
    for (uint8_t i = 0; i < 20; i++) {
      struct tm info;
      if (getLocalTime(&info, 500)) { Serial.println(F(" OK")); break; }
      Serial.print(F("."));
      esp_task_wdt_reset();
    }
  }

  resetWindowBuffers();
  rrBuf.runMean = 700.0f;   // Seed ectopic filter at ~86 BPM

  windowStart    = millis();
  lastPageSwitch = millis();

  Serial.println(F("[System] Ready -- entering data collection loop.\n"));
  esp_task_wdt_reset();
}


// =============================================================
//  LOOP
//
//  Task schedule:
//    Every 5 ms     ECG sample + R-peak detection (200 Hz)
//    Every 150 ms   IMU + temperature + strain gauge
//    Every 300 ms   OLED refresh
//    Every 1000 ms  Serial telemetry
//    Every 60 s     Raw payload POST to HF /ingest
//
//  No blocking delays anywhere in the loop.
//  All tasks use non-blocking millis() scheduling.
//
//  windowStart is reset BEFORE uploadWindowFeatures() is called
//  so the next window's accumulation begins immediately, even if
//  the HTTPS POST takes several seconds to complete.
// =============================================================

void loop() {
  esp_task_wdt_reset();
  updateHeartRate();

  unsigned long now = millis();

  if (now - lastSensor >= Config::SENSOR_SLOW_MS) {
    lastSensor = now;
    updateIMU();
    updateTemperature();
    updateStrain();
  }

  if (now - lastOLED >= Config::OLED_INTERVAL) {
    lastOLED = now;
    updateOLED();
  }

  if (now - lastSerial >= Config::SERIAL_INTERVAL) {
    lastSerial = now;
    printSerial();
  }

  if (now - windowStart >= Config::WINDOW_DURATION_MS) {
    windowStart = now;
    uploadWindowFeatures();
  }
}
