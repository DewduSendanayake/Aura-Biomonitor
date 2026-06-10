/*
 * =============================================================
 * RESEARCH  : R26-DS-012 | Component 1 | SLIIT
 * PROJECT   : Personalized Anxiety Detection -- Phase 2
 *             Wearable Data Collection Firmware
 * AUTHOR    : Sendanayake H.D. (IT22107596)
 * VERSION   : 2.1.0
 * =============================================================
 *
 * CHANGELOG v2.1.0
 *   Payload field names and structure now exactly match the
 *   RawSensorPayload Pydantic model in main.py on the HF Space:
 *     user_id, sampling_rate, ecg, resp, temp, acc_x, acc_y, acc_z
 *
 *   Added a 12,100-sample raw ECG buffer (200 Hz x 60 s) so
 *   the backend's nk.ecg_clean() + nk.ecg_peaks() pipeline
 *   receives the actual waveform. The firmware previously sent
 *   only pre-computed RR intervals, which the /ingest endpoint
 *   cannot process.
 *
 *   strain_raw renamed to resp (respiration via strain gauge).
 *   temp_c renamed to temp.
 *   accel_x/y/z renamed to acc_x/y/z.
 *   Removed: rr_ms, window_ts, fw_version, ecg_sample_hz,
 *             slow_sample_hz, leads_on (not in Pydantic model).
 *
 *   Replaced setInsecure() with setCACert() using the ISRG
 *   Root X1 CA cert (Let's Encrypt root that signs *.hf.space).
 *   A placeholder guard falls back to insecure mode and logs a
 *   warning if the cert has not been filled in yet.
 *
 * =============================================================
 * KNOWN MULTI-RATE MISMATCH (must fix in main.py eventually):
 *   main.py uses a single sampling_rate for ALL signals.
 *   ECG is sampled at 200 Hz; acc/resp/temp are at ~6.67 Hz.
 *   This firmware sends sampling_rate = 200 (the ECG rate).
 *   Consequence: HRV extraction is correct. The Butterworth
 *   high-pass for acc and nk.rsp_rate() for resp will receive
 *   the wrong fs value and produce inaccurate feature values
 *   for those channels. Fix: add slow_sample_hz to the Pydantic
 *   model in main.py and use it for acc/resp processing.
 *
 * =============================================================
 * PAYLOAD SENT TO POST /ingest (every 60 seconds):
 *   user_id        string  -- user identifier
 *   sampling_rate  int     -- ECG sampling rate (200 Hz)
 *   ecg            float[] -- raw ADC waveform at 200 Hz
 *   resp           float[] -- strain gauge ADC at ~6.67 Hz
 *   temp           float[] -- skin temperature (deg C) ~0.5 Hz
 *   acc_x          float[] -- BMI160 x-axis raw counts ~6.67 Hz
 *   acc_y          float[] -- BMI160 y-axis raw counts
 *   acc_z          float[] -- BMI160 z-axis raw counts
 *
 * =============================================================
 * SECURITY NOTE (GitHub):
 *   SSID and PASSWORD in Config are hardcoded. Do NOT push this
 *   file to a public repo with real credentials. Either move
 *   them to a gitignored secrets.h or use WiFi provisioning.
 *
 * =============================================================
 * HARDWARE
 *   ESP32-C3 Mini-1
 *   AD8232  ECG   -- Analog GPIO4, LO+ GPIO5, LO- GPIO6
 *   BMI160  IMU   -- I2C SDA=8, SCL=9, addr 0x69
 *   DS18B20 Temp  -- OneWire GPIO2
 *   BF350   Resp  -- Analog GPIO3 (strain gauge)
 *   SSD1306 OLED  -- I2C SDA=8, SCL=9, addr 0x3C (128x64)
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
//  ISRG ROOT X1 CA CERTIFICATE
//  Let's Encrypt signs *.hf.space certificates using this root.
//
//  HOW TO FILL THIS IN:
//    1. Go to https://letsencrypt.org/certificates/
//    2. Under "Root Certificates" click "ISRG Root X1 (PEM)"
//       or directly fetch: https://letsencrypt.org/certs/isrgrootx1.pem
//    3. Copy the entire block (-----BEGIN ... -----END-----) and
//       paste it between the R"EOF( and )EOF" delimiters below.
//
//  UNTIL you fill this in the firmware falls back to insecure
//  mode and logs a warning on Serial -- functional for dev,
//  NOT acceptable for production deployment.
//
//  Cert SHA-256: 96:BC:EC:06:26:49:76:F3:74:60:77:9A:CB:28:5A:24
//                52:A9:6E:25:75:3F:F7:C5:73:7E:F9:F2:2B:96:F2:07
//  Expiry: 4 June 2035
// =============================================================
static const char HF_CA_CERT[] = R"EOF(
REPLACE_ME -- paste the ISRG Root X1 PEM content here.
Fetch from: https://letsencrypt.org/certs/isrgrootx1.pem
)EOF";

// Helper: returns true if the cert placeholder has been filled.
inline bool certIsLoaded() {
  return (strncmp(HF_CA_CERT, "\nREPLACE_ME", 11) == 0) ? false : true;
}


// =============================================================
//  CONFIGURATION
//  Edit ONLY this section to change user, WiFi, and HF server.
//
//  WiFi credentials are hardcoded here. Do NOT commit this file
//  to a public GitHub repo. Move SSID/PASSWORD to a gitignored
//  secrets.h if you need version control for the rest.
// =============================================================
namespace Config {

  // User identifier -- must match the user_id tag in InfluxDB.
  constexpr char USER_ID[]  = "P001";   // CHANGE PER USER

  // WiFi -- see security note in file header.
  constexpr char SSID[]     = "SLT FIBER";
  constexpr char PASSWORD[] = "20hOmewIFi25";

  // Hugging Face FastAPI Space.
  // Format: {hf-username}-{space-name}.hf.space (lowercase, hyphens).
  // This resolves to https://dewdu-physiological-anxiety-escalation.hf.space/ingest
  constexpr char HF_HOST[] = "dewdu-physiological-anxiety-escalation.hf.space";

  // Timing (milliseconds unless noted).
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

  // Watchdog: must exceed HTTPS_TIMEOUT_MS + TLS handshake (~5 s).
  constexpr uint32_t WDT_TIMEOUT_SEC = 40;

  // GPIO Pins.
  constexpr uint8_t SDA_PIN     = 8;
  constexpr uint8_t SCL_PIN     = 9;
  constexpr uint8_t DS18B20_PIN = 2;
  constexpr uint8_t STRAIN_PIN  = 3;
  constexpr uint8_t ECG_PIN     = 4;
  constexpr uint8_t LO_PLUS     = 5;
  constexpr uint8_t LO_MINUS    = 6;

  // I2C addresses.
  constexpr uint8_t BMI160_ADDR = 0x69;
  constexpr uint8_t OLED_ADDR   = 0x3C;

  // ADC.
  constexpr float ADC_REF_V = 3.3f;
  constexpr float ADC_MAX   = 4095.0f;

  // ECG / Heart Rate.
  constexpr uint32_t ECG_SAMPLE_INTERVAL = 5;    // 5 ms = 200 Hz
  constexpr uint8_t  BPM_RATE_SIZE       = 10;
  constexpr float    BPM_MIN             = 30.0f;
  constexpr float    BPM_MAX             = 200.0f;
  constexpr uint8_t  ECG_FILTER_SIZE     = 8;
  constexpr float    ECG_THRESH_RATIO    = 0.40f;
  constexpr float    ECG_HYST_RATIO      = 0.15f;
  constexpr int      ECG_ADAPT_DECAY     = 1;
  constexpr uint32_t ECG_SETTLE_MS       = 5000;
  constexpr float    RR_MIN_MS           = 300.0f;
  constexpr float    RR_MAX_MS           = 2000.0f;
  constexpr float    RR_ECTOPIC_THRESH   = 0.20f;

  // Respiration.
  constexpr uint8_t  RESP_FILTER_SIZE  = 5;
  constexpr uint8_t  RESP_DC_SIZE      = 30;
  constexpr uint8_t  RESP_RATE_SIZE    = 6;
  constexpr float    RESP_MIN          = 4.0f;
  constexpr float    RESP_MAX          = 40.0f;
  constexpr uint32_t RESP_MIN_INTERVAL = 1500UL;

  // Raw ECG buffer: 200 Hz x 60 s + headroom.
  // main.py's /ingest runs nk.ecg_clean() + nk.ecg_peaks() on
  // this waveform -- it needs the full ADC sample array, not RR
  // intervals. At uint16_t this occupies 24.2 KB of SRAM.
  constexpr uint16_t ECG_RAW_BUF_SIZE = 12100;

  // Slow sensor buffer: 150 ms interval → ~400 samples per window.
  constexpr uint16_t SLOW_BUF_SIZE = 440;

  // Temperature buffer: 2 s interval → 30 samples per window.
  constexpr uint8_t TEMP_BUF_SIZE = 36;

  // Feature accumulation.
  constexpr uint16_t RR_BUFFER_SIZE = 256;
  constexpr uint8_t  BR_BUFFER_SIZE = 60;
  constexpr uint8_t  MIN_VALID_RR   = 20;

  // BMI160 default +/-2g: 16384 LSB per g.
  constexpr float BMI160_ACC_SCALE_G = 16384.0f;

  // Minimum free heap before attempting an upload.
  // Budget: TLS context ~60 KB + 75 KB payload String + headroom.
  constexpr uint32_t MIN_HEAP_FOR_UPLOAD = 160000UL;

  // OLED.
  constexpr uint8_t SCREEN_W   = 128;
  constexpr uint8_t SCREEN_H   = 64;
  constexpr uint8_t OLED_PAGES = 4;

  // Firmware version.
  constexpr char FW_VERSION[] = "2.1.0-R26DS012";
}


// =============================================================
//  DATA STRUCTURES
// =============================================================

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

struct RRBuffer {
  float    intervals[Config::RR_BUFFER_SIZE] = {0.0f};
  uint16_t count   = 0;
  float    runMean = 700.0f;
};

struct HRVFeatures {
  float    mean_HR = 0.0f;
  float    mean_RR = 0.0f;
  float    SDNN    = 0.0f;
  float    RMSSD   = 0.0f;
  float    pNN50   = 0.0f;
  bool     valid   = false;
  uint16_t n_rr    = 0;
};

struct BRBuffer {
  float   rates[Config::BR_BUFFER_SIZE] = {0.0f};
  uint8_t count = 0;
};

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

// On-device window features -- OLED display and serial only.
// main.py re-extracts its own feature set from the raw arrays.
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
//  Populated across the 60-second window, then flushed into the
//  JSON payload for POST /ingest.
//
//  Field name mapping (firmware → main.py Pydantic model):
//    ecgRawBuf    → ecg        (raw ADC waveform, 200 Hz)
//    accelRawBuf  → acc_x/y/z  (BMI160 raw counts, ~6.67 Hz)
//    strainRawBuf → resp        (strain gauge ADC, ~6.67 Hz)
//    tempRawBuf   → temp        (DS18B20 deg C, ~0.5 Hz)
// =============================================================

struct ECGRawBuf {
  uint16_t values[Config::ECG_RAW_BUF_SIZE];
  uint16_t count = 0;
};

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
  bool server  = false;
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

RRBuffer     rrBuf;
BRBuffer     brBuf;
WelfordState tempWelford;
WelfordState accWelford;

ECGRawBuf    ecgRawBuf;
AccelRawBuf  accelRawBuf;
StrainRawBuf strainRawBuf;
TempRawBuf   tempRawBuf;

WindowFeatures lastWindow;
uint32_t       windowCount   = 0;
uint32_t       missedUploads = 0;

OneWire           oneWire(Config::DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
Adafruit_SSD1306  display(Config::SCREEN_W, Config::SCREEN_H, &Wire, -1);

unsigned long lastOLED       = 0;
unsigned long lastSerial     = 0;
unsigned long lastSensor     = 0;
unsigned long lastEcgRead    = 0;
unsigned long lastPageSwitch = 0;
unsigned long windowStart    = 0;

uint8_t currentOledPage = 0;


// =============================================================
//  UTILITY
// =============================================================

bool getFormattedTime(char* buf, size_t len) {
  struct tm t;
  if (!getLocalTime(&t, 500)) return false;
  strftime(buf, len, "%d/%m/%Y %H:%M:%S", &t);
  return true;
}

String getShortTime() {
  struct tm t;
  char buf[10];
  if (!getLocalTime(&t, 200)) return "--:--";
  strftime(buf, sizeof(buf), "%H:%M:%S", &t);
  return String(buf);
}


// =============================================================
//  HRV (on-device, display use only)
// =============================================================

HRVFeatures computeHRV(const RRBuffer& buf) {
  HRVFeatures f;
  f.n_rr = buf.count;
  if (buf.count < Config::MIN_VALID_RR) { f.valid = false; return f; }
  double sum = 0.0;
  for (uint16_t i = 0; i < buf.count; i++) sum += buf.intervals[i];
  f.mean_RR = (float)(sum / buf.count);
  f.mean_HR = (f.mean_RR > 0.0f) ? (60000.0f / f.mean_RR) : 0.0f;
  double sq = 0.0;
  for (uint16_t i = 0; i < buf.count; i++) {
    double d = buf.intervals[i] - f.mean_RR;
    sq += d * d;
  }
  f.SDNN = (float)sqrt(sq / (buf.count - 1));
  if (buf.count >= 2) {
    double rmssd_sum = 0.0; uint16_t above50 = 0;
    uint16_t nd = buf.count - 1;
    for (uint16_t i = 1; i < buf.count; i++) {
      float d = buf.intervals[i] - buf.intervals[i-1];
      rmssd_sum += d * d;
      if (fabsf(d) > 50.0f) above50++;
    }
    f.RMSSD = (float)sqrt(rmssd_sum / nd);
    f.pNN50 = (float)above50 / (float)nd * 100.0f;
  }
  f.valid = true;
  return f;
}

void computeBRStats(const BRBuffer& buf, float& m, float& s) {
  m = 0.0f; s = 0.0f;
  if (!buf.count) return;
  double sum = 0.0;
  for (uint8_t i = 0; i < buf.count; i++) sum += buf.rates[i];
  m = (float)(sum / buf.count);
  if (buf.count < 2) return;
  double sq = 0.0;
  for (uint8_t i = 0; i < buf.count; i++) { double d = buf.rates[i]-m; sq += d*d; }
  s = (float)sqrt(sq / (buf.count - 1));
}


// =============================================================
//  WIFI
// =============================================================

void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s\n", Config::SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(Config::SSID, Config::PASSWORD);
  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500); Serial.print("."); retries++;
    esp_task_wdt_reset();
  }
  health.wifi = (WiFi.status() == WL_CONNECTED);
  if (health.wifi)
    Serial.printf("\n[WiFi] Connected  IP:%s  RSSI:%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  else
    Serial.println("\n[WiFi] FAILED -- offline mode.");
}

void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) { health.wifi = false; connectWiFi(); }
}


// =============================================================
//  BUFFER RESET
// =============================================================

void resetWindowBuffers() {
  ecgRawBuf.count    = 0;
  rrBuf.count        = 0;
  brBuf.count        = 0;
  accelRawBuf.count  = 0;
  strainRawBuf.count = 0;
  tempRawBuf.count   = 0;
  tempWelford.reset();
  accWelford.reset();
  // rrBuf.runMean intentionally kept -- seeds the ectopic filter.
}


// =============================================================
//  PAYLOAD BUILDER
//
//  Field names match RawSensorPayload in main.py exactly.
//  Estimated output: 69-73 KB for a full 60-second window.
//
//  SAMPLING RATE NOTE: sampling_rate = 200 (the ECG rate).
//  main.py uses this single value for ALL signals. The acc and
//  resp channels are at ~6.67 Hz, so their server-side
//  processing (Butterworth HP, nk.rsp_rate) will use the wrong
//  fs. Fix this in main.py by adding slow_sample_hz. See header.
// =============================================================

String buildIngestPayload() {
  String s;
  s.reserve(75000);  // Pre-allocate; avoids realloc during concat.

  s  = F("{\"user_id\":\"");
  s += Config::USER_ID;
  s += F("\",\"sampling_rate\":");
  s += (uint32_t)(1000UL / Config::ECG_SAMPLE_INTERVAL);  // 200

  // ECG raw waveform (main.py field: ecg)
  // nk.ecg_clean() + nk.ecg_peaks() run on this array.
  s += F(",\"ecg\":[");
  for (uint16_t i = 0; i < ecgRawBuf.count; i++) {
    if (i) s += ',';
    s += ecgRawBuf.values[i];
  }
  s += ']';

  // Respiration via strain gauge (main.py field: resp)
  s += F(",\"resp\":[");
  for (uint16_t i = 0; i < strainRawBuf.count; i++) {
    if (i) s += ',';
    s += strainRawBuf.values[i];
  }
  s += ']';

  // Skin temperature (main.py field: temp)
  s += F(",\"temp\":[");
  for (uint8_t i = 0; i < tempRawBuf.count; i++) {
    if (i) s += ',';
    s += String(tempRawBuf.values[i], 2);
  }
  s += ']';

  // BMI160 accelerometer (main.py fields: acc_x, acc_y, acc_z)
  s += F(",\"acc_x\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.x[i];
  }
  s += ']';

  s += F(",\"acc_y\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.y[i];
  }
  s += ']';

  s += F(",\"acc_z\":[");
  for (uint16_t i = 0; i < accelRawBuf.count; i++) {
    if (i) s += ',';
    s += accelRawBuf.z[i];
  }
  s += F("]}");

  return s;
}


// =============================================================
//  HTTPS POST TO /ingest
//
//  TLS verification uses ISRG Root X1 (setCACert).
//  If HF_CA_CERT is still the placeholder, falls back to
//  insecure mode with a serial warning.
// =============================================================

bool postToServer(const String& payload) {
  WiFiClientSecure secureClient;

  if (certIsLoaded()) {
    secureClient.setCACert(HF_CA_CERT);
    Serial.println(F("[TLS] Using ISRG Root X1 certificate."));
  } else {
    secureClient.setInsecure();
    Serial.println(F("[TLS] WARNING: CA cert not set -- insecure mode. "
                     "Fill in HF_CA_CERT before production deployment."));
  }

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
      String body = https.getString();
      if (body.length())
        Serial.printf("[HF] Server: %s\n", body.c_str());
    }
  } else {
    Serial.printf("[HF] Error: %s\n", https.errorToString(code).c_str());
  }

  https.end();
  secureClient.stop();
  return ok;
}


// =============================================================
//  60-SECOND WINDOW: COMPUTE + UPLOAD
// =============================================================

void uploadWindowFeatures() {
  char ts[32];
  if (!getFormattedTime(ts, sizeof(ts))) strcpy(ts, "unknown");

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
    "[Window #%lu] valid=%s RR_n=%d HR=%.1f SDNN=%.1f RMSSD=%.1f pNN50=%.1f%%\n"
    "             BR=%.1f+/-%.1f  Temp=%.2f+/-%.3f  AccMag=%.3f+/-%.3f\n"
    "             Raw: ecg=%d  resp=%d  temp=%d  acc=%d\n",
    windowCount, wf.hrv_valid ? "YES" : "NO",
    wf.rr_count, wf.mean_HR, wf.SDNN, wf.RMSSD, wf.pNN50,
    wf.mean_BR, wf.std_BR,
    wf.mean_temp, wf.std_temp,
    wf.mean_acc_mag, wf.std_acc_mag,
    ecgRawBuf.count, strainRawBuf.count, tempRawBuf.count, accelRawBuf.count
  );

  ensureWiFi();
  if (!health.wifi) {
    Serial.println(F("[HF] Skipped -- no WiFi."));
    missedUploads++;
    resetWindowBuffers();
    return;
  }

  uint32_t heap = ESP.getFreeHeap();
  if (heap < Config::MIN_HEAP_FOR_UPLOAD) {
    Serial.printf("[HF] Skipped -- heap too low (%lu B).\n", (unsigned long)heap);
    missedUploads++;
    resetWindowBuffers();
    return;
  }

  Serial.printf("[HF] Building payload (heap: %lu B)...\n", (unsigned long)heap);
  String payload = buildIngestPayload();
  Serial.printf("[HF] Payload: %u B  -> %s\n", payload.length(), Config::HF_HOST);

  bool ok = postToServer(payload);
  payload = String();  // Release heap immediately.

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

// Called every 5 ms (200 Hz).
// Every raw ADC sample is stored in ecgRawBuf regardless of peak
// detection status -- main.py needs the full waveform.
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

  bool connected = !(digitalRead(Config::LO_PLUS) || digitalRead(Config::LO_MINUS));
  if (!connected) {
    if (hrData.leadsConnected) {
      rrBuf.runMean = 700.0f;
      hrData.leadsConnectedAt = 0;
    }
    hrData.leadsConnected = false;
    return;
  }
  if (!hrData.leadsConnected) {
    hrData.leadsConnectedAt = now;
    hrData.leadsConnected   = true;
  }

  analogRead(Config::ECG_PIN);  // Dummy read -- reduces ADC glitch on ESP32-C3.
  int raw = analogRead(Config::ECG_PIN);
  hrData.ecgValue = raw;

  // Store every sample for the /ingest payload (before settle check).
  if (ecgRawBuf.count < Config::ECG_RAW_BUF_SIZE)
    ecgRawBuf.values[ecgRawBuf.count++] = (uint16_t)raw;

  // Moving-average noise filter (display BPM only).
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

  int thresh = sigMin + (int)((sigMax - sigMin) * Config::ECG_THRESH_RATIO);
  int hyst   = (int)((sigMax - sigMin) * Config::ECG_HYST_RATIO);
  hrData.adaptThresh = thresh;

  // R-peak detection with hysteresis.
  if (hrData.ecgSmoothed > thresh && !aboveThreshold) {
    aboveThreshold = true;
    bool settled = (now - hrData.leadsConnectedAt) > Config::ECG_SETTLE_MS;
    if (settled && hrData.lastBeat != 0) {
      float dt_ms = (float)(now - hrData.lastBeat);
      if (dt_ms >= Config::RR_MIN_MS && dt_ms <= Config::RR_MAX_MS) {
        float dev = fabsf(dt_ms - rrBuf.runMean) / rrBuf.runMean;
        bool ectopic = (rrBuf.count >= 5) && (dev > Config::RR_ECTOPIC_THRESH);
        if (!ectopic) {
          if (rrBuf.count < Config::RR_BUFFER_SIZE)
            rrBuf.intervals[rrBuf.count++] = dt_ms;
          rrBuf.runMean = rrBuf.runMean * 0.9f + dt_ms * 0.1f;
          float bpm = 60000.0f / dt_ms;
          hrData.bpm = bpm;
          hrData.rates[hrData.rateSpot++] = bpm;
          hrData.rateSpot %= Config::BPM_RATE_SIZE;
          if (hrData.validBeats < Config::BPM_RATE_SIZE) hrData.validBeats++;
          float s = 0.0f;
          for (int i = 0; i < hrData.validBeats; i++) s += hrData.rates[i];
          hrData.bpmAvg = (int)(s / hrData.validBeats);
        }
      }
    }
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
  health.bmi160 = true;
  Serial.println(F("OK (+/-2g)"));
}

void updateIMU() {
  if (!health.bmi160) return;
  BMI160.readAccelerometer(imuData.ax, imuData.ay, imuData.az);
  BMI160.readGyro(imuData.gx, imuData.gy, imuData.gz);
  float ax_g = imuData.ax / Config::BMI160_ACC_SCALE_G;
  float ay_g = imuData.ay / Config::BMI160_ACC_SCALE_G;
  float az_g = imuData.az / Config::BMI160_ACC_SCALE_G;
  imuData.acc_mag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  accWelford.update(imuData.acc_mag_g);
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
  Serial.printf("%s (%d device[s])\n", health.ds18b20 ? "OK" : "FAILED", n);
  if (health.ds18b20) {
    tempSensor.setResolution(12);
    tempSensor.setWaitForConversion(false);
  }
}

void updateTemperature() {
  if (!health.ds18b20) return;
  static unsigned long lastReq   = 0;
  static bool          requested = false;
  unsigned long        now       = millis();

  if (!requested && (now - lastReq >= 2000)) {
    tempSensor.requestTemperatures();
    requested = true; lastReq = now;
  }
  if (requested && (now - lastReq >= 800)) {
    float c = tempSensor.getTempCByIndex(0);
    if (c == DEVICE_DISCONNECTED_C || c < -55.0f || c > 125.0f) {
      tempData.valid = false; tempData.celsius = 0.0f;
    } else {
      tempData.celsius    = c;
      tempData.fahrenheit = tempSensor.toFahrenheit(c);
      tempData.valid      = true;
      // Only store physiologically plausible skin values (28-38 C).
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
//  Three-stage pipeline: LP filter → DC removal → zero-crossing.
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

  if (strainRawBuf.count < Config::SLOW_BUF_SIZE)
    strainRawBuf.values[strainRawBuf.count++] = adcAvg;

  static float   maBuffer[Config::RESP_FILTER_SIZE] = {0};
  static uint8_t maIdx = 0;
  maBuffer[maIdx] = (float)adcAvg;
  maIdx = (maIdx + 1) % Config::RESP_FILTER_SIZE;
  float maSum = 0;
  for (uint8_t i = 0; i < Config::RESP_FILTER_SIZE; i++) maSum += maBuffer[i];
  respData.filteredRaw = maSum / Config::RESP_FILTER_SIZE;

  static float   dcBuffer[Config::RESP_DC_SIZE] = {0};
  static uint8_t dcIdx = 0;
  dcBuffer[dcIdx] = respData.filteredRaw;
  dcIdx = (dcIdx + 1) % Config::RESP_DC_SIZE;
  float dcSum = 0;
  for (uint8_t i = 0; i < Config::RESP_DC_SIZE; i++) dcSum += dcBuffer[i];
  respData.acComponent = respData.filteredRaw - (dcSum / Config::RESP_DC_SIZE);

  static float prevAC = 0.0f;
  static long  lastBreathLocal = 0;
  respData.breathDetected = false;
  unsigned long now = millis();

  if (prevAC < 0.0f && respData.acComponent >= 0.0f &&
      (now - lastBreathLocal > Config::RESP_MIN_INTERVAL)) {
    if (lastBreathLocal != 0) {
      float rate = 60.0f / ((now - lastBreathLocal) / 1000.0f);
      if (rate >= Config::RESP_MIN && rate <= Config::RESP_MAX) {
        respData.rateInstant = rate;
        respData.rates[respData.rateSpot++] = rate;
        respData.rateSpot %= Config::RESP_RATE_SIZE;
        if (respData.validBreaths < Config::RESP_RATE_SIZE) respData.validBreaths++;
        float rs = 0;
        for (byte i = 0; i < respData.validBreaths; i++) rs += respData.rates[i];
        respData.rateAvg = (int)(rs / respData.validBreaths);
        respData.breathDetected = true;
        if (brBuf.count < Config::BR_BUFFER_SIZE)
          brBuf.rates[brBuf.count++] = rate;
      }
    }
    lastBreathLocal = now;
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
    Serial.println(F("FAILED at 0x3C")); return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); display.cp437(true); display.setTextSize(1);
  display.setCursor(4, 8);  display.print(F("R26-DS-012 RESEARCH"));
  display.setCursor(8, 22); display.print(F("Anxiety Detection"));
  display.setCursor(4, 36); display.print(F("User: ")); display.print(Config::USER_ID);
  display.setCursor(4, 50); display.print(F("Initializing..."));
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
  Serial.println(F("OK"));
}

void drawOledPage0() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK); display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("ECG + HRV"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);
  if (!hrData.leadsConnected) {
    display.setCursor(8, 22); display.print(F("Leads disconnected."));
    display.setCursor(8, 35); display.print(F("Attach electrodes."));
    display.display(); return;
  }
  bool settled = (millis() - hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;
  if (!settled) {
    unsigned long r = (Config::ECG_SETTLE_MS-(millis()-hrData.leadsConnectedAt))/1000UL+1;
    display.setCursor(8, 22); display.print(F("Calibrating ECG..."));
    display.setCursor(8, 35); display.printf("Ready in %lus", r);
    display.display(); return;
  }
  display.setTextSize(3); display.setCursor(0, 14); display.print(hrData.bpmAvg);
  display.setTextSize(1);
  display.setCursor(56, 16); display.print(F("BPM avg"));
  display.setCursor(56, 26); display.printf("ECG:%d", ecgRawBuf.count);
  display.setCursor(0, 44);
  if (lastWindow.hrv_valid) {
    display.printf("RMSSD:%.1f SDNN:%.1f", lastWindow.RMSSD, lastWindow.SDNN);
    display.setCursor(0, 54); display.printf("pNN50: %.1f%%", lastWindow.pNN50);
  } else {
    display.print(F("HRV: collecting..."));
    display.setCursor(0, 54);
    display.printf("RR: %d / %d min", rrBuf.count, (int)Config::MIN_VALID_RR);
  }
  display.display();
}

void drawOledPage1() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK); display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("RESPIRATION"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(3); display.setCursor(0, 14);
  if (respData.validBreaths > 0) display.print(respData.rateAvg);
  else                           display.print(F("--"));
  display.setTextSize(1);
  display.setCursor(56, 22); display.print(F("br/min"));
  display.setCursor(0, 44);
  if (respData.rateInstant > 0) display.printf("Inst: %.1f br/min", respData.rateInstant);
  else                          display.print(F("Calibrating..."));
  display.setCursor(0, 54);
  if (lastWindow.mean_BR > 0.0f)
    display.printf("Avg:%.1f Std:%.1f", lastWindow.mean_BR, lastWindow.std_BR);
  display.drawLine(100, 63, 127, 63, SSD1306_WHITE);
  int barH = (int)constrain(respData.acComponent / 20.0f, -6.0f, 6.0f);
  if (barH >= 0) display.fillRect(100, 63-barH, 27, barH+1, SSD1306_WHITE);
  else           display.fillRect(100, 63, 27, -barH, SSD1306_WHITE);
  if (respData.breathDetected) display.fillCircle(120, 56, 3, SSD1306_WHITE);
  display.display();
}

void drawOledPage2() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK); display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("TEMP + MOTION"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 15);
  if (tempData.valid) display.printf("Temp: %.2f C", tempData.celsius);
  else               display.print(F("Temp: -- sensor ERR"));
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
  display.printf("A:[%d,%d,%d]k", imuData.ax/1000, imuData.ay/1000, imuData.az/1000);
  display.display();
}

void drawOledPage3() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK); display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("SYSTEM"));
  display.setCursor(72, 2); display.print(getShortTime());
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 14); display.printf("User: %s", Config::USER_ID);
  display.setCursor(0, 23); display.printf("Win:#%lu ECG:%d", windowCount, ecgRawBuf.count);
  display.setCursor(0, 33); display.printf("Missed: %lu", missedUploads);
  display.setCursor(0, 43);
  display.printf("WiFi:%s Srv:%s %dkB",
    health.wifi   ? "Y" : "N",
    health.server ? "Y" : "N",
    ESP.getFreeHeap() / 1024);
  display.setCursor(0, 53);
  display.printf("RSSI:%d TLS:%s", (int)WiFi.RSSI(), certIsLoaded() ? "ON" : "OFF");
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
  char ts[32]; getFormattedTime(ts, sizeof(ts));
  bool settled = hrData.leadsConnected &&
                 (millis()-hrData.leadsConnectedAt) >= Config::ECG_SETTLE_MS;
  Serial.printf(
    "[%s] User=%s  Win=#%lu  FW=%s\n"
    "  ECG  : BPM=%d(avg) %.1f(inst) leads=%s settled=%s thr=%d ecg_n=%d\n"
    "  HRV  : HR=%.1f RR=%.1fms SDNN=%.1f RMSSD=%.1f pNN50=%.1f%% [valid=%s]\n"
    "  RESP : avg=%d inst=%.1f BR_n=%d resp_n=%d\n"
    "  TEMP : %.2fC valid=%s temp_n=%d\n"
    "  ACC  : mag=%.3fg [%d,%d,%d] acc_n=%d\n"
    "  SYS  : WiFi=%s Srv=%s TLS=%s RSSI=%d Heap=%dkB Missed=%lu\n\n",
    ts, Config::USER_ID, windowCount, Config::FW_VERSION,
    hrData.bpmAvg, hrData.bpm,
    hrData.leadsConnected ? "YES" : "NO",
    settled ? "YES" : "NO",
    hrData.adaptThresh, ecgRawBuf.count,
    lastWindow.mean_HR, lastWindow.mean_RR,
    lastWindow.SDNN, lastWindow.RMSSD, lastWindow.pNN50,
    lastWindow.hrv_valid ? "YES" : "NO",
    respData.rateAvg, respData.rateInstant, brBuf.count, strainRawBuf.count,
    tempData.celsius, tempData.valid ? "YES" : "NO", tempRawBuf.count,
    imuData.acc_mag_g, imuData.ax, imuData.ay, imuData.az, accelRawBuf.count,
    health.wifi   ? "Y" : "N",
    health.server ? "Y" : "N",
    certIsLoaded() ? "Y" : "N",
    (int)WiFi.RSSI(), ESP.getFreeHeap() / 1024, missedUploads
  );
}


// =============================================================
//  SETUP
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n\n=== R26-DS-012 Anxiety Detection Firmware v2.1.0 ==="));
  Serial.printf("User   : %s\n", Config::USER_ID);
  Serial.printf("Host   : %s\n", Config::HF_HOST);
  Serial.printf("TLS    : %s\n", certIsLoaded() ? "ISRG Root X1" : "INSECURE (cert not set)");

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
    Serial.print(F("[NTP] Syncing"));
    for (uint8_t i = 0; i < 20; i++) {
      struct tm info;
      if (getLocalTime(&info, 500)) { Serial.println(F(" OK")); break; }
      Serial.print('.'); esp_task_wdt_reset();
    }
  }

  resetWindowBuffers();
  rrBuf.runMean  = 700.0f;
  windowStart    = millis();
  lastPageSwitch = millis();

  Serial.println(F("[System] Ready -- entering collection loop.\n"));
  esp_task_wdt_reset();
}


// =============================================================
//  LOOP
//
//  Every 5 ms     ECG sample + R-peak detection (200 Hz)
//  Every 150 ms   IMU, temperature, strain gauge
//  Every 300 ms   OLED refresh
//  Every 1000 ms  Serial telemetry
//  Every 60 s     POST raw payload to HF /ingest
//
//  windowStart is advanced before uploadWindowFeatures() so the
//  next window accumulates data during the upload.
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
