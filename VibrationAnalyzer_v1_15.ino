// Verzija 1.13.1
// Based on v1.7_RTC by Vladimir Divic
// Added: RTC timestamp for each record, serial command to set RTC
//
// Changes in v1.8:
// - Migrated to M5Unified framework (replaces M5StickCPlus library).
// - Removed BluetoothSerial support (Serial-only).
//
//
// FEATURES:
// - Auto-saves each recording to flash storage with RTC timestamp
// - Circular buffer: oldest records overwritten when full (~50 records max)
// - Browse through stored records on Screen 10
// - Data persists across power cycles
// - RTC time setting via Serial
//
// CONTROLS:
// - BtnA: Record (or browse prev record on Screen 10)
// - BtnB short: Cycle screens (1-10)
// - BtnB long: Serial dump (or clear all records on Screen 10)
// - BtnB held at boot: External trigger mode
//
// SERIAL COMMANDS:
// - 'R' : Trigger recording
// - 'E' : Export all records
// - 'I' : Print storage info
// - 'T' : Print current RTC time
// - "Tyyyy-MM-dd HH:mm:ss" : Set RTC time (e.g., "T2025-01-15 14:30:00")
// - 'A' : Dump current time series (idx, t_us, ax, ay, az)
// - 'F' : Dump current FFT (f_Hz, X, Y, Z)
// - 'S' : Dump stats + FFT peaks + log decrement summary
// - 'Cxx' : Load record with logical index xx (0..N-1); use C-1 for LIVE
//
// SCREENS:
// 0 - Startup/info
// 1 - Trigger settings
// 2 - Acquisition setup
// 3 - Statistics
// 4 - Time series
// 5 - Envelope & log decrement
// 6 - Spectrum
// 7 - FFT Peaks (1-7)
// 8 - X-Y / X-Z correlation
// 9 - Spectrogram
// 10 - Storage browser

// Changes in v1.9:
// - Added Trigger settings screen (now Screen 1) with configurable trigger modes.
// - New trigger modes: Immediate (BtnA), Time-delay (1/3/10 s), Microphone level, Accelerometer magnitude.
// - Mic trigger: adjustable level + start delay (default 100 ms).
// - Acc trigger: adjustable |a| threshold (default 1.2 g).
// - Added simple trigger state machine (armed/pending/recording) without Bluetooth.
//
//
// Changes in v1.9.1:
// - Fixed compile error by removing duplicate LONG_PRESS_MS declaration in loop().
// - Version banner updated.
//
// Changes in v1.9.2:
// - Reordered screens: Trigger settings is now Screen 1 (first in cycle).
// - Added armed indicator: a small green square is shown when a non-immediate trigger is armed/pending.
// - Acceleration trigger threshold is now user-selectable from 1.0g to 2.0g in 0.1g steps (default 1.5g).

// Changes in v1.9.3:
// - Trigger screen UI: when Mode=Accel, the parameter line shows acceleration threshold (instead of delay).
// - Removed separate Acc threshold line to avoid overlap with status area.

// Changes in v1.9.4:
// - Trigger screen layout compacted to ensure the active parameter (Delay/Acc thr) is always visible.

// Changes in v1.9.5:
// - Removed microphone trigger mode (it was unreliable on the current hardware setup).
// - Simplified Trigger screen menu accordingly (Mode + Delay/Acc thr only).

// Changes in v1.10.0:
// - TTNS_US (time-to-next-sample) converted from milliseconds to microseconds; sampling scheduler now uses micros() so higher Fs works reliably.
// - Added Acquisition setup screen (Screen 2) to select sample count (512/1024/2048) and sampling rate (100/200/400/800/1600 Hz).
// - Default acquisition settings: 1024 samples @ 200 Hz (TTNS_US=4000 us).
// - Recording length (duration) is derived and shown on Screen 2.
// - FFT magnitude spectra are now normalized by sample count so amplitude is comparable across different N.
// - Storage format upgraded (backwards compatible): new records store sample count + nominal Fs in header; old records (v1.9.x) still load as 1024 samples.
// Changes in v1.10.1:
// - Fixed compile error: ensured drawAcquisitionSetupScreen() is forward-declared before loop() for toolchains that don't auto-prototype it reliably.

// Changes in v1.11.0:
// - Added Screen 5 (Envelope & log decrement): draws signal envelope by connecting local maxima for X/Y/Z.
// - Fits an exponential decay with offset (y=C + A*exp(-b*t)) through maxima when >= 6 peaks are available.
// - Computes goodness-of-fit R2; if R2 > 0.80, computes logarithmic decrement (delta = b * Tpeak).
// - Envelope colors: X=orange, Y=yellow, Z=cyan; fit results printed under plot in matching colors.
// - Screen order updated: screens are now 1-9 (Storage is Screen 9).

// Changes in v1.12.0:
// - Added Screen 7: FFT Peaks table (top 7 peaks for X/Y/Z) after Spectrum.
// - Updated screen count to 10 (Storage is Screen 10).

// Changes in v1.12.1:
// - Fixed Spectrogram rendering at Fs >= 400 Hz: enforce true single-sided spectrum (0..Fs/2) by using bins 0..N/2 (incl. Nyquist).
// - Spectrogram x-axis label now uses the effective record Fs, and spectrogram magnitudes use the same single-sided normalization used elsewhere.


// Changes in v1.13.1:
// - Persist Trigger (Screen 1) + Acquisition setup (Screen 2) settings to NVS (Preferences) and restore them on boot.
// - Added Serial commands: A (dump time series), F (dump FFT), S (dump stats+FFT peaks+log decrement), Cxx (select record).
// - Serial command R now arms recording via current trigger mode (same behavior as BtnA outside Storage screen).
// - When loading a stored record, reconstruct the time vector t[i] from stored sampling frequency for correct time-series serial dumps.

// Changes in v1.13.1:
// - Fixed compile errors: added forward declarations for serial dump functions and armRecording(), and moved redrawRequested definition above its first use.

// Changes in v1.14.0:
// - Explicit ending of data stream 

// Changes in v1.14.1:
// Intro

#include <M5Unified.h>
#include "arduinoFFT.h"
#include <math.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "vibe_intro.h"

// #ifndef M5_LED
#define M5_LED 19 // 10 for M5 StickC Plus
#define LEDINV false
//#endif

#ifndef TFT_ORANGE
  #define TFT_ORANGE 0xFD20
#endif

#ifndef BLACK
  #define BLACK   TFT_BLACK
  #define WHITE   TFT_WHITE
  #define RED     TFT_RED
  #define GREEN   TFT_GREEN
  #define BLUE    TFT_BLUE
  #define YELLOW  TFT_YELLOW
  #define CYAN    TFT_CYAN
  #define MAGENTA TFT_MAGENTA
  #define ORANGE  TFT_ORANGE
#endif

#define MAX_SAMPLES 2048
#define CALIB_T_PIN 26
#define EXT_TRIG_PIN 36

// ---------- Acquisition Settings (Screen 2) ----------
// Selectable sample count and sampling rate (TTNS_US in microseconds).
static uint16_t sampleCountSetting = 1024;   // 512, 1024, 2048
static uint16_t samplingRateSettingHz = 200; // 100, 200, 400, 800, 1600
static uint32_t TTNS_US = 4000;             // time-to-next-sample in microseconds

// Active record length (may differ per stored record)
static uint16_t recordSamples = 1024;
static uint16_t recordFsNomHz = 200;

// Back-compat helper: use SAMPLES as the active record length (runtime)
#define SAMPLES (recordSamples)


// ---------- Storage Configuration ----------
#define MAX_RECORDS      40           // Maximum stored records (~50 fits in 1.5MB)
#define RECORDS_DIR      "/records"
#define METADATA_FILE    "/meta.dat"

// Structure for RTC date/time (matches M5 RTC structure)
struct RecordDateTime {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
};

// Structure for record metadata (stored at beginning of each record file)
struct RecordMeta {
    uint32_t timestamp;               // millis() when recorded (for relative timing)
    RecordDateTime rtcTime;           // RTC time when recording started
    float samplingFreq;
    float peakX, peakY, peakZ;
    float maxX, maxY, maxZ;
    float minX, minY, minZ;
    float meanX, meanY, meanZ;
    float sdX, sdY, sdZ;
};

// Record file format v2 (backwards compatible with v1.9.x):
// [RecordHeaderV2][accX][accY][accZ][specX][specY][specZ]
// where each array length = header.samples
static const uint32_t REC_MAGIC_V2 = 0x32564156u; // 'VAV2'

struct RecordHeaderV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t samples;
    uint16_t fsNomHz;
    uint16_t reserved;
    RecordMeta meta;
};


// Current record's RTC time (set when recording starts)
RecordDateTime currentRecordTime;

// Storage state variables
int  storedRecordCount = 0;
int  currentViewRecord = -1;          // -1 = live data, 0..N-1 = stored records
int  oldestRecordIndex = 0;           // For circular buffer
int  newestRecordIndex = -1;
bool storageReady = false;

// ---------- Acceleration Data Arrays ----------
float accXvec[MAX_SAMPLES];
float accYvec[MAX_SAMPLES];
float accZvec[MAX_SAMPLES];

float vRealX[MAX_SAMPLES];
float vRealY[MAX_SAMPLES];
float vRealZ[MAX_SAMPLES];

float vReal[MAX_SAMPLES];
float vImag[MAX_SAMPLES];

// ---------- Spectrogram (Screen 9) ----------
#define SEG_LEN          128          // window length for each FFT
#define SEG_OVERLAP_PCT  50           // overlap percentage between windows
#define SEG_MAX          17           // max number of segments stored per record
// For real-valued signals, unique spectrum bins are 0..N/2 inclusive (Nyquist included).
// Using only this range avoids plotting the mirrored (negative-frequency) half.
#define SPEC_BINS        ((SEG_LEN / 2) + 1)

float specMag[SEG_MAX][SPEC_BINS];
int   specSegments = 0;
float specLogMin = 0.0f;
float specLogMax = 1.0f;
float segReal[SEG_LEN];
float segImag[SEG_LEN];

// ---------- Current Readings ----------
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

long int t[MAX_SAMPLES];
long int t0;
uint32_t prevSampleUs;  // previous sample timestamp (micros)
int      count;
bool     isLogging;
bool     isTrig;
uint16_t trigLevel = 3000;
bool     isExtTriged;
ArduinoFFT<float> FFT = ArduinoFFT<float>();

static inline void normalizeSpectrum(float* mag, int n) {
    if (n <= 0) return;
    const float s = 2.0f / (float)n;
    const int half = n / 2;
    for (int i = 0; i < half; i++) {
        mag[i] *= s;
    }
    // Note: DC/Nyquist special-casing omitted; this is a pragmatic normalization for cross-N comparability.
}

double samplingFrequency = 0;
double xM;  // x,y,z mode freq
double yM;
double zM;

float maxV[3];
float meanV[3];
float sdV[3];
float minV[3];

long   timeout  = 60000;
long   extendT  = 60000;
int   th;        // text height
int   curDisp;   // 0 startup, 1 trigger, 2 setup, 3 stats, 4 timeseries, 5 spectrum, 6 corr, 7 spectro, 8 storage
Preferences preferences;

// ============================================================================
// TRIGGER SETTINGS (Screen 1)
// ============================================================================

enum TriggerMode : uint8_t {
    TRIG_IMMEDIATE = 0,   // Start recording immediately on BtnA
    TRIG_DELAY     = 1,   // Start after a fixed delay on BtnA
    TRIG_ACCEL     = 2    // Start when |a| exceeds threshold
};

TriggerMode trigMode = TRIG_IMMEDIATE;

// Time-delay trigger
const uint16_t delayOptionsMs[3] = {1000, 3000, 10000};
uint8_t delayIndex = 0; // 0=1s, 1=3s, 2=10s

// Acc trigger
float accelTrigThresholdG = 1.5f;   // Default threshold in g (1.0 .. 2.0)

// Trigger state machine
bool triggerArmed      = false;     // Waiting for mic/acc/external condition
bool startPending      = false;     // Trigger condition met, waiting for start time
uint32_t pendingStartMs = 0;        // millis() when recording should start
uint32_t armStartedMs   = 0;        // millis() when arming occurred

// Screen 1 menu navigation
uint8_t trigMenuItem = 0;           // Which setting is selected on Screen 1
const uint8_t TRIG_MENU_ITEMS = 2;  // Mode, Param(Delay/AccThr)

// Live readouts for Screen 1
float    accelMag_live    = 0.0f;


// ============================================================================
// ACQUISITION SETUP (Screen 2)
// ============================================================================
static const uint16_t sampleCountOptions[3] = {512, 1024, 2048};
static const uint16_t samplingRateOptionsHz[5] = {100, 200, 400, 800, 1000};
static const uint32_t ttnsOptionsUs[5]        = {10000, 5000, 2500, 1250, 1000};
static uint8_t sampleCountIndex = 1;      // default 1024
static uint8_t samplingRateIndex = 1;     // default 200
static uint8_t setupMenuItem = 0;         // 0=Samples, 1=Fs
static const uint8_t SETUP_MENU_ITEMS = 2;


int DW = 240;
int DH = 135;

// ============================================================================
// ARMED INDICATOR (small green square in corner)
// Shows when a non-immediate trigger is armed or pending (delay/mic/acc/external).
// ============================================================================
static bool isNonImmediateArmed() {
    if (!isLogging || isTrig) return false;
    if (!isExtTriged && (trigMode == TRIG_IMMEDIATE)) return false;
    return true; // includes delay pending, mic/acc armed, and external trigger armed
}

static void drawArmedIndicator(bool on) {
    const int sz = 8;
    const int x = DW - sz - 2;
    const int y = 2;
    M5.Display.fillRect(x, y, sz, sz, on ? GREEN : BLACK);
}

static void updateArmedIndicator() {
    static bool prev = false;
    const bool now = isNonImmediateArmed();
    if (now != prev) {
        drawArmedIndicator(now);
        prev = now;
    }
}


// Serial input buffer for time setting
String serialBuffer = "";

// ============================================================================
// RTC FUNCTIONS
// ============================================================================

// Get current RTC time into RecordDateTime structure
void getRtcTime(RecordDateTime* dt) {
    // M5Unified RTC8563 uses rtc_datetime_t (year 1900-2099)
    if (!M5.Rtc.isEnabled()) {
        dt->year = 0; dt->month = 0; dt->day = 0;
        dt->hour = 0; dt->minute = 0; dt->second = 0;
        return;
    }
    auto now = M5.Rtc.getDateTime();
    dt->year   = now.date.year;
    dt->month  = now.date.month;
    dt->day    = now.date.date;
    dt->hour   = now.time.hours;
    dt->minute = now.time.minutes;
    dt->second = now.time.seconds;
}

// Set RTC time from RecordDateTime structure
void setRtcTime(RecordDateTime* dt) {
    if (!M5.Rtc.isEnabled()) return;

    // rtc_datetime_t initializer: { {year, month, date}, {hours, minutes, seconds} }
    M5.Rtc.setDateTime({ { (int16_t)dt->year, (int8_t)dt->month, (int8_t)dt->day }, 
                         { (int8_t)dt->hour, (int8_t)dt->minute, (int8_t)dt->second } });
}

// Parse time string "yyyy-MM-dd HH:mm:ss" and set RTC
bool parseAndSetTime(String timeStr) {
    // Expected format: "yyyy-MM-dd HH:mm:ss" (19 characters)
    // Example: "2025-01-15 14:30:00"
    
    timeStr.trim();
    
    if (timeStr.length() < 19) {
        Serial.println("Error: Time string too short");
        Serial.println("Format: yyyy-MM-dd HH:mm:ss");
        return false;
    }
    
    RecordDateTime dt;
    
    // Parse year (positions 0-3)
    dt.year = timeStr.substring(0, 4).toInt();
    
    // Parse month (positions 5-6)
    dt.month = timeStr.substring(5, 7).toInt();
    
    // Parse day (positions 8-9)
    dt.day = timeStr.substring(8, 10).toInt();
    
    // Parse hour (positions 11-12)
    dt.hour = timeStr.substring(11, 13).toInt();
    
    // Parse minute (positions 14-15)
    dt.minute = timeStr.substring(14, 16).toInt();
    
    // Parse second (positions 17-18)
    dt.second = timeStr.substring(17, 19).toInt();
    
    // Validate ranges
    if (dt.year < 2020 || dt.year > 2099) {
        Serial.println("Error: Invalid year (2020-2099)");
        return false;
    }
    if (dt.month < 1 || dt.month > 12) {
        Serial.println("Error: Invalid month (1-12)");
        return false;
    }
    if (dt.day < 1 || dt.day > 31) {
        Serial.println("Error: Invalid day (1-31)");
        return false;
    }
    if (dt.hour > 23) {
        Serial.println("Error: Invalid hour (0-23)");
        return false;
    }
    if (dt.minute > 59) {
        Serial.println("Error: Invalid minute (0-59)");
        return false;
    }
    if (dt.second > 59) {
        Serial.println("Error: Invalid second (0-59)");
        return false;
    }
    
    // Set the RTC
    setRtcTime(&dt);
    
    Serial.print("RTC set to: ");
    printDateTime(&dt);
    Serial.println();
    
    return true;
}

// Print RecordDateTime to Serial
void printDateTime(RecordDateTime* dt) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt->year, dt->month, dt->day,
             dt->hour, dt->minute, dt->second);
    Serial.print(buf);
}


// Format RecordDateTime to string buffer
void formatDateTime(RecordDateTime* dt, char* buf, size_t bufSize) {
    snprintf(buf, bufSize, "%04d-%02d-%02d %02d:%02d:%02d",
             dt->year, dt->month, dt->day,
             dt->hour, dt->minute, dt->second);
}

// Format short date/time for display (fits on small screen)
void formatDateTimeShort(RecordDateTime* dt, char* buf, size_t bufSize) {
    snprintf(buf, bufSize, "%02d/%02d %02d:%02d",
             dt->day, dt->month,
             dt->hour, dt->minute);
}

// Print current RTC time
void printCurrentTime() {
    RecordDateTime now;
    getRtcTime(&now);
    Serial.print("Current RTC: ");
    printDateTime(&now);
    Serial.println();
}


// Forward declarations for functions used by processSerialInput()
static bool redrawRequested = false;

static void armRecording();
static void disarmRecording();
void dumpTimeSeriesSerial();
void dumpFftSerial();
void dumpSummarySerial();

bool loadRecord(int logicalIndex);
void exportAllRecordsSerial();
void printStorageInfo();

// Process serial input for time setting
void processSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                // Process command
                if (serialBuffer.charAt(0) == 'T' || serialBuffer.charAt(0) == 't') {
                    if (serialBuffer.length() == 1) {
                        // Just 'T' - print current time
                        printCurrentTime();
                    } else {
                        // 'T' followed by time string
                        String timeStr = serialBuffer.substring(1);
                        parseAndSetTime(timeStr);
                    }
                }
                else if (serialBuffer.charAt(0) == 'R' || serialBuffer.charAt(0) == 'r') {
                    armRecording();
                }
                else if (serialBuffer.charAt(0) == 'E' || serialBuffer.charAt(0) == 'e') {
                    exportAllRecordsSerial();
                }
                else if (serialBuffer.charAt(0) == 'I' || serialBuffer.charAt(0) == 'i') {
                    printStorageInfo();
                }
                else if (serialBuffer.charAt(0) == 'A' || serialBuffer.charAt(0) == 'a') {
                    dumpTimeSeriesSerial();
                }
                else if (serialBuffer.charAt(0) == 'F' || serialBuffer.charAt(0) == 'f') {
                    dumpFftSerial();
                }
                else if (serialBuffer.charAt(0) == 'S' || serialBuffer.charAt(0) == 's') {
                    dumpSummarySerial();
                }
                else if (serialBuffer.charAt(0) == 'C' || serialBuffer.charAt(0) == 'c') {
                    String s = serialBuffer.substring(1);
                    s.trim();
                    int idx = s.length() ? s.toInt() : currentViewRecord;
                    if (isTrig || isLogging) {
                        Serial.println("BUSY: stop/cancel recording first");
                    } else if (idx < 0) {
                        currentViewRecord = -1;
                        Serial.println("Selected LIVE");
                        redrawRequested = true;
                    } else if (!storageReady || storedRecordCount == 0) {
                        Serial.println("No stored records");
                    } else if (idx >= storedRecordCount) {
                        Serial.println("Index out of range");
                    } else {
                        loadRecord(idx);
                        redrawRequested = true;
                    }
                }
                else if (serialBuffer.charAt(0) == 'H' || serialBuffer.charAt(0) == 'h' || serialBuffer.charAt(0) == '?') {
                    // Help
                    Serial.println("=== Serial Commands ===");
                    Serial.println("R - Arm recording (uses Trigger settings)");
                    Serial.println("A - Dump current time series (idx t_us ax ay az)");
                    Serial.println("F - Dump current FFT (f_Hz X Y Z)");
                    Serial.println("S - Dump summary: stats + FFT peaks + log decrement");
                    Serial.println("Cxx - Load record xx (0..N-1); C-1 for LIVE");
                    Serial.println("T - Print current RTC time");
                    Serial.println("Tyyyy-MM-dd HH:mm:ss - Set RTC time");
                    Serial.println("  Example: T2025-01-15 14:30:00");
                    Serial.println("E - Export all records");
                    Serial.println("I - Print storage info");
                    Serial.println("H or ? - This help");
                    Serial.println("=======================");
                }
                
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
            // Prevent buffer overflow
            if (serialBuffer.length() > 30) {
                serialBuffer = "";
            }
        }
    }
}



// ============================================================================
// TRIGGER HELPERS
// ============================================================================

static inline float accelMagnitudeG(float ax, float ay, float az) {
    return sqrtf(ax*ax + ay*ay + az*az);
}

static void applyAcquisitionSettings() {
    // Clamp indices
    if (sampleCountIndex > 2) sampleCountIndex = 1;
    if (samplingRateIndex > 4) samplingRateIndex = 1;

    sampleCountSetting    = sampleCountOptions[sampleCountIndex];
    samplingRateSettingHz = samplingRateOptionsHz[samplingRateIndex];
    TTNS_US               = ttnsOptionsUs[samplingRateIndex];
}


// ============================================================================
// PERSISTENT USER SETTINGS (NVS Preferences)
// Stores Trigger + Acquisition setup so they survive power cycles.
// ============================================================================

static const char* PREF_NS = "viban";
static bool settingsLoaded = false;


void loadUserSettings() {
    preferences.begin(PREF_NS, true);
    uint8_t tm = preferences.getUChar("trigMode", (uint8_t)TRIG_IMMEDIATE);
    trigMode = (tm <= (uint8_t)TRIG_ACCEL) ? (TriggerMode)tm : TRIG_IMMEDIATE;

    delayIndex = preferences.getUChar("delayIdx", 0);
    if (delayIndex > 2) delayIndex = 0;

    accelTrigThresholdG = preferences.getFloat("accThrG", 1.5f);
    if (accelTrigThresholdG < 1.0f) accelTrigThresholdG = 1.0f;
    if (accelTrigThresholdG > 2.0f) accelTrigThresholdG = 2.0f;

    sampleCountIndex = preferences.getUChar("nIdx", 1);
    if (sampleCountIndex > 2) sampleCountIndex = 1;

    samplingRateIndex = preferences.getUChar("fsIdx", 1);
    if (samplingRateIndex > 4) samplingRateIndex = 1;

    preferences.end();
    applyAcquisitionSettings();
    settingsLoaded = true;
}

void saveUserSettings() {
    if (!settingsLoaded) return;
    preferences.begin(PREF_NS, false);
    preferences.putUChar("trigMode", (uint8_t)trigMode);
    preferences.putUChar("delayIdx", delayIndex);
    preferences.putFloat("accThrG", accelTrigThresholdG);
    preferences.putUChar("nIdx", sampleCountIndex);
    preferences.putUChar("fsIdx", samplingRateIndex);
    preferences.end();
}



static bool isTriggerMenuItemVisible(uint8_t item) {
    if (item == 0) return true; // Mode always visible
    // Item 1 is a shared "parameter" line: Delay (TRIG_DELAY) or Acc threshold (TRIG_ACCEL)
    if (item == 1) return (trigMode == TRIG_DELAY) || (trigMode == TRIG_ACCEL);
    return false;
}

static uint8_t nextVisibleTriggerMenuItem(uint8_t current) {
    for (uint8_t k = 0; k < TRIG_MENU_ITEMS; k++) {
        current = (current + 1) % TRIG_MENU_ITEMS;
        if (isTriggerMenuItemVisible(current)) return current;
    }
    return 0;
}

static void disarmRecording() {
    isLogging = false;
    isTrig = false;
    triggerArmed = false;
    startPending = false;
    pendingStartMs = 0;
    armStartedMs = 0;
}

static void startRecordingNow() {
    // Start sampling immediately with current acquisition settings
    recordSamples  = sampleCountSetting;
    recordFsNomHz  = samplingRateSettingHz;
    count          = 0;
    // Make the first sample happen immediately
    prevSampleUs   = micros() - TTNS_US;
    isTrig         = true;
}


static void armRecording() {
    // If already armed (but not triggered), treat BtnA as cancel
    if (isLogging && !isTrig) {
        disarmRecording();
        return;
    }

    isLogging = true;
    isTrig = false;
    triggerArmed = false;
    startPending = false;
    pendingStartMs = 0;
    armStartedMs = millis();

    // External trigger mode (BtnB held at boot) keeps original behavior:
    // pressing BtnA arms, EXT_TRIG_PIN crossing starts sampling.
    if (isExtTriged) {
        triggerArmed = true;
        return;
    }

    // Internal trigger modes
    if (trigMode == TRIG_IMMEDIATE) {
        startRecordingNow();
        return;
    }

    if (trigMode == TRIG_DELAY) {
        startPending = true;
        pendingStartMs = armStartedMs + delayOptionsMs[delayIndex];
        return;
    }

    // Acc: arm and wait for threshold
    if (trigMode == TRIG_ACCEL) {
        triggerArmed = true;
    }
}


static void updateTriggerState() {
    if (!isLogging || isTrig) return;

    const uint32_t now = millis();

    // If we are waiting for the scheduled start moment
    if (startPending) {
        if ((int32_t)(now - pendingStartMs) >= 0) {
            startRecordingNow();
            startPending = false;
            triggerArmed = false;
        }
        return;
    }

    // External trigger mode
    if (isExtTriged) {
        if (analogRead(EXT_TRIG_PIN) > 4000) {
            startRecordingNow();
            triggerArmed = false;
        }
        return;
    }

    // Internal arming (mic/acc)
    if (!triggerArmed) return;

    if (trigMode == TRIG_ACCEL) {
        // Read accel once and check magnitude
        float ax, ay, az;
        M5.Imu.getAccel(&ax, &ay, &az);
        accelMag_live = accelMagnitudeG(ax, ay, az);
        if (accelMag_live >= accelTrigThresholdG) {
            startRecordingNow();
            triggerArmed = false;
        }
        return;
    }
}

// ============================================================================
// STORAGE FUNCTIONS
// ============================================================================

// Initialize LittleFS storage
bool initStorage() {
    if (!LittleFS.begin(true)) {  // true = format if mount fails
        Serial.println("LittleFS mount failed");
        return false;
    }
    
    // Create records directory if needed
    if (!LittleFS.exists(RECORDS_DIR)) {
        LittleFS.mkdir(RECORDS_DIR);
    }
    
    // Load metadata
    loadStorageMetadata();
    
    Serial.print("Storage initialized. Records: ");
    Serial.print(storedRecordCount);
    Serial.print("/");
    Serial.print(MAX_RECORDS);
    Serial.print(" | Used: ");
    Serial.print(LittleFS.usedBytes() / 1024);
    Serial.print("/");
    Serial.print(LittleFS.totalBytes() / 1024);
    Serial.println(" KB");
    
    return true;
}

// Save storage state metadata
void saveStorageMetadata() {
    File file = LittleFS.open(METADATA_FILE, "w");
    if (file) {
        file.write((uint8_t*)&storedRecordCount, sizeof(int));
        file.write((uint8_t*)&oldestRecordIndex, sizeof(int));
        file.write((uint8_t*)&newestRecordIndex, sizeof(int));
        file.close();
    }
}

// Load storage state metadata
void loadStorageMetadata() {
    File file = LittleFS.open(METADATA_FILE, "r");
    if (file && file.size() >= 3 * sizeof(int)) {
        file.read((uint8_t*)&storedRecordCount, sizeof(int));
        file.read((uint8_t*)&oldestRecordIndex, sizeof(int));
        file.read((uint8_t*)&newestRecordIndex, sizeof(int));
        file.close();
        
        // Sanity check
        if (storedRecordCount < 0 || storedRecordCount > MAX_RECORDS) {
            storedRecordCount = 0;
            oldestRecordIndex = 0;
            newestRecordIndex = -1;
        }
    } else {
        storedRecordCount = 0;
        oldestRecordIndex = 0;
        newestRecordIndex = -1;
        if (file) file.close();
    }
}

// Save current record to storage
bool saveCurrentRecord() {
    if (!storageReady) return false;
    
    // Determine next record index (circular buffer)
    newestRecordIndex = (newestRecordIndex + 1) % MAX_RECORDS;
    
    if (storedRecordCount >= MAX_RECORDS) {
        // Overwriting oldest
        oldestRecordIndex = (oldestRecordIndex + 1) % MAX_RECORDS;
    } else {
        storedRecordCount++;
    }
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/rec%03d.bin", RECORDS_DIR, newestRecordIndex);
    
    File file = LittleFS.open(filename, "w");
    if (!file) {
        Serial.println("Failed to open file for writing");
        return false;
    }
    
    // Prepare metadata
    RecordMeta meta;
    meta.timestamp = millis();
    meta.rtcTime = currentRecordTime;  // RTC time captured at recording start
    meta.samplingFreq = samplingFrequency;
    meta.peakX = xM;
    meta.peakY = yM;
    meta.peakZ = zM;
    
    // Calculate statistics for metadata
    meta.maxX = meta.maxY = meta.maxZ = -9999.0f;
    meta.minX = meta.minY = meta.minZ = 9999.0f;
    meta.meanX = meta.meanY = meta.meanZ = 0.0f;
    
    for (int i = 0; i < SAMPLES; i++) {
        if (accXvec[i] > meta.maxX) meta.maxX = accXvec[i];
        if (accXvec[i] < meta.minX) meta.minX = accXvec[i];
        if (accYvec[i] > meta.maxY) meta.maxY = accYvec[i];
        if (accYvec[i] < meta.minY) meta.minY = accYvec[i];
        if (accZvec[i] > meta.maxZ) meta.maxZ = accZvec[i];
        if (accZvec[i] < meta.minZ) meta.minZ = accZvec[i];
        meta.meanX += accXvec[i];
        meta.meanY += accYvec[i];
        meta.meanZ += accZvec[i];
    }
    
    meta.meanX /= (float)SAMPLES;
    meta.meanY /= (float)SAMPLES;
    meta.meanZ /= (float)SAMPLES;
    
    meta.sdX = meta.sdY = meta.sdZ = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
        meta.sdX += (accXvec[i] - meta.meanX) * (accXvec[i] - meta.meanX);
        meta.sdY += (accYvec[i] - meta.meanY) * (accYvec[i] - meta.meanY);
        meta.sdZ += (accZvec[i] - meta.meanZ) * (accZvec[i] - meta.meanZ);
    }
    meta.sdX = sqrtf(meta.sdX / (float)SAMPLES);
    meta.sdY = sqrtf(meta.sdY / (float)SAMPLES);
    meta.sdZ = sqrtf(meta.sdZ / (float)SAMPLES);
    
    // Update global stats arrays
    maxV[0] = meta.maxX; maxV[1] = meta.maxY; maxV[2] = meta.maxZ;
    minV[0] = meta.minX; minV[1] = meta.minY; minV[2] = meta.minZ;
    meanV[0] = meta.meanX; meanV[1] = meta.meanY; meanV[2] = meta.meanZ;
    sdV[0] = meta.sdX; sdV[1] = meta.sdY; sdV[2] = meta.sdZ;
    // Write header (v2)
    RecordHeaderV2 hdr;
    hdr.magic = REC_MAGIC_V2;
    hdr.version = 2;
    hdr.samples = (uint16_t)SAMPLES;
    hdr.fsNomHz = recordFsNomHz;
    hdr.reserved = 0;
    hdr.meta = meta;
    file.write((uint8_t*)&hdr, sizeof(RecordHeaderV2));

    // Write acceleration data (without timestamps to save space)
    file.write((uint8_t*)accXvec, SAMPLES * sizeof(float));
    file.write((uint8_t*)accYvec, SAMPLES * sizeof(float));
    file.write((uint8_t*)accZvec, SAMPLES * sizeof(float));

    // Write spectra
    file.write((uint8_t*)vRealX, SAMPLES * sizeof(float));
    file.write((uint8_t*)vRealY, SAMPLES * sizeof(float));
    file.write((uint8_t*)vRealZ, SAMPLES * sizeof(float));
    file.close();
    
    // Save storage metadata
    saveStorageMetadata();
    
    Serial.print("Saved record #");
    Serial.print(newestRecordIndex);
    Serial.print(" @ ");
    printDateTime(&currentRecordTime);
    Serial.print(" (");
    Serial.print(storedRecordCount);
    Serial.print("/");
    Serial.print(MAX_RECORDS);
    Serial.println(")");
    
    return true;
}

// Load a specific record from storage (by logical index 0 = oldest)
bool loadRecord(int logicalIndex) {
    if (!storageReady || logicalIndex < 0 || storedRecordCount == 0) return false;
    
    if (logicalIndex >= storedRecordCount) logicalIndex = storedRecordCount - 1;
    
    // Convert logical index to actual file index
    int fileIndex = (oldestRecordIndex + logicalIndex) % MAX_RECORDS;
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/rec%03d.bin", RECORDS_DIR, fileIndex);
    
    File file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.print("Failed to open: ");
        Serial.println(filename);
        return false;
    }
    
    // Read header (v2) or legacy metadata (v1.9.x)
    RecordMeta meta;
    uint32_t magic = 0;
    file.read((uint8_t*)&magic, sizeof(uint32_t));
    file.seek(0);

    if (magic == REC_MAGIC_V2) {
        RecordHeaderV2 hdr;
        file.read((uint8_t*)&hdr, sizeof(RecordHeaderV2));
        meta = hdr.meta;
        recordSamples = hdr.samples;
        recordFsNomHz = hdr.fsNomHz;
    } else {
        // Legacy v1.9.x record (fixed 1024 samples)
        file.read((uint8_t*)&meta, sizeof(RecordMeta));
        recordSamples = 1024;
        recordFsNomHz = 0;
    }

    
    samplingFrequency = meta.samplingFreq;

    // Reconstruct time vector (microseconds) for loaded records (timestamps are not stored to save space)
    if (samplingFrequency > 1e-3) {
        const double dt_us = 1000000.0 / (double)samplingFrequency;
        for (int i = 0; i < SAMPLES; i++) {
            t[i] = (long int)lround((double)i * dt_us);
        }
    } else {
        for (int i = 0; i < SAMPLES; i++) t[i] = 0;
    }
    xM = meta.peakX;
    yM = meta.peakY;
    zM = meta.peakZ;
    currentRecordTime = meta.rtcTime;  // Restore RTC time for display
    
    // Restore stats arrays
    maxV[0] = meta.maxX; maxV[1] = meta.maxY; maxV[2] = meta.maxZ;
    minV[0] = meta.minX; minV[1] = meta.minY; minV[2] = meta.minZ;
    meanV[0] = meta.meanX; meanV[1] = meta.meanY; meanV[2] = meta.meanZ;
    sdV[0] = meta.sdX; sdV[1] = meta.sdY; sdV[2] = meta.sdZ;
    
    // Read acceleration data
    file.read((uint8_t*)accXvec, SAMPLES * sizeof(float));
    file.read((uint8_t*)accYvec, SAMPLES * sizeof(float));
    file.read((uint8_t*)accZvec, SAMPLES * sizeof(float));
    
    // Read spectra
    file.read((uint8_t*)vRealX, SAMPLES * sizeof(float));
    file.read((uint8_t*)vRealY, SAMPLES * sizeof(float));
    file.read((uint8_t*)vRealZ, SAMPLES * sizeof(float));
    
    file.close();
    
    // Rebuild spectrogram from loaded data
    computeSpectrogramFromRecord();
    
    currentViewRecord = logicalIndex;
    
    Serial.print("Loaded record #");
    Serial.print(logicalIndex);
    Serial.print(" @ ");
    printDateTime(&currentRecordTime);
    Serial.println();
    
    return true;
}

// Load newest record
bool loadNewestRecord() {
    if (storedRecordCount > 0) {
        return loadRecord(storedRecordCount - 1);
    }
    return false;
}

// Delete all stored records
void clearAllRecords() {
    Serial.println("Clearing all records...");
    
    for (int i = 0; i < MAX_RECORDS; i++) {
        char filename[32];
        snprintf(filename, sizeof(filename), "%s/rec%03d.bin", RECORDS_DIR, i);
        if (LittleFS.exists(filename)) {
            LittleFS.remove(filename);
        }
    }
    
    storedRecordCount = 0;
    oldestRecordIndex = 0;
    newestRecordIndex = -1;
    currentViewRecord = -1;
    saveStorageMetadata();
    
    Serial.println("All records cleared");
}

// Print storage info to serial
void printStorageInfo() {
    Serial.println("=== Storage Info ===");
    Serial.print("Total bytes: ");
    Serial.println(LittleFS.totalBytes());
    Serial.print("Used bytes: ");
    Serial.println(LittleFS.usedBytes());
    Serial.print("Records stored: ");
    Serial.print(storedRecordCount);
    Serial.print("/");
    Serial.println(MAX_RECORDS);
    Serial.print("Oldest index: ");
    Serial.println(oldestRecordIndex);
    Serial.print("Newest index: ");
    Serial.println(newestRecordIndex);
    Serial.print("Currently viewing: ");
    if (currentViewRecord < 0) Serial.println("LIVE");
    else Serial.println(currentViewRecord);
    printCurrentTime();
    Serial.println("====================");
}

// Export all records via serial (for PC backup)
void exportAllRecordsSerial() {
    Serial.println("=== BEGIN EXPORT ===");
    Serial.print("RECORD_COUNT:");
    Serial.println(storedRecordCount);
    
    // Print current RTC
    RecordDateTime now;
    getRtcTime(&now);
    Serial.print("EXPORT_TIME:");
    printDateTime(&now);
    Serial.println();
    
    // Save current state
    int savedViewRecord = currentViewRecord;
    RecordDateTime savedRecordTime = currentRecordTime;
    
    for (int r = 0; r < storedRecordCount; r++) {
        loadRecord(r);
        
        Serial.print("=== RECORD ");
        Serial.print(r);
        Serial.println(" ===");
        
        // Print RTC timestamp
        Serial.print("TIMESTAMP:");
        printDateTime(&currentRecordTime);
        Serial.println();
        
        // Print metadata
        Serial.print("SAMPLING_FREQ:");
        Serial.println(samplingFrequency);
        Serial.print("SAMPLES:");
        Serial.println((int)SAMPLES);
        
        Serial.print("PEAK_FREQ:");
        Serial.print(xM, 3); Serial.print(",");
        Serial.print(yM, 3); Serial.print(",");
        Serial.println(zM, 3);
        
        // Print time-domain data
        Serial.println("TIME_DATA:");
        for (int i = 0; i < SAMPLES; i++) {
            Serial.print(accXvec[i], 5);
            Serial.print(",");
            Serial.print(accYvec[i], 5);
            Serial.print(",");
            Serial.println(accZvec[i], 5);
        }
        
        // Print spectra
        Serial.println("SPECTRUM:");
        for (int i = 0; i < (SAMPLES / 2); i++) {
            float f = (i * 1.0 * samplingFrequency) / (float)SAMPLES;
            Serial.print(f, 3);
            Serial.print(",");
            Serial.print(vRealX[i], 5);
            Serial.print(",");
            Serial.print(vRealY[i], 5);
            Serial.print(",");
            Serial.println(vRealZ[i], 5);
        }
    }
    
    Serial.println("=== END EXPORT ===");
    
    // Restore view state
    if (savedViewRecord >= 0) {
        loadRecord(savedViewRecord);
    }
    currentViewRecord = savedViewRecord;
    currentRecordTime = savedRecordTime;
}

// ============================================================================
// SPECTROGRAM FUNCTIONS
// ============================================================================

// Simple "jet-like" RGB colormap for value 0..255
uint16_t spectroColor(uint8_t v) {
    float x = v / 255.0f;
    uint8_t r = 0, g = 0, b = 0;

    if (x < 0.25f) {
        float t = x / 0.25f;
        r = 0;
        g = (uint8_t)(255 * t);
        b = 255;
    } else if (x < 0.5f) {
        float t = (x - 0.25f) / 0.25f;
        r = 0;
        g = 255;
        b = (uint8_t)(255 * (1.0f - t));
    } else if (x < 0.75f) {
        float t = (x - 0.5f) / 0.25f;
        r = (uint8_t)(255 * t);
        g = 255;
        b = 0;
    } else {
        float t = (x - 0.75f) / 0.25f;
        r = 255;
        g = (uint8_t)(255 * (1.0f - t));
        b = 0;
    }

    return M5.Display.color565(r, g, b);
}

// Build spectrogram from current record (total acceleration)
void computeSpectrogramFromRecord() {
    specSegments = 0;
    specLogMin   =  1e9f;
    specLogMax   = -1e9f;

    int overlapSamples = (SEG_LEN * SEG_OVERLAP_PCT) / 100;
    if (overlapSamples >= SEG_LEN) overlapSamples = SEG_LEN - 1;
    int hop = SEG_LEN - overlapSamples;

    for (int start = 0; start + SEG_LEN <= SAMPLES && specSegments < SEG_MAX; start += hop) {
        for (int i = 0; i < SEG_LEN; i++) {
            float ax = accXvec[start + i];
            float ay = accYvec[start + i];
            float az = accZvec[start + i];
            segReal[i] = sqrtf(ax * ax + ay * ay + az * az);
            segImag[i] = 0.0f;
        }

        FFT.dcRemoval(segReal, SEG_LEN);
        FFT.windowing(segReal, SEG_LEN, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(segReal, segImag, SEG_LEN, FFT_FORWARD);
        FFT.complexToMagnitude(segReal, segImag, SEG_LEN);

        // Normalize to a single-sided amplitude spectrum (consistent with main FFT plots).
        normalizeSpectrum(segReal, SEG_LEN);

        for (int k = 0; k < SPEC_BINS; k++) {
            float mag = segReal[k];
            if (mag < 1e-9f) mag = 1e-9f;
            float lv = log10f(mag);

            specMag[specSegments][k] = lv;

            if (lv < specLogMin) specLogMin = lv;
            if (lv > specLogMax) specLogMax = lv;
        }

        specSegments++;
    }

    if (specSegments == 0) {
        specSegments = 1;
        for (int k = 0; k < SPEC_BINS; k++) specMag[0][k] = 0.0f;
        specLogMin = 0.0f;
        specLogMax = 1.0f;
    }

    if (specLogMax - specLogMin < 1e-3f) {
        specLogMax = specLogMin + 1e-3f;
    }

    float span = specLogMax - specLogMin;
    if (span > 3.0f) {
        specLogMin = specLogMax - 3.0f;
    }
}

// ============================================================================
// SETUP
// ============================================================================


// Forward declarations (required for some Arduino toolchains)
void drawTriggerScreen();
void drawAcquisitionSetupScreen();
void drawStatisticsScreen();
void drawTimeSeriesScreen();
void drawEnvelopeLogDecScreen();
void drawSpectrumScreen();
void drawFftPeaksScreen();
void drawCorrelationScreen();
void drawSpectrogramScreen();
void drawStorageScreen();

void loadUserSettings();
void saveUserSettings();
void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    cfg.clear_display = true;
    cfg.internal_imu = true;
    cfg.internal_rtc = true;
    cfg.internal_mic = false;
    cfg.internal_spk = false;
    // cfg.output_power = true; // default=true on AXP192 devices
    M5.begin(cfg);
    M5.update();

    if (!M5.Imu.isEnabled()) {
        M5.Imu.begin();
    }
    pinMode(M5_LED, OUTPUT);
    pinMode(CALIB_T_PIN, OUTPUT);
    pinMode(EXT_TRIG_PIN, INPUT);


    // ADC setup (used by external trigger input / general ADC reads)
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
#endif

    // BtnB held at boot = external trigger mode
    if (M5.BtnB.isPressed()) isExtTriged = true;
    else isExtTriged = false;

    digitalWrite(M5_LED, LOW);
    isLogging = false;
    isTrig    = false;
    count     = 0;
    digitalWrite(CALIB_T_PIN, LOW);
    prevSampleUs = micros();

    loadUserSettings();

    // Initialize storage
    storageReady = initStorage();

    // Print current RTC time
    printCurrentTime();
    // Startup Screen
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    M5.Display.setTextFont(2);
    
    runTiltWalkIntro(10000);
    th = 17;

    M5.Display.setCursor(5, 1);
    M5.Display.print("Vibration analyser");
    M5.Display.setCursor(5, th);
    M5.Display.print("ver. 1.14 (M5Unified)");
    M5.Display.setCursor(5, 2 * th);
    M5.Display.print("Vladimir Divic, 2025");
    M5.Display.setCursor(5, 3 * th);
    M5.Display.print("Serial: USB");

    // Show current RTC time
    M5.Display.setCursor(5, 4 * th);
    M5.Display.print("RTC: ");
    RecordDateTime now;
    getRtcTime(&now);
    char timeBuf[24];
    formatDateTime(&now, timeBuf, sizeof(timeBuf));
    M5.Display.print(timeBuf);
    
    M5.Display.setCursor(5, 5 * th);
    if (storageReady) {
        M5.Display.print("Storage: ");
        M5.Display.print(storedRecordCount);
        M5.Display.print("/");
        M5.Display.print(MAX_RECORDS);
        M5.Display.print(" rec");
    } else {
        M5.Display.setTextColor(RED);
        M5.Display.print("Storage FAILED!");
        M5.Display.setTextColor(WHITE);
    }

    M5.Display.setCursor(5, 6 * th);
    M5.Display.print("BtnA-rec BtnB-screen");
    M5.Display.setCursor(5, 7 * th);
    if (isExtTriged) M5.Display.print("Ext trig mode");
    else             M5.Display.print("Self-trigger mode");

    curDisp = 0;
    timeout = extendT + millis();
    
    // Print help to serial
    Serial.println("=== Vibration Analyzer v1.14 ===");
    Serial.println("Serial commands: R=arm, A=timeseries, F=fft, S=summary, Cxx=select rec, T=time, E=export, I=info, H=help");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

constexpr uint32_t LONG_PRESS_MS = 700;  // long-press threshold for BtnA/BtnB

void loop() {
    M5.update();

// --- BtnA handling ---
// Screen 1: edit trigger settings (short=change value, long=next item)
// Screen 2: acquisition setup (short=change value, long=next item)
static bool btnALongHandledTrig = false;
static bool btnALongHandledSetup = false;

if (curDisp == 1) {
    if (M5.BtnA.pressedFor(LONG_PRESS_MS) && !btnALongHandledTrig) {
        btnALongHandledTrig = true;
        timeout = extendT + millis();

        trigMenuItem = nextVisibleTriggerMenuItem(trigMenuItem);
        drawTriggerScreen();
    }

    if (M5.BtnA.wasReleased()) {
        timeout = extendT + millis();

        if (!btnALongHandledTrig) {
            // Short press: increment selected setting
            if (trigMenuItem == 0) {
                // Mode
                trigMode = (TriggerMode)((((uint8_t)trigMode) + 1) % 3);

                // Keep menu item valid for the new mode
                if (!isTriggerMenuItemVisible(trigMenuItem)) trigMenuItem = 0;

                // Disarm any pending recording when changing modes
                if (isLogging && !isTrig) disarmRecording();
            }
            else if (trigMenuItem == 1) {
                // Param: Delay (TRIG_DELAY) or Acc threshold (TRIG_ACCEL)
                if (trigMode == TRIG_DELAY) {
                    delayIndex = (delayIndex + 1) % 3;
                } else if (trigMode == TRIG_ACCEL) {
                    accelTrigThresholdG += 0.1f;
                    if (accelTrigThresholdG > 2.0001f) accelTrigThresholdG = 1.0f;
                }
            }
            saveUserSettings();
            drawTriggerScreen();
        }

        btnALongHandledTrig = false;
    }
}
else if (curDisp == 2) {
    if (M5.BtnA.pressedFor(LONG_PRESS_MS) && !btnALongHandledSetup) {
        btnALongHandledSetup = true;
        timeout = extendT + millis();

        setupMenuItem = (setupMenuItem + 1) % SETUP_MENU_ITEMS;
        drawAcquisitionSetupScreen();
    }

    if (M5.BtnA.wasReleased()) {
        timeout = extendT + millis();

        if (!btnALongHandledSetup) {
            if (setupMenuItem == 0) {
                sampleCountIndex = (sampleCountIndex + 1) % 3;
            } else if (setupMenuItem == 1) {
                samplingRateIndex = (samplingRateIndex + 1) % 5;
            }
            applyAcquisitionSettings();
            saveUserSettings();

            // Disarm any pending recording when changing acquisition settings
            if (isLogging && !isTrig) disarmRecording();

            drawAcquisitionSetupScreen();
        }

        btnALongHandledSetup = false;
    }
}
else {
    // All other screens: BtnA is used to browse (Screen 10) or arm recording (Screens 3-9)
    static bool btnAHandled = false;

    if (M5.BtnA.isPressed() && !btnAHandled) {
        btnAHandled = true;
        timeout = extendT + millis();

        if (curDisp == 10 && storedRecordCount > 0) {
            // On storage screen (Screen 10): browse through stored records
            if (currentViewRecord < 0) {
                // Start with newest
                currentViewRecord = storedRecordCount - 1;
            } else {
                currentViewRecord--;
                if (currentViewRecord < 0) currentViewRecord = storedRecordCount - 1;
            }
            loadRecord(currentViewRecord);

            // Refresh storage screen to show new selection
            drawStorageScreen();
        } else {
            // Arm recording according to trigger settings (or cancel if already armed)
            armRecording();
        }
    }

    if (M5.BtnA.wasReleased()) {
        btnAHandled = false;
    }
}


    // Process serial input (including time setting)
    processSerialInput();

    if (redrawRequested) {
        redrawRequested = false;
        // Redraw current screen to reflect serial-driven state changes (e.g., Cxx record selection)
        if (curDisp == 1) drawTriggerScreen();
        else if (curDisp == 2) drawAcquisitionSetupScreen();
        else if (curDisp == 3) drawStatisticsScreen();
        else if (curDisp == 4) drawTimeSeriesScreen();
        else if (curDisp == 5) drawEnvelopeLogDecScreen();
        else if (curDisp == 6) drawSpectrumScreen();
        else if (curDisp == 7) drawFftPeaksScreen();
        else if (curDisp == 8) drawCorrelationScreen();
        else if (curDisp == 9) drawSpectrogramScreen();
        else if (curDisp == 10) drawStorageScreen();
    }
    // Trigger state machine (external + internal trigger modes)
    updateTriggerState();
    updateArmedIndicator();

    // --- Sampling finished: FFT, stats, spectrogram, save ---
    if (count == SAMPLES) {
        digitalWrite(CALIB_T_PIN, LOW);
        digitalWrite(M5_LED, LOW);

        long  dt  = 0;
        float dts = 0.0f;
        for (int i = 0; i < SAMPLES - 1; i++) {
            dt += t[i + 1] - t[i];
        }

        dts = ((float)dt / (float)(SAMPLES - 1)) / 1000000.0f;
        samplingFrequency = 1.0f / dts;

        // ---------- X axis FFT ----------
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = accXvec[i];
            vImag[i] = 0.0f;
        }
        FFT.dcRemoval(vReal, SAMPLES);
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);
        normalizeSpectrum(vReal, SAMPLES);
        xM = FFT.majorPeak(vReal, SAMPLES, samplingFrequency);
        for (int i = 0; i < SAMPLES; i++) {
            vRealX[i] = vReal[i];
        }

        // ---------- Y axis FFT ----------
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = accYvec[i];
            vImag[i] = 0.0f;
        }
        FFT.dcRemoval(vReal, SAMPLES);
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);
        normalizeSpectrum(vReal, SAMPLES);
        yM = FFT.majorPeak(vReal, SAMPLES, samplingFrequency);
        for (int i = 0; i < SAMPLES; i++) {
            vRealY[i] = vReal[i];
        }

        // ---------- Z axis FFT ----------
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = accZvec[i];
            vImag[i] = 0.0f;
        }
        FFT.dcRemoval(vReal, SAMPLES);
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);
        normalizeSpectrum(vReal, SAMPLES);
        zM = FFT.majorPeak(vReal, SAMPLES, samplingFrequency);
        for (int i = 0; i < SAMPLES; i++) {
            vRealZ[i] = vReal[i];
        }

        // Build spectrogram
        computeSpectrogramFromRecord();

        // *** AUTO-SAVE TO PERSISTENT STORAGE ***
        if (storageReady) {
            saveCurrentRecord();
        }
        currentViewRecord = -1;  // Mark as viewing live/newest data

        count     = 0;
        isLogging = false;
        isTrig    = false;
    }

    // --- BtnB short/long press handling ---
    static bool btnBLongHandled = false;
    // Long press action
    if (M5.BtnB.pressedFor(LONG_PRESS_MS) && !btnBLongHandled) {
        btnBLongHandled = true;

        if (curDisp == 10) {
            // Long press on storage screen = clear all records
            clearAllRecords();
            drawStorageScreen();
        }
        else if (curDisp == 3) {
            // Statistics dump
            Serial.println("Statistics:");
            Serial.print("Timestamp: ");
            printDateTime(&currentRecordTime);
            Serial.println();
            Serial.println("Value\tX\tY\tZ");
            Serial.print("MAX:\t");
            for(int i=0;i<3;i++) {Serial.print(maxV[i],3); Serial.print("\t");}
            Serial.println("");
            Serial.print("MEAN:\t");
            for(int i=0;i<3;i++) {Serial.print(meanV[i],3); Serial.print("\t");}
            Serial.println("");
            Serial.print("SD:\t");
            for(int i=0;i<3;i++) {Serial.print(sdV[i],3); Serial.print("\t");}
            Serial.println("");
            Serial.print("MIN:\t");
            for(int i=0;i<3;i++) {Serial.print(minV[i],3); Serial.print("\t");}
            Serial.println("");
            Serial.print("F1[Hz]:\t");
            Serial.print(xM,3); Serial.print("\t");
            Serial.print(yM,3); Serial.print("\t");
            Serial.print(zM,3); Serial.println("\t");
        }
        else if ((curDisp == 4) || (curDisp == 5) || (curDisp == 7)) {
            // Time series / correlation -> dump time-domain data
            Serial.print("# Timestamp: ");
            printDateTime(&currentRecordTime);
            Serial.println();
            for (int i = 0; i < SAMPLES; i++) {
                Serial.print(t[i]);
                Serial.print(" ");
                Serial.print(accXvec[i], 4);
                Serial.print(" ");
                Serial.print(accYvec[i], 4);
                Serial.print(" ");
                Serial.print(accZvec[i], 4);
                Serial.println("");
            }
        }
        else if ((curDisp == 6) || (curDisp == 8)) {
            // Spectrum / spectrogram -> dump spectra
            Serial.print("# Timestamp: ");
            printDateTime(&currentRecordTime);
            Serial.println();
            for (int i = 0; i < (SAMPLES / 2); i++) {
                float f = (i * 1.0 * samplingFrequency) / (float)SAMPLES;
                Serial.print(f);
                Serial.print(" ");
                Serial.print(vRealX[i]);
                Serial.print(" ");
                Serial.print(vRealY[i]);
                Serial.print(" ");
                Serial.println(vRealZ[i]);
            }
        }
    }

    // On release: short press = change screen
    if (M5.BtnB.wasReleased()) {
        timeout = extendT + millis();

        if (!btnBLongHandled) {
            // SHORT PRESS → change screen
            curDisp++;
            if (curDisp > 10) curDisp = 1;

            // --- Draw screens ---
            if (curDisp == 1) {
                drawTriggerScreen();
            }

            if (curDisp == 2) {
                drawAcquisitionSetupScreen();
            }

            if (curDisp == 3) {
                drawStatisticsScreen();
            }

            if (curDisp == 4) {
                drawTimeSeriesScreen();
            }

            if (curDisp == 5) {
                drawEnvelopeLogDecScreen();
            }

            if (curDisp == 6) {
                drawSpectrumScreen();
            }

            if (curDisp == 7) {
                drawFftPeaksScreen();
            }

            if (curDisp == 8) {
                drawCorrelationScreen();
            }

            if (curDisp == 9) {
                drawSpectrogramScreen();
            }

            if (curDisp == 10) {
                drawStorageScreen();
            }
        }

        btnBLongHandled = false;
    }

    // --- Sampling logic ---
    if (isTrig && (count < SAMPLES)) {
        const uint32_t nowUs = micros();
        if ((uint32_t)(nowUs - prevSampleUs) >= TTNS_US) {
            prevSampleUs = nowUs;

            digitalWrite(M5_LED, HIGH);
            digitalWrite(CALIB_T_PIN, !digitalRead(CALIB_T_PIN));

            M5.Imu.getAccel(&accX, &accY, &accZ);

            if (count == 0) {
                // *** CAPTURE RTC TIME AT START OF RECORDING ***
                getRtcTime(&currentRecordTime);

                t0       = nowUs;
                t[count] = 0;
            } else {
                t[count] = nowUs - t0;
            }

            accXvec[count] = accX;
            accYvec[count] = accY;
            accZvec[count] = accZ;

            count++;
        }
    }

    // Power off on timeout when on battery
    if ((millis() > timeout) && (M5.Power.getBatteryCurrent() < -1)) {
        M5.Power.powerOff();
    }
}

// ============================================================================
// SCREEN DRAWING FUNCTIONS

// ============================================================================

void drawStatisticsScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE);
    M5.Display.setRotation(1);

    float minX =  9999.0;
    float maxX = -9999.0;
    float minY =  9999.0;
    float maxY = -9999.0;
    float minZ =  9999.0;
    float maxZ = -9999.0;
    float maxXYZ;
    float minXYZ;
    float meanX = 0.0f;
    float meanY = 0.0f;
    float meanZ = 0.0f;
    float sdX   = 0.0f;
    float sdY   = 0.0f;
    float sdZ   = 0.0f;

    for (int i = 0; i < SAMPLES; i++) {
        if (accXvec[i] > maxX) maxX = accXvec[i];
        if (accXvec[i] < minX) minX = accXvec[i];
        if (accYvec[i] > maxY) maxY = accYvec[i];
        if (accYvec[i] < minY) minY = accYvec[i];
        if (accZvec[i] > maxZ) maxZ = accZvec[i];
        if (accZvec[i] < minZ) minZ = accZvec[i];
        maxXYZ = max(max(maxX, maxY), maxZ) * 100;
        minXYZ = min(min(minX, minY), minZ) * 100;
        meanX += accXvec[i];
        meanY += accYvec[i];
        meanZ += accZvec[i];
    }

    meanX = meanX / (float)SAMPLES;
    meanY = meanY / (float)SAMPLES;
    meanZ = meanZ / (float)SAMPLES;

    for (int i = 0; i < SAMPLES; i++) {
        sdX += (accXvec[i] - meanX) * (accXvec[i] - meanX);
        sdY += (accYvec[i] - meanY) * (accYvec[i] - meanY);
        sdZ += (accZvec[i] - meanZ) * (accZvec[i] - meanZ);
    }
    sdX = sqrtf(sdX / (float)SAMPLES);
    sdY = sqrtf(sdY / (float)SAMPLES);
    sdZ = sqrtf(sdZ / (float)SAMPLES);

    maxV[0] = maxX; maxV[1] = maxY; maxV[2] = maxZ;
    meanV[0] = meanX; meanV[1] = meanY; meanV[2] = meanZ;
    sdV[0] = sdX; sdV[1] = sdY; sdV[2] = sdZ;
    minV[0] = minX; minV[1] = minY; minV[2] = minZ;

    // Row 1: Timestamp and record indicator
    M5.Display.setCursor(5, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    M5.Display.print(timeBuf);
    
    M5.Display.setCursor(100, 1);
    M5.Display.print("Fs:");
    M5.Display.print((int)samplingFrequency);
    M5.Display.print("Hz");
    
    // Record indicator
    M5.Display.setCursor(180, 1);
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
        M5.Display.print("LIVE");
    } else {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print("#");
        M5.Display.print(currentViewRecord);
    }
    M5.Display.setTextColor(WHITE);
    
    // Row 2
    M5.Display.setCursor(5, th);
    M5.Display.print("STATISTICS  N=");
    M5.Display.print(SAMPLES);
    
    // Row 3 - headers
    M5.Display.setCursor(60, th * 2);
    M5.Display.setTextColor(RED);
    M5.Display.print("X");
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 2);
    M5.Display.print("Y");
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 2);
    M5.Display.print("Z");
    
    // Row 4 - MAX
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, th * 3);
    M5.Display.print("MAX:");
    M5.Display.setTextColor(RED);
    M5.Display.setCursor(60, th * 3);
    M5.Display.print(maxX, 3);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 3);
    M5.Display.print(maxY, 3);
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 3);
    M5.Display.print(maxZ, 3);
    
    // Row 5 - MEAN
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, th * 4);
    M5.Display.print("MEAN:");
    M5.Display.setTextColor(RED);
    M5.Display.setCursor(60, th * 4);
    M5.Display.print(meanX, 3);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 4);
    M5.Display.print(meanY, 3);
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 4);
    M5.Display.print(meanZ, 3);
    
    // Row 6 - SD
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, th * 5);
    M5.Display.print("SD:");
    M5.Display.setTextColor(RED);
    M5.Display.setCursor(60, th * 5);
    M5.Display.print(sdX, 3);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 5);
    M5.Display.print(sdY, 3);
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 5);
    M5.Display.print(sdZ, 3);
    
    // Row 7 - MIN
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, th * 6);
    M5.Display.print("MIN:");
    M5.Display.setTextColor(RED);
    M5.Display.setCursor(60, th * 6);
    M5.Display.print(minX, 3);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 6);
    M5.Display.print(minY, 3);
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 6);
    M5.Display.print(minZ, 3);
    
    // Row 8 - PEAK
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, th * 7);
    M5.Display.print("PEAK/Hz:");
    M5.Display.setTextColor(RED);
    M5.Display.setCursor(60, th * 7);
    M5.Display.print(xM, 3);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(120, th * 7);
    M5.Display.print(yM, 3);
    M5.Display.setTextColor(BLUE);
    M5.Display.setCursor(180, th * 7);
    M5.Display.print(zM, 3);

    M5.Display.setTextColor(WHITE);

    drawArmedIndicator(isNonImmediateArmed());
}

void drawAcquisitionSetupScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE);

    // Compact layout for 135px height screen
    const int lh = 14;
    const int xLabel = 5;
    const int xVal   = 125;

    // Title
    M5.Display.setCursor(xLabel, 0);
    M5.Display.setTextColor(CYAN);
    M5.Display.print("ACQUISITION SETUP");
    M5.Display.setTextColor(WHITE);

    auto drawLine = [&](int yy, const char* label, const String& value, bool selected) {
        if (selected) {
            M5.Display.drawRect(0, yy - 2, DW, lh + 3, YELLOW);
        }
        M5.Display.setCursor(xLabel, yy);
        M5.Display.setTextColor(WHITE);
        M5.Display.print(label);

        M5.Display.setCursor(xVal, yy);
        M5.Display.setTextColor(CYAN);
        M5.Display.print(value);
        M5.Display.setTextColor(WHITE);
    };

    int y = lh + 2;

    drawLine(y, "Samples:", String(sampleCountSetting), (setupMenuItem == 0));
    y += lh;

    drawLine(y, "Fs:", String(samplingRateSettingHz) + " Hz", (setupMenuItem == 1));
    y += lh;

    // Derived duration and TTNS_US (read-only)
    float durS = (samplingRateSettingHz > 0) ? ((float)sampleCountSetting / (float)samplingRateSettingHz) : 0.0f;

    M5.Display.setCursor(xLabel, y + 2);
    M5.Display.setTextColor(WHITE);
    M5.Display.print("Dur: ");
    M5.Display.setTextColor(CYAN);
    M5.Display.print(durS, 3);
    M5.Display.print(" s");
    M5.Display.setTextColor(WHITE);

    M5.Display.setCursor(xLabel, y + lh + 2);
    M5.Display.setTextColor(WHITE);
    M5.Display.print("TTNS: ");
    M5.Display.setTextColor(CYAN);
    M5.Display.print(TTNS_US);
    M5.Display.print(" us");
    M5.Display.setTextColor(WHITE);

    // Footer hints
    int yFooter = DH - lh;
    M5.Display.setCursor(xLabel, yFooter);
    M5.Display.setTextColor(CYAN);
    M5.Display.print("BtnA:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" edit   ");
    M5.Display.setTextColor(CYAN);
    M5.Display.print("BtnB:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" next screen");

    drawArmedIndicator(isNonImmediateArmed());
}


void drawTimeSeriesScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    
    float minX =  9999.0;
    float maxX = -9999.0;
    float minY =  9999.0;
    float maxY = -9999.0;
    float minZ =  9999.0;
    float maxZ = -9999.0;
    float maxXYZ;
    float minXYZ;
    
    for (int i = 0; i < SAMPLES; i++) {
        if (accXvec[i] > maxX) maxX = accXvec[i];
        if (accXvec[i] < minX) minX = accXvec[i];
        if (accYvec[i] > maxY) maxY = accYvec[i];
        if (accYvec[i] < minY) minY = accYvec[i];
        if (accZvec[i] > maxZ) maxZ = accZvec[i];
        if (accZvec[i] < minZ) minZ = accZvec[i];
        maxXYZ = max(max(maxX, maxY), maxZ) * 100;
        minXYZ = min(min(minX, minY), minZ) * 100;
    }
    
    M5.Display.fillScreen(TFT_BLACK);
    for (int i = 1; i < SAMPLES; i++) {
        int yp1 = map(accXvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        int xp1 = map(i - 1, 0, SAMPLES, 0, DW - 1);
        int yp2 = map(accXvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        int xp2 = map(i, 0, SAMPLES, 0, DW - 1);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_RED);
        yp1 = map(accYvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        yp2 = map(accYvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_GREEN);
        yp1 = map(accZvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        yp2 = map(accZvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_BLUE);
    }
    
    M5.Display.setCursor(2, 2);
    M5.Display.print(maxXYZ / 100);
    M5.Display.print("g");
    M5.Display.setCursor(2, 72);
    M5.Display.print(minXYZ / 100);
    M5.Display.print("g");
    
    int yp1 = map(0, minXYZ, maxXYZ, DH - 1, 0);
    M5.Display.drawLine(0, yp1, DW - 1, yp1, TFT_WHITE);
    M5.Display.drawLine(0, 0, 0, DH - 1, TFT_WHITE);

    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(80, 1);
    M5.Display.print("Timeseries");
    
    // Timestamp
    M5.Display.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
    } else {
        M5.Display.setTextColor(YELLOW);
    }
    M5.Display.print(timeBuf);
    M5.Display.setTextColor(WHITE);

    drawArmedIndicator(isNonImmediateArmed());
}



// ============================================================================
// Screen 5: Envelope & Logarithmic Decrement
// - Envelope is drawn by connecting local maxima (above DC) for each axis.
// - Fit: y(t) = C + A * exp(-b*t) through maxima (offset C included).
// - If peaks < 6 -> no fit. If R2 > 0.80 -> compute log decrement: delta = b * Tpeak.
// ============================================================================

#ifndef TFT_ORANGE
  #define TFT_ORANGE 0xFD20
#endif

static const int MAX_ENV_PEAKS = 200;

struct EnvFitResult {
    int   peakCount = 0;
    bool  fitTried  = false;
    bool  fitOK     = false;
    float A = 0.0f;
    float b = 0.0f;
    float C = 0.0f;
    float R2 = -1.0f;
    float logDec = 0.0f;
};

static int findLocalMaximaAboveDC(const float* sig, int len, float dc, float fs,
                                 uint16_t* idxOut, float* valOut, int maxOut) {
    if (len < 8) return 0;

    // dynamic threshold based on zero-mean amplitude
    float maxAbs = 0.0f;
    for (int i = 0; i < len; i++) {
        float v = sig[i] - dc;
        float a = fabsf(v);
        if (a > maxAbs) maxAbs = a;
    }
    if (maxAbs < 1e-6f) return 0;

    const float thr = maxAbs * 0.05f;  // 5% of max zero-mean amplitude
    const int minGap = max(2, (int)(fs * 0.005f)); // >=5 ms between peaks

    int count = 0;
    int lastIdx = -999999;

    for (int i = 2; i < len - 2; i++) {
        float v  = sig[i]   - dc;
        if (v <= thr) continue; // only positive peaks above threshold

        float vm1 = sig[i-1] - dc;
        float vp1 = sig[i+1] - dc;
        float vm2 = sig[i-2] - dc;
        float vp2 = sig[i+2] - dc;

        // local maximum (robust over ±2 samples)
        if (v >= vm1 && v >= vp1 && v >= vm2 && v >= vp2) {
            if (count == 0 || (i - lastIdx) >= minGap) {
                idxOut[count] = (uint16_t)i;
                valOut[count] = sig[i]; // store RAW value (with offset)
                lastIdx = i;
                count++;
                if (count >= maxOut) break;
            }
        }
    }
    return count;
}

static bool fitExpDecayWithOffset(const uint16_t* idx, const float* y, int n, float fs,
                                  float* outA, float* outb, float* outC, float* outR2) {
    if (n < 6) return false;

    int m = n;
    if (m > 60) m = 60;  // fit on first peaks (typically most informative)

    float minY = y[0], maxY = y[0];
    for (int i = 0; i < m; i++) {
        if (y[i] < minY) minY = y[i];
        if (y[i] > maxY) maxY = y[i];
    }

    float meanY = 0.0f;
    for (int i = 0; i < m; i++) meanY += y[i];
    meanY /= (float)m;

    float SST = 0.0f;
    for (int i = 0; i < m; i++) {
        float d = y[i] - meanY;
        SST += d * d;
    }
    if (SST < 1e-12f) SST = 1e-12f;

    const float span = maxY - minY;
    if (span < 1e-6f) return false;

    // Offset search range (must be < minY)
    float Cmax = minY - 1e-6f;
    float Cmin = minY - 2.0f * span; // allow baseline below smallest peak

    const int STEPS = 40;

    float bestSSE = 1e30f;
    float bestA = 0.0f, bestb = 0.0f, bestC = 0.0f;

    for (int s = 0; s <= STEPS; s++) {
        float C = Cmin + (Cmax - Cmin) * ((float)s / (float)STEPS);

        // linear regression on ln(y - C) = ln(A) - b*t
        float sumT = 0.0f, sumL = 0.0f, sumT2 = 0.0f, sumTL = 0.0f;
        bool ok = true;

        for (int i = 0; i < m; i++) {
            float yi = y[i] - C;
            if (yi <= 1e-6f) { ok = false; break; }
            float ti = (float)idx[i] / fs;
            float li = logf(yi);
            sumT  += ti;
            sumL  += li;
            sumT2 += ti * ti;
            sumTL += ti * li;
        }

        if (!ok) continue;

        float denom = (float)m * sumT2 - sumT * sumT;
        if (fabsf(denom) < 1e-9f) continue;

        float slope = ((float)m * sumTL - sumT * sumL) / denom;
        float intercept = (sumL - slope * sumT) / (float)m;

        float b = -slope;
        if (!(b > 0.0f) || !isfinite(b)) continue;

        float A = expf(intercept);
        if (!isfinite(A) || A <= 0.0f) continue;

        float SSE = 0.0f;
        for (int i = 0; i < m; i++) {
            float ti = (float)idx[i] / fs;
            float yhat = C + A * expf(-b * ti);
            float d = y[i] - yhat;
            SSE += d * d;
        }

        if (SSE < bestSSE) {
            bestSSE = SSE;
            bestA = A;
            bestb = b;
            bestC = C;
        }
    }

    if (bestSSE > 1e29f) return false;

    float R2 = 1.0f - (bestSSE / SST);
    if (!isfinite(R2)) R2 = -1.0f;

    *outA = bestA;
    *outb = bestb;
    *outC = bestC;
    *outR2 = R2;
    return true;
}

static float estimatePeakPeriod(const uint16_t* idx, int n, float fs) {
    if (n < 2) return 0.0f;
    int m = n;
    if (m > 60) m = 60;

    float sum = 0.0f;
    int cnt = 0;
    for (int i = 1; i < m; i++) {
        sum += (float)(idx[i] - idx[i-1]);
        cnt++;
    }
    if (cnt <= 0) return 0.0f;
    float avgSamp = sum / (float)cnt;
    return avgSamp / fs;
}

static void drawPeakPolyline(const uint16_t* idx, const float* y, int n,
                             float yMin, float yMax,
                             int x0, int x1, int yTop, int yBot,
                             uint16_t color) {
    if (n < 2) return;
    if (fabsf(yMax - yMin) < 1e-9f) return;

    auto mapX = [&](int sampleIdx) -> int {
        float u = (float)sampleIdx / (float)(SAMPLES - 1);
        int xp = x0 + (int)(u * (float)(x1 - x0));
        if (xp < x0) xp = x0;
        if (xp > x1) xp = x1;
        return xp;
    };
    auto mapY = [&](float v) -> int {
        float u = (v - yMin) / (yMax - yMin);
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;
        int yp = yBot - (int)(u * (float)(yBot - yTop));
        if (yp < yTop) yp = yTop;
        if (yp > yBot) yp = yBot;
        return yp;
    };

    int px = mapX(idx[0]);
    int py = mapY(y[0]);
    for (int i = 1; i < n; i++) {
        int cx = mapX(idx[i]);
        int cy = mapY(y[i]);
        M5.Display.drawLine(px, py, cx, cy, color);
        px = cx; py = cy;
    }
}

static void drawFitCurve(float A, float b, float C, float fs,
                         float yMin, float yMax,
                         int x0, int x1, int yTop, int yBot,
                         uint16_t color) {
    if (!(A > 0.0f) || !(b > 0.0f)) return;
    if (fabsf(yMax - yMin) < 1e-9f) return;

    auto mapX = [&](int sampleIdx) -> int {
        float u = (float)sampleIdx / (float)(SAMPLES - 1);
        int xp = x0 + (int)(u * (float)(x1 - x0));
        if (xp < x0) xp = x0;
        if (xp > x1) xp = x1;
        return xp;
    };
    auto mapY = [&](float v) -> int {
        float u = (v - yMin) / (yMax - yMin);
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;
        int yp = yBot - (int)(u * (float)(yBot - yTop));
        if (yp < yTop) yp = yTop;
        if (yp > yBot) yp = yBot;
        return yp;
    };

    // dotted curve (pixels)
    const int step = max(2, SAMPLES / 240);
    for (int i = 0; i < SAMPLES; i += step) {
        float t = (float)i / fs;
        float y = C + A * expf(-b * t);
        int xp = mapX(i);
        int yp = mapY(y);
        M5.Display.drawPixel(xp, yp, color);
    }
}

void drawEnvelopeLogDecScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);

    const int yTop = 12;
    const int yBot = 92;
    const int x0 = 0;
    const int x1 = DW - 1;

    // Compute global min/max for plotting (raw signal)
    float minV =  1e9f;
    float maxV = -1e9f;

    for (int i = 0; i < SAMPLES; i++) {
        float ax = accXvec[i];
        float ay = accYvec[i];
        float az = accZvec[i];
        if (ax < minV) minV = ax;
        if (ay < minV) minV = ay;
        if (az < minV) minV = az;
        if (ax > maxV) maxV = ax;
        if (ay > maxV) maxV = ay;
        if (az > maxV) maxV = az;
    }
    float span = maxV - minV;
    if (span < 1e-6f) { span = 1.0f; minV -= 0.5f; maxV += 0.5f; }
    // small padding
    minV -= 0.05f * span;
    maxV += 0.05f * span;

    // Draw raw traces (thin)
    auto mapX = [&](int i) -> int {
        float u = (float)i / (float)(SAMPLES - 1);
        int xp = x0 + (int)(u * (float)(x1 - x0));
        if (xp < x0) xp = x0;
        if (xp > x1) xp = x1;
        return xp;
    };
    auto mapY = [&](float v) -> int {
        float u = (v - minV) / (maxV - minV);
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;
        int yp = yBot - (int)(u * (float)(yBot - yTop));
        if (yp < yTop) yp = yTop;
        if (yp > yBot) yp = yBot;
        return yp;
    };

    for (int i = 1; i < SAMPLES; i++) {
        int x1p = mapX(i - 1);
        int x2p = mapX(i);

        int yx1 = mapY(accXvec[i - 1]);
        int yx2 = mapY(accXvec[i]);
        M5.Display.drawLine(x1p, yx1, x2p, yx2, TFT_RED);

        int yy1 = mapY(accYvec[i - 1]);
        int yy2 = mapY(accYvec[i]);
        M5.Display.drawLine(x1p, yy1, x2p, yy2, TFT_GREEN);

        int yz1 = mapY(accZvec[i - 1]);
        int yz2 = mapY(accZvec[i]);
        M5.Display.drawLine(x1p, yz1, x2p, yz2, TFT_BLUE);
    }

    // Axes lines
    int yZero = mapY(0.0f);
    if (yZero >= yTop && yZero <= yBot) {
        M5.Display.drawLine(x0, yZero, x1, yZero, TFT_WHITE);
    }
    M5.Display.drawLine(0, yTop, 0, yBot, TFT_WHITE);

    // Find DC offsets for peak detection (mean)
    float dcX = 0.0f, dcY = 0.0f, dcZ = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
        dcX += accXvec[i];
        dcY += accYvec[i];
        dcZ += accZvec[i];
    }
    dcX /= (float)SAMPLES;
    dcY /= (float)SAMPLES;
    dcZ /= (float)SAMPLES;

    // Peak arrays
    static uint16_t idxX[MAX_ENV_PEAKS];
    static uint16_t idxY[MAX_ENV_PEAKS];
    static uint16_t idxZ[MAX_ENV_PEAKS];
    static float    pkX[MAX_ENV_PEAKS];
    static float    pkY[MAX_ENV_PEAKS];
    static float    pkZ[MAX_ENV_PEAKS];

    EnvFitResult fx, fy, fz;

    const float fs = (samplingFrequency > 1e-3f) ? samplingFrequency : (float)samplingRateSettingHz;

    fx.peakCount = findLocalMaximaAboveDC(accXvec, SAMPLES, dcX, fs, idxX, pkX, MAX_ENV_PEAKS);
    fy.peakCount = findLocalMaximaAboveDC(accYvec, SAMPLES, dcY, fs, idxY, pkY, MAX_ENV_PEAKS);
    fz.peakCount = findLocalMaximaAboveDC(accZvec, SAMPLES, dcZ, fs, idxZ, pkZ, MAX_ENV_PEAKS);

    // Fit each axis
    for (int axis = 0; axis < 3; axis++) {
        EnvFitResult* fr = (axis == 0) ? &fx : (axis == 1) ? &fy : &fz;
        const uint16_t* idx = (axis == 0) ? idxX : (axis == 1) ? idxY : idxZ;
        const float*    pk  = (axis == 0) ? pkX  : (axis == 1) ? pkY  : pkZ;

        if (fr->peakCount >= 6) {
            fr->fitTried = true;
            float A, b, C, R2;
            if (fitExpDecayWithOffset(idx, pk, fr->peakCount, fs, &A, &b, &C, &R2)) {
                fr->fitOK = true;
                fr->A = A;
                fr->b = b;
                fr->C = C;
                fr->R2 = R2;

                if (R2 > 0.80f) {
                    float T = estimatePeakPeriod(idx, fr->peakCount, fs);
                    if (T > 1e-6f) {
                        fr->logDec = b * T;
                    } else {
                        fr->logDec = 0.0f;
                    }
                }
            }
        }
    }

    // Draw envelopes (connect maxima) and fitted curves
    drawPeakPolyline(idxX, pkX, fx.peakCount, minV, maxV, x0, x1, yTop, yBot, TFT_ORANGE);
    drawPeakPolyline(idxY, pkY, fy.peakCount, minV, maxV, x0, x1, yTop, yBot, TFT_YELLOW);
    drawPeakPolyline(idxZ, pkZ, fz.peakCount, minV, maxV, x0, x1, yTop, yBot, TFT_CYAN);

    if (fx.fitOK) drawFitCurve(fx.A, fx.b, fx.C, fs, minV, maxV, x0, x1, yTop, yBot, TFT_ORANGE);
    if (fy.fitOK) drawFitCurve(fy.A, fy.b, fy.C, fs, minV, maxV, x0, x1, yTop, yBot, TFT_YELLOW);
    if (fz.fitOK) drawFitCurve(fz.A, fz.b, fz.C, fs, minV, maxV, x0, x1, yTop, yBot, TFT_CYAN);

    // Header
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(80, 1);
    M5.Display.print("Envelope");

    // Timestamp
    M5.Display.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
    } else {
        M5.Display.setTextColor(YELLOW);
    }
    M5.Display.print(timeBuf);
    M5.Display.setTextColor(WHITE);

    // Bottom stats
    int yTxt = yBot + 4;
    const int lh = 10;

    auto printAxisLine = [&](char axisChar, const EnvFitResult& fr, uint16_t col) {
        M5.Display.setTextColor(col);
        M5.Display.setCursor(2, yTxt);
        M5.Display.print(axisChar);
        M5.Display.print(": ");

        if (fr.peakCount < 6) {
            M5.Display.print("pk<6 (");
            M5.Display.print(fr.peakCount);
            M5.Display.print(")");
        } else if (!fr.fitOK) {
            M5.Display.print("fit err");
        } else {
            M5.Display.print("R2=");
            M5.Display.print(fr.R2, 2);

            if (fr.R2 > 0.80f) {
                M5.Display.print("  d=");
                M5.Display.print(fr.logDec, 3);
            } else {
                M5.Display.print("  d=--");
            }
        }

        yTxt += lh;
        M5.Display.setTextColor(WHITE);
    };

    printAxisLine('X', fx, TFT_ORANGE);
    printAxisLine('Y', fy, TFT_YELLOW);
    printAxisLine('Z', fz, TFT_CYAN);

    // Small footer: N and Fs
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(DW - 90, yBot + 4);
    M5.Display.print(SAMPLES);
    M5.Display.print(" @");
    M5.Display.print((int)(fs + 0.5f));
    M5.Display.print("Hz");

    drawArmedIndicator(isNonImmediateArmed());
}

void drawSpectrumScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    
    float minX = 0.0;
    float maxX = -9999.0;
    float minY = 0.0;
    float maxY = -9999.0;
    float minZ = 0.0;
    float maxZ = -9999.0;
    float maxXYZ;
    float minXYZ = 0.0;
    int   majAxis = 0;
    
    for (int i = 0; i < (SAMPLES / 2); i++) {
        if (vRealX[i] > maxX) maxX = vRealX[i];
        if (vRealY[i] > maxY) maxY = vRealY[i];
        if (vRealZ[i] > maxZ) maxZ = vRealZ[i];
        maxXYZ = max(max(maxX, maxY), maxZ) * 100;
        if ((maxX >= maxY) && (maxX >= maxZ)) majAxis = 1;
        if ((maxY >= maxX) && (maxY >= maxZ)) majAxis = 2;
        if ((maxZ >= maxX) && (maxZ >= maxY)) majAxis = 3;
    }
    
    M5.Display.fillScreen(TFT_BLACK);
    for (int i = 1; i < (SAMPLES / 2); i++) {
        int yp1 = map(vRealX[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        int xp1 = map(i - 1, 0, (SAMPLES / 2), 0, DW - 1);
        int yp2 = map(vRealX[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        int xp2 = map(i, 0, (SAMPLES / 2), 0, DW - 1);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_RED);
        yp1 = map(vRealY[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        yp2 = map(vRealY[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_GREEN);
        yp1 = map(vRealZ[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        yp2 = map(vRealZ[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        M5.Display.drawLine(xp1, yp1, xp2, yp2, TFT_BLUE);
    }
    
    if (majAxis == 1) {
        int xMplot = (samplingFrequency > 1e-3) ? (int)(xM / (samplingFrequency * 0.5) * (DW - 1)) : 0;
        if (xMplot < 0) xMplot = 0; if (xMplot > DW - 1) xMplot = DW - 1;
        M5.Display.setCursor(xMplot, 1);
        M5.Display.setTextColor(TFT_RED);
        M5.Display.print(xM);
    }
    if (majAxis == 2) {
        int yMplot = (samplingFrequency > 1e-3) ? (int)(yM / (samplingFrequency * 0.5) * (DW - 1)) : 0;
        if (yMplot < 0) yMplot = 0; if (yMplot > DW - 1) yMplot = DW - 1;
        M5.Display.setCursor(yMplot, 1);
        M5.Display.setTextColor(TFT_GREEN);
        M5.Display.print(yM);
    }
    if (majAxis == 3) {
        int zMplot = (samplingFrequency > 1e-3) ? (int)(zM / (samplingFrequency * 0.5) * (DW - 1)) : 0;
        if (zMplot < 0) zMplot = 0; if (zMplot > DW - 1) zMplot = DW - 1;
        M5.Display.setCursor(zMplot, 1);
        M5.Display.setTextColor(TFT_BLUE);
        M5.Display.print(zM);
    }
    
    int yp1 = map(0, minXYZ, maxXYZ, DH - 1, 0);
    M5.Display.drawLine(0, yp1, DW - 1, yp1, TFT_WHITE);
    M5.Display.drawLine(0, 0, 0, DH - 1, TFT_WHITE);

    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(80, 1);
    M5.Display.print("Spectrum");
    
    // Timestamp indicator
    M5.Display.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
    } else {
        M5.Display.setTextColor(YELLOW);
    }
    M5.Display.print(timeBuf);
    M5.Display.setTextColor(WHITE);

    drawArmedIndicator(isNonImmediateArmed());
}


// ==========================
// FFT Peak Picking (Screen 7)
// ==========================

static float refinePeakFreqQuadratic(const float* mag, int idx, int nBins, float fsHz) {
    // Quadratic interpolation around a peak bin to refine frequency estimate.
    // Returns frequency in Hz.
    if (idx <= 0 || idx >= (nBins - 1) || fsHz <= 0.0f) {
        return ((float)idx) * fsHz / (float)SAMPLES;
    }
    const float m1 = mag[idx - 1];
    const float m2 = mag[idx];
    const float m3 = mag[idx + 1];
    const float den = (m1 - 2.0f * m2 + m3);
    float delta = 0.0f;
    if (fabsf(den) > 1e-12f) {
        delta = 0.5f * (m1 - m3) / den;
        if (delta >  0.5f) delta =  0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }
    return ((float)idx + delta) * fsHz / (float)SAMPLES;
}

static int pickTopFftPeaks(const float* mag, int nBins, float fsHz, float* outFreqHz, int maxPeaks) {
    // Pragmatic peak picker for embedded use:
    // 1) Find local maxima above a small fraction of global max.
    // 2) Keep strongest candidates.
    // 3) Enforce minimum separation in bins.
    // 4) Refine peak frequency using quadratic interpolation.
    if (!mag || !outFreqHz || nBins < 8 || fsHz <= 0.0f || maxPeaks <= 0) return 0;

    float maxMag = 0.0f;
    for (int i = 1; i < nBins; i++) {
        if (mag[i] > maxMag) maxMag = mag[i];
    }
    if (maxMag <= 0.0f) return 0;

    const float thr = maxMag * 0.05f;  // 5% of global max (tunable)
    const int   TOP = 64;

    static uint16_t topIdx[TOP];
    static float    topMag[TOP];
    int topCount = 0;

    float minTop = 1e30f;
    int   minPos = -1;

    auto recomputeMin = [&]() {
        minTop = 1e30f;
        minPos = 0;
        for (int k = 0; k < topCount; k++) {
            if (topMag[k] < minTop) {
                minTop = topMag[k];
                minPos = k;
            }
        }
    };

    // Gather top local maxima
    for (int i = 2; i < (nBins - 2); i++) {
        const float m = mag[i];
        if (m < thr) continue;
        if (!(m > mag[i - 1] && m >= mag[i + 1])) continue;

        if (topCount < TOP) {
            topIdx[topCount] = (uint16_t)i;
            topMag[topCount] = m;
            topCount++;
            if (m < minTop) {
                minTop = m;
                minPos = topCount - 1;
            }
        } else if (m > minTop && minPos >= 0) {
            topIdx[minPos] = (uint16_t)i;
            topMag[minPos] = m;
            recomputeMin();
        }
    }

    if (topCount == 0) return 0;

    // Sort candidates by magnitude descending (selection sort; TOP is small)
    for (int a = 0; a < topCount - 1; a++) {
        int best = a;
        for (int b = a + 1; b < topCount; b++) {
            if (topMag[b] > topMag[best]) best = b;
        }
        if (best != a) {
            float tm = topMag[a]; topMag[a] = topMag[best]; topMag[best] = tm;
            uint16_t ti = topIdx[a]; topIdx[a] = topIdx[best]; topIdx[best] = ti;
        }
    }

    // Minimum separation (in bins) – about 0.5 Hz, but at least 2 bins
    const float binW = fsHz / (float)SAMPLES;
    int minSepBins = 2;
    if (binW > 0.0f) {
        minSepBins = (int)lroundf(0.5f / binW);
        if (minSepBins < 2)  minSepBins = 2;
        if (minSepBins > 20) minSepBins = 20;
    }

    int chosenIdx[8] = {0};
    int outCount = 0;

    for (int k = 0; k < topCount && outCount < maxPeaks; k++) {
        const int idx = (int)topIdx[k];

        bool tooClose = false;
        for (int j = 0; j < outCount; j++) {
            if (abs(idx - chosenIdx[j]) < minSepBins) {
                tooClose = true;
                break;
            }
        }
        if (tooClose) continue;

        chosenIdx[outCount] = idx;
        outFreqHz[outCount] = refinePeakFreqQuadratic(mag, idx, nBins, fsHz);
        outCount++;
    }

    // If still not enough peaks (e.g., low-energy signal), relax threshold and pick remaining maxima by magnitude
    // (keeps behavior stable without exploding false positives).
    return outCount;
}

// ============================================================================
// SERIAL OUTPUT HELPERS
// ============================================================================

void dumpTimeSeriesSerial() {
    Serial.print("# Timestamp: ");
    printDateTime(&currentRecordTime);
    Serial.println();
    Serial.print("# View: ");
    if (currentViewRecord < 0) Serial.println("LIVE");
    else { Serial.print("REC "); Serial.println(currentViewRecord); }
    Serial.printf("# Fs: %.3f Hz, N: %d\n", samplingFrequency, (int)SAMPLES);
    Serial.println("# idx t_us ax ay az");
    for (int i = 0; i < SAMPLES; i++) {
        Serial.print(i);
        Serial.print(' ');
        Serial.print(t[i]);
        Serial.print(' ');
        Serial.print(accXvec[i], 6);
        Serial.print(' ');
        Serial.print(accYvec[i], 6);
        Serial.print(' ');
        Serial.println(accZvec[i], 6);
    }
    Serial.println("#END_A");
}

void dumpFftSerial() {
    Serial.print("# Timestamp: ");
    printDateTime(&currentRecordTime);
    Serial.println();
    Serial.print("# View: ");
    if (currentViewRecord < 0) Serial.println("LIVE");
    else { Serial.print("REC "); Serial.println(currentViewRecord); }
    Serial.printf("# Fs: %.3f Hz, N: %d\n", samplingFrequency, (int)SAMPLES);
    Serial.println("# f_Hz X Y Z");
    const int nBins = (SAMPLES / 2) + 1;
    for (int i = 0; i < nBins; i++) {
        float f = (float)i * (float)samplingFrequency / (float)SAMPLES;
        Serial.print(f, 6);
        Serial.print(' ');
        Serial.print(vRealX[i], 8);
        Serial.print(' ');
        Serial.print(vRealY[i], 8);
        Serial.print(' ');
        Serial.println(vRealZ[i], 8);
    }
    Serial.println("#END_F");
}

static void computeStatsFromAcc(float* outMin, float* outMax, float* outMean, float* outSd) {
    float minX =  9999.0f, maxX = -9999.0f;
    float minY =  9999.0f, maxY = -9999.0f;
    float minZ =  9999.0f, maxZ = -9999.0f;
    float meanX = 0.0f, meanY = 0.0f, meanZ = 0.0f;

    for (int i = 0; i < SAMPLES; i++) {
        float ax = accXvec[i];
        float ay = accYvec[i];
        float az = accZvec[i];
        if (ax < minX) minX = ax; if (ax > maxX) maxX = ax;
        if (ay < minY) minY = ay; if (ay > maxY) maxY = ay;
        if (az < minZ) minZ = az; if (az > maxZ) maxZ = az;
        meanX += ax; meanY += ay; meanZ += az;
    }
    meanX /= (float)SAMPLES;
    meanY /= (float)SAMPLES;
    meanZ /= (float)SAMPLES;

    float sdX = 0.0f, sdY = 0.0f, sdZ = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
        float dx = accXvec[i] - meanX;
        float dy = accYvec[i] - meanY;
        float dz = accZvec[i] - meanZ;
        sdX += dx*dx; sdY += dy*dy; sdZ += dz*dz;
    }
    sdX = sqrtf(sdX / (float)SAMPLES);
    sdY = sqrtf(sdY / (float)SAMPLES);
    sdZ = sqrtf(sdZ / (float)SAMPLES);

    outMin[0]=minX; outMin[1]=minY; outMin[2]=minZ;
    outMax[0]=maxX; outMax[1]=maxY; outMax[2]=maxZ;
    outMean[0]=meanX; outMean[1]=meanY; outMean[2]=meanZ;
    outSd[0]=sdX; outSd[1]=sdY; outSd[2]=sdZ;
}

void dumpSummarySerial() {
    Serial.println("=== SUMMARY ===");
    Serial.print("# Timestamp: ");
    printDateTime(&currentRecordTime);
    Serial.println();
    Serial.print("# View: ");
    if (currentViewRecord < 0) Serial.println("LIVE");
    else { Serial.print("REC "); Serial.println(currentViewRecord); }
    Serial.printf("# Fs: %.3f Hz, N: %d\n", samplingFrequency, (int)SAMPLES);

    float mn[3], mx[3], mu[3], sd[3];
    computeStatsFromAcc(mn, mx, mu, sd);

    Serial.println("[Statistics]");
    Serial.println("Value	X	Y	Z");
    Serial.print("MAX:	");  Serial.printf("%.6f	%.6f	%.6f\n", mx[0], mx[1], mx[2]);
    Serial.print("MEAN:	"); Serial.printf("%.6f	%.6f	%.6f\n", mu[0], mu[1], mu[2]);
    Serial.print("SD:	");   Serial.printf("%.6f	%.6f	%.6f\n", sd[0], sd[1], sd[2]);
    Serial.print("MIN:	");  Serial.printf("%.6f	%.6f	%.6f\n", mn[0], mn[1], mn[2]);
    Serial.print("F1[Hz]:	"); Serial.printf("%.3f	%.3f	%.3f\n", xM, yM, zM);

    Serial.println();
    Serial.println("[FFT Peaks 1-7]");
    float fx[7]={0}, fy[7]={0}, fz[7]={0};
    const int nBins = (SAMPLES / 2);
    const int nx = pickTopFftPeaks(vRealX, nBins, (float)samplingFrequency, fx, 7);
    const int ny = pickTopFftPeaks(vRealY, nBins, (float)samplingFrequency, fy, 7);
    const int nz = pickTopFftPeaks(vRealZ, nBins, (float)samplingFrequency, fz, 7);
    Serial.println("#	X[Hz]	Y[Hz]	Z[Hz]");
    for (int i=0;i<7;i++) {
        Serial.print(i+1); Serial.print('\t');
        if (i<nx) Serial.print(fx[i], 3); else Serial.print('-');
        Serial.print('\t');
        if (i<ny) Serial.print(fy[i], 3); else Serial.print('-');
        Serial.print('\t');
        if (i<nz) Serial.print(fz[i], 3); else Serial.print('-');
        Serial.println();
    }

    Serial.println();
    Serial.println("[Envelope / Log decrement]");
    const float fs = (samplingFrequency > 1e-3f) ? (float)samplingFrequency : (float)samplingRateSettingHz;

    float dcX=0, dcY=0, dcZ=0;
    for (int i=0;i<SAMPLES;i++){ dcX+=accXvec[i]; dcY+=accYvec[i]; dcZ+=accZvec[i]; }
    dcX/=SAMPLES; dcY/=SAMPLES; dcZ/=SAMPLES;

    static uint16_t idxX[MAX_ENV_PEAKS];
    static uint16_t idxY[MAX_ENV_PEAKS];
    static uint16_t idxZ[MAX_ENV_PEAKS];
    static float pkX[MAX_ENV_PEAKS];
    static float pkY[MAX_ENV_PEAKS];
    static float pkZ[MAX_ENV_PEAKS];

    EnvFitResult fxr, fyr, fzr;
    fxr.peakCount = findLocalMaximaAboveDC(accXvec, SAMPLES, dcX, fs, idxX, pkX, MAX_ENV_PEAKS);
    fyr.peakCount = findLocalMaximaAboveDC(accYvec, SAMPLES, dcY, fs, idxY, pkY, MAX_ENV_PEAKS);
    fzr.peakCount = findLocalMaximaAboveDC(accZvec, SAMPLES, dcZ, fs, idxZ, pkZ, MAX_ENV_PEAKS);

    auto computeOne = [&](EnvFitResult &fr, const uint16_t* idx, const float* pk){
        if (fr.peakCount >= 6) {
            fr.fitTried = true;
            float A,b,C,R2;
            if (fitExpDecayWithOffset(idx, pk, fr.peakCount, fs, &A, &b, &C, &R2)) {
                fr.fitOK = true; fr.A=A; fr.b=b; fr.C=C; fr.R2=R2;
                if (R2 > 0.80f) {
                    float T = estimatePeakPeriod(idx, fr.peakCount, fs);
                    if (T > 1e-6f) fr.logDec = b * T;
                }
            }
        }
    };
    computeOne(fxr, idxX, pkX);
    computeOne(fyr, idxY, pkY);
    computeOne(fzr, idxZ, pkZ);

    auto printOne = [&](const char* name, const EnvFitResult& fr){
        Serial.print(name);
        Serial.print(": pk="); Serial.print(fr.peakCount);
        if (fr.peakCount < 6) { Serial.println(" (<6, no fit)"); return; }
        if (!fr.fitOK) { Serial.println(" (fit failed)"); return; }
        Serial.print(" R2="); Serial.print(fr.R2, 3);
        if (fr.R2 > 0.80f) {
            Serial.print(" delta="); Serial.println(fr.logDec, 6);
        } else {
            Serial.println(" (R2<=0.80, delta n/a)");
        }
    };

    printOne("X", fxr);
    printOne("Y", fyr);
    printOne("Z", fzr);

    Serial.println("=== END SUMMARY ===");
}

void drawFftPeaksScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    M5.Display.setTextColor(WHITE);

    // Title
    M5.Display.setCursor(5, 1);
    M5.Display.print("FFT PEAKS (1-7)");

    // Timestamp indicator (same convention as other screens)
    M5.Display.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
    } else {
        M5.Display.setTextColor(YELLOW);
    }
    M5.Display.print(timeBuf);
    M5.Display.setTextColor(WHITE);

    const int nBins = (SAMPLES / 2);
    float fx[7] = {0}, fy[7] = {0}, fz[7] = {0};
    const int nx = pickTopFftPeaks(vRealX, nBins, (float)samplingFrequency, fx, 7);
    const int ny = pickTopFftPeaks(vRealY, nBins, (float)samplingFrequency, fy, 7);
    const int nz = pickTopFftPeaks(vRealZ, nBins, (float)samplingFrequency, fz, 7);

    // Table header
    int y0 = 16;
    M5.Display.setCursor(5, y0);
    M5.Display.setTextColor(WHITE);
    M5.Display.print("#");

    M5.Display.setCursor(40, y0);
    M5.Display.setTextColor(TFT_RED);
    M5.Display.print("X");

    M5.Display.setCursor(110, y0);
    M5.Display.setTextColor(TFT_GREEN);
    M5.Display.print("Y");

    M5.Display.setCursor(180, y0);
    M5.Display.setTextColor(TFT_BLUE);
    M5.Display.print("Z");

    const int rowH = 14;
    for (int r = 0; r < 7; r++) {
        const int y = y0 + 10 + r * rowH;

        // Row index
        M5.Display.setTextColor(WHITE);
        M5.Display.setCursor(5, y);
        M5.Display.printf("%d", r + 1);

        // X
        M5.Display.setTextColor(TFT_RED);
        M5.Display.setCursor(30, y);
        if (r < nx) M5.Display.printf("%6.1f", fx[r]);
        else        M5.Display.print("  -");

        // Y
        M5.Display.setTextColor(TFT_GREEN);
        M5.Display.setCursor(100, y);
        if (r < ny) M5.Display.printf("%6.1f", fy[r]);
        else        M5.Display.print("  -");

        // Z
        M5.Display.setTextColor(TFT_BLUE);
        M5.Display.setCursor(170, y);
        if (r < nz) M5.Display.printf("%6.1f", fz[r]);
        else        M5.Display.print("  -");
    }

    // Footer: Fs and N
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(5, 125);
    M5.Display.printf("Fs=%.0fHz  N=%d", samplingFrequency, (int)SAMPLES);

    drawArmedIndicator(isNonImmediateArmed());
}

void drawCorrelationScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    
    float minX =  9999.0;
    float maxX = -9999.0;
    float minY =  9999.0;
    float maxY = -9999.0;
    float minZ =  9999.0;
    float maxZ = -9999.0;
    float maxXYZ;
    float minXYZ;
    
    for (int i = 0; i < SAMPLES; i++) {
        if (accXvec[i] > maxX) maxX = accXvec[i];
        if (accXvec[i] < minX) minX = accXvec[i];
        if (accYvec[i] > maxY) maxY = accYvec[i];
        if (accYvec[i] < minY) minY = accYvec[i];
        if (accZvec[i] > maxZ) maxZ = accZvec[i];
        if (accZvec[i] < minZ) minZ = accZvec[i];
        maxXYZ = max(max(maxX, maxY), maxZ) * 100;
        minXYZ = min(min(minX, minY), minZ) * 100;
    }
    
    int aX1 = map(0, minXYZ, maxXYZ, 0, 119);
    int aY1 = map(0, minXYZ, maxXYZ, 135, 15);
    M5.Display.drawLine(aX1, 15, aX1, 135, WHITE);
    M5.Display.drawLine(0, aY1, 119, aY1, WHITE);

    int aX2 = map(0, minXYZ, maxXYZ, 120, 239);
    int aY2 = map(0, minXYZ, maxXYZ, 135, 15);
    M5.Display.drawLine(aX2, 15, aX2, 135, WHITE);
    M5.Display.drawLine(120, aY2, 239, aY2, WHITE);
    
    for (int i = 0; i < SAMPLES; i++) {
        int yp1 = map(accYvec[i] * 100, minXYZ, maxXYZ, 135, 15);
        int xp1 = map(accXvec[i] * 100, minXYZ, maxXYZ, 0, 119);
        M5.Display.drawPixel(xp1, yp1, ORANGE);
        int yp2 = map(accZvec[i] * 100, minXYZ, maxXYZ, 135, 15);
        int xp2 = map(accXvec[i] * 100, minXYZ, maxXYZ, 120, 239);
        M5.Display.drawPixel(xp2, yp2, MAGENTA);
    }
    
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(5, 1);
    M5.Display.print("X-Y corr");
    M5.Display.setCursor(125, 1);
    M5.Display.print("X-Z corr");

    drawArmedIndicator(isNonImmediateArmed());
}

void drawSpectrogramScreen() {
    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.fillScreen(TFT_BLACK);

    // Title with timestamp
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(2, 2);
    M5.Display.print("Spectro ");
    
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
    } else {
        M5.Display.setTextColor(YELLOW);
    }
    M5.Display.print(timeBuf);
    M5.Display.setTextColor(WHITE);

    if (specSegments <= 0) {
        M5.Display.setCursor(2, 20);
        M5.Display.print("No data yet");
    } else {
        const float fs = (samplingFrequency > 1e-3) ? (float)samplingFrequency : (float)samplingRateSettingHz;

        int plotY0 = 15;
        int plotH  = DH - plotY0 - th;
        int plotW  = DW;

        int segH  = max(1, plotH / specSegments);
        float span = specLogMax - specLogMin;
        if (span < 1e-3f) span = 1e-3f;

        for (int s = 0; s < specSegments; s++) {
            int yBottom = DH - 1 - s * segH - th;
            int yTop    = yBottom - segH + 1;
            if (yTop < plotY0) yTop = plotY0;
            if (yBottom < plotY0) break;

            for (int k = 0; k < SPEC_BINS; k++) {
                float lv   = specMag[s][k];
                float norm = (lv - specLogMin) / span;
                if (norm < 0.0f) norm = 0.0f;
                if (norm > 1.0f) norm = 1.0f;

                uint8_t  v = (uint8_t)(norm * 255.0f);
                uint16_t c = spectroColor(v);

                int x0 = (k * plotW) / SPEC_BINS;
                int x1 = ((k + 1) * plotW) / SPEC_BINS - 1;
                if (x1 < x0) x1 = x0;

                int w = x1 - x0 + 1;
                M5.Display.fillRect(x0, yTop, w, yBottom - yTop + 1, c);
            }
        }

        M5.Display.setTextColor(WHITE);
        M5.Display.setCursor(2, DH - th);
        M5.Display.print("0 Hz");

        M5.Display.setCursor(DW - 60, DH - th);
        M5.Display.print((int)(fs / 2));
        M5.Display.print(" Hz");
    }

    drawArmedIndicator(isNonImmediateArmed());
}

void drawStorageScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setRotation(1);
    M5.Display.setTextColor(WHITE);

    // Title with current RTC time
    M5.Display.setCursor(5, 1);
    M5.Display.print("STORAGE  RTC:");
    RecordDateTime now;
    getRtcTime(&now);
    char timeBuf[16];
    formatDateTimeShort(&now, timeBuf, sizeof(timeBuf));
    M5.Display.print(timeBuf);

    M5.Display.setCursor(5, th * 2);
    M5.Display.print("Records: ");
    M5.Display.setTextColor(CYAN);
    M5.Display.print(storedRecordCount);
    M5.Display.setTextColor(WHITE);
    M5.Display.print("/");
    M5.Display.print(MAX_RECORDS);

    M5.Display.setCursor(5, th * 3);
    M5.Display.print("Flash: ");
    if (storageReady) {
        int usedKB = LittleFS.usedBytes() / 1024;
        int totalKB = LittleFS.totalBytes() / 1024;
        int pct = (usedKB * 100) / totalKB;
        M5.Display.print(usedKB);
        M5.Display.print("/");
        M5.Display.print(totalKB);
        M5.Display.print(" KB (");
        M5.Display.print(pct);
        M5.Display.print("%)");
    } else {
        M5.Display.setTextColor(RED);
        M5.Display.print("NOT READY");
        M5.Display.setTextColor(WHITE);
    }

    M5.Display.setCursor(5, th * 4);
    M5.Display.print("View: ");
    if (currentViewRecord < 0) {
        M5.Display.setTextColor(GREEN);
        M5.Display.print("LIVE");
    } else {
        M5.Display.setTextColor(YELLOW);
        M5.Display.print("#");
        M5.Display.print(currentViewRecord);
        M5.Display.setTextColor(WHITE);
        M5.Display.print(" @ ");
        formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
        M5.Display.print(timeBuf);
    }
    M5.Display.setTextColor(WHITE);

    // Instructions
    M5.Display.setCursor(5, th * 6);
    M5.Display.setTextColor(CYAN);
    M5.Display.print("BtnA:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" Browse records");

    M5.Display.setCursor(5, th * 7);
    M5.Display.setTextColor(RED);
    M5.Display.print("Long BtnB:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" CLEAR ALL");

    drawArmedIndicator(isNonImmediateArmed());
}

void drawTriggerScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE);

    // Compact layout for 135px height screen
    const int lh = 14;          // line height (px)
    const int xLabel = 5;
    const int xVal   = 125;

    // Title
    M5.Display.setCursor(xLabel, 0);
    M5.Display.setTextColor(CYAN);
    M5.Display.print("TRIGGER SETTINGS");
    M5.Display.setTextColor(WHITE);

    // Line helper: highlight with rectangle
    auto drawLine = [&](int yy, const char* label, const String& value, bool selected) {
        if (selected) {
            M5.Display.drawRect(0, yy - 2, DW, lh + 3, YELLOW);
        }
        M5.Display.setCursor(xLabel, yy);
        M5.Display.setTextColor(WHITE);
        M5.Display.print(label);

        // value aligned to the right-ish
        M5.Display.setCursor(xVal, yy);
        M5.Display.setTextColor(CYAN);
        M5.Display.print(value);
        M5.Display.setTextColor(WHITE);
    };

    int y = lh + 2;

    // Mode
    String modeStr = "Immediate";
    if (trigMode == TRIG_DELAY) modeStr = "Delay";
    if (trigMode == TRIG_ACCEL) modeStr = "Accel";
    drawLine(y, "Mode:", modeStr, (trigMenuItem == 0));
    y += lh;

    // Param (Delay or Acc threshold)
    if (isTriggerMenuItemVisible(1)) {
        const char* paramLabel = (trigMode == TRIG_ACCEL) ? "Acc thr:" : "Delay:";
        String paramStr = "-";
        if (trigMode == TRIG_DELAY) {
            paramStr = String(delayOptionsMs[delayIndex] / 1000.0f, 0) + " s";
        } else if (trigMode == TRIG_ACCEL) {
            paramStr = String(accelTrigThresholdG, 1) + " g";
        }
        drawLine(y, paramLabel, paramStr, (trigMenuItem == 1));
        y += lh;
    }

    // Status + live readouts (2 lines), keep above footer
    int yFooter = DH - lh;
    int yStatus = yFooter - 2 * lh;
    if (yStatus < y + 2) yStatus = y + 2;

    M5.Display.setCursor(xLabel, yStatus);
    M5.Display.setTextColor(WHITE);

    String st = "IDLE";
    if (isTrig) st = "RECORDING";
    else if (isLogging && startPending) st = "PENDING";
    else if (isLogging && triggerArmed) st = "ARMED";

    M5.Display.print("Status: ");
    M5.Display.setTextColor(GREEN);
    M5.Display.print(st);
    M5.Display.setTextColor(WHITE);

    // Countdown if pending
    if (isLogging && startPending) {
        uint32_t now = millis();
        int32_t msLeft = (int32_t)pendingStartMs - (int32_t)now;
        if (msLeft < 0) msLeft = 0;
        M5.Display.print(" (");
        M5.Display.print(msLeft);
        M5.Display.print(" ms)");
    }
    // Live readout / hints
    M5.Display.setCursor(xLabel, yStatus + lh);
    if (trigMode == TRIG_ACCEL) {
        float ax, ay, az;
        M5.Imu.getAccel(&ax, &ay, &az);
        accelMag_live = accelMagnitudeG(ax, ay, az);
        M5.Display.print("|a|: ");
        M5.Display.setTextColor(CYAN);
        M5.Display.print(accelMag_live, 2);
        M5.Display.print(" g");
        M5.Display.setTextColor(WHITE);
    } else {
        M5.Display.print("A: change, hold A: next");
    }

    // Footer hints
    M5.Display.setCursor(xLabel, yFooter);
    M5.Display.setTextColor(CYAN);
    M5.Display.print("BtnA:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" edit   ");
    M5.Display.setTextColor(CYAN);
    M5.Display.print("BtnB:");
    M5.Display.setTextColor(WHITE);
    M5.Display.print(" next screen");

    drawArmedIndicator(isNonImmediateArmed());
}

