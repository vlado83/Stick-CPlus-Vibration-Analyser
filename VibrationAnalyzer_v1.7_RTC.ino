// Verzija 1.6
// Based on v1.5 by Vladimir Divic
// Added: RTC timestamp for each record, serial command to set RTC
//
// FEATURES:
// - Auto-saves each recording to flash storage with RTC timestamp
// - Circular buffer: oldest records overwritten when full (~50 records max)
// - Browse through stored records on Screen 6
// - Data persists across power cycles
// - RTC time setting via Serial
//
// CONTROLS:
// - BtnA: Record (or browse prev record on Screen 6)
// - BtnB short: Cycle screens (1-6)
// - BtnB long: Serial dump (or clear all records on Screen 6)
// - BtnB held at boot: External trigger mode
//
// SERIAL COMMANDS:
// - 'R' : Trigger recording
// - 'E' : Export all records
// - 'I' : Print storage info
// - 'T' : Print current RTC time
// - "Tyyyy-MM-dd HH:mm:ss" : Set RTC time (e.g., "T2025-01-15 14:30:00")
//
// SCREENS:
// 0 - Startup/info
// 1 - Statistics
// 2 - Time series
// 3 - Spectrum
// 4 - X-Y / X-Z correlation
// 5 - Spectrogram
// 6 - Storage browser

#include <M5StickCPlus.h>
#include "arduinoFFT.h"
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <LittleFS.h>

#define SAMPLES 1024
#define TTNS    4
#define CALIB_T_PIN 26
#define EXT_TRIG_PIN 36

// ---------- Storage Configuration ----------
#define MAX_RECORDS      30           // Maximum stored records (~32 fits in 1.408 MB)
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

// Current record's RTC time (set when recording starts)
RecordDateTime currentRecordTime;

// Storage state variables
int  storedRecordCount = 0;
int  currentViewRecord = -1;          // -1 = live data, 0..N-1 = stored records
int  oldestRecordIndex = 0;           // For circular buffer
int  newestRecordIndex = -1;
bool storageReady = false;

// ---------- Acceleration Data Arrays ----------
float accXvec[SAMPLES];
float accYvec[SAMPLES];
float accZvec[SAMPLES];

float vRealX[SAMPLES];
float vRealY[SAMPLES];
float vRealZ[SAMPLES];

float vReal[SAMPLES];
float vImag[SAMPLES];

// ---------- Spectrogram (screen 5) ----------
#define SEG_LEN          128          // window length for each FFT
#define SEG_OVERLAP_PCT  50           // overlap percentage between windows
#define SEG_MAX          17           // max number of segments stored per record
#define SPEC_BINS        (SEG_LEN / 2)

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

long int t[SAMPLES];
long int t0;
long int prevTime;
int      count;
bool     isLogging;
bool     isTrig;
uint16_t trigLevel = 3000;
bool     isExtTriged;
bool     BTCONNECT = true;

ArduinoFFT<float> FFT = ArduinoFFT<float>();
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
BluetoothSerial SerialBT;

int   th;        // text height
int   curDisp;   // 0 startup, 1 stats, 2 timeseries, 3 spectrum, 4 corr, 5 spectro, 6 storage
Preferences preferences;

int DW = 240;
int DH = 135;

// Serial input buffer for time setting
String serialBuffer = "";

// ============================================================================
// RTC FUNCTIONS
// ============================================================================

// Get current RTC time into RecordDateTime structure
void getRtcTime(RecordDateTime* dt) {
    RTC_TimeTypeDef rtcTime;
    RTC_DateTypeDef rtcDate;
    
    M5.Rtc.GetTime(&rtcTime);
    M5.Rtc.GetDate(&rtcDate);
    
    dt->year   = rtcDate.Year;
    dt->month  = rtcDate.Month;
    dt->day    = rtcDate.Date;
    dt->hour   = rtcTime.Hours;
    dt->minute = rtcTime.Minutes;
    dt->second = rtcTime.Seconds;
}

// Set RTC time from RecordDateTime structure
void setRtcTime(RecordDateTime* dt) {
    RTC_TimeTypeDef rtcTime;
    RTC_DateTypeDef rtcDate;
    
    rtcDate.Year  = dt->year;
    rtcDate.Month = dt->month;
    rtcDate.Date  = dt->day;
    
    rtcTime.Hours   = dt->hour;
    rtcTime.Minutes = dt->minute;
    rtcTime.Seconds = dt->second;
    
    M5.Rtc.SetTime(&rtcTime);
    M5.Rtc.SetDate(&rtcDate);
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

// Print RecordDateTime to SerialBT
void printDateTimeBT(RecordDateTime* dt) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt->year, dt->month, dt->day,
             dt->hour, dt->minute, dt->second);
    SerialBT.print(buf);
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
                    isLogging = true;
                }
                else if (serialBuffer.charAt(0) == 'E' || serialBuffer.charAt(0) == 'e') {
                    exportAllRecordsSerial();
                }
                else if (serialBuffer.charAt(0) == 'I' || serialBuffer.charAt(0) == 'i') {
                    printStorageInfo();
                }
                else if (serialBuffer.charAt(0) == 'H' || serialBuffer.charAt(0) == 'h' || serialBuffer.charAt(0) == '?') {
                    // Help
                    Serial.println("=== Serial Commands ===");
                    Serial.println("R - Trigger recording");
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

// Process SerialBT input (same commands)
void processSerialBTInput() {
    while (SerialBT.available()) {
        char c = SerialBT.read();
        
        if (c == 'R' || c == 'r') {
            isLogging = true;
        }
        else if (c == 'T' || c == 't') {
            // For BT, just print time (setting via BT would need buffering)
            RecordDateTime now;
            getRtcTime(&now);
            SerialBT.print("RTC: ");
            printDateTimeBT(&now);
            SerialBT.println();
        }
        else if (c == 'E' || c == 'e') {
            // Export abbreviated version over BT
            SerialBT.print("Records: ");
            SerialBT.println(storedRecordCount);
        }
        else if (c == 'I' || c == 'i') {
            SerialBT.print("Storage: ");
            SerialBT.print(storedRecordCount);
            SerialBT.print("/");
            SerialBT.println(MAX_RECORDS);
        }
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
    
    // Write metadata
    file.write((uint8_t*)&meta, sizeof(RecordMeta));
    
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
    
    // Read metadata
    RecordMeta meta;
    file.read((uint8_t*)&meta, sizeof(RecordMeta));
    
    samplingFrequency = meta.samplingFreq;
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

    return M5.Lcd.color565(r, g, b);
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

void setup() {
    M5.begin();
    M5.Axp.begin();
    M5.IMU.Init();
    M5.Rtc.begin();  // Initialize RTC

    pinMode(M5_LED, OUTPUT);
    pinMode(CALIB_T_PIN, OUTPUT);
    pinMode(EXT_TRIG_PIN, INPUT);

    // BtnB held at boot = external trigger mode
    if (M5.BtnB.isPressed()) isExtTriged = true;
    else isExtTriged = false;

    digitalWrite(M5_LED, HIGH);
    isLogging = false;
    isTrig    = false;
    count     = 0;
    digitalWrite(CALIB_T_PIN, LOW);
    prevTime = millis();

    // Initialize storage
    storageReady = initStorage();

    // Print current RTC time
    printCurrentTime();

    if (BTCONNECT)
        SerialBT.begin("Vibe002");

    // Startup Screen
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextFont(2);
    th = 17;

    M5.Lcd.setCursor(5, 1);
    M5.Lcd.print("Vibration analyser");
    M5.Lcd.setCursor(5, th);
    M5.Lcd.print("ver. 1.7 + RTC");
    M5.Lcd.setCursor(5, 2 * th);
    M5.Lcd.print("Vladimir Divic, 2025");
    M5.Lcd.setCursor(5, 3 * th);
    M5.Lcd.print("BT: Vibe002");
    
    // Show current RTC time
    M5.Lcd.setCursor(5, 4 * th);
    M5.Lcd.print("RTC: ");
    RecordDateTime now;
    getRtcTime(&now);
    char timeBuf[24];
    formatDateTime(&now, timeBuf, sizeof(timeBuf));
    M5.Lcd.print(timeBuf);
    
    M5.Lcd.setCursor(5, 5 * th);
    if (storageReady) {
        M5.Lcd.print("Storage: ");
        M5.Lcd.print(storedRecordCount);
        M5.Lcd.print("/");
        M5.Lcd.print(MAX_RECORDS);
        M5.Lcd.print(" rec");
    } else {
        M5.Lcd.setTextColor(RED);
        M5.Lcd.print("Storage FAILED!");
        M5.Lcd.setTextColor(WHITE);
    }

    M5.Lcd.setCursor(5, 6 * th);
    M5.Lcd.print("BtnA-rec BtnB-screen");
    M5.Lcd.setCursor(5, 7 * th);
    if (isExtTriged) M5.Lcd.print("Ext trig mode");
    else             M5.Lcd.print("Self-trigger mode");

    curDisp = 0;
    timeout = extendT + millis();
    
    // Print help to serial
    Serial.println("=== Vibration Analyzer v1.7 ===");
    Serial.println("Serial commands: R=record, T=time, E=export, I=info, H=help");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    M5.update();

    // --- BtnA handling ---
    static bool btnAHandled = false;
    
    if (M5.BtnA.isPressed() && !btnAHandled) {
        btnAHandled = true;
        timeout = extendT + millis();
        
        if (curDisp == 6 && storedRecordCount > 0) {
            // On storage screen: browse through stored records
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
            // Normal: start recording
            isLogging = true;
        }
    }
    
    if (M5.BtnA.wasReleased()) {
        btnAHandled = false;
    }

    // Process serial input (including time setting)
    processSerialInput();
    
    // Process SerialBT input
    processSerialBTInput();

    // External trigger logic
    if (isExtTriged) {
        if (isLogging) {
            if (analogRead(EXT_TRIG_PIN) > 4000) {
                isTrig = true;
            }
        }
    } else {
        if (isLogging) isTrig = true;
    }

    // --- Sampling finished: FFT, stats, spectrogram, save, BT out ---
    if (count == SAMPLES) {
        digitalWrite(CALIB_T_PIN, LOW);
        digitalWrite(M5_LED, HIGH);

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

        // BlueTooth output
        if (BTCONNECT) {
            // Send timestamp first
            SerialBT.print("*T");
            printDateTimeBT(&currentRecordTime);
            SerialBT.println("*");
            
            SerialBT.print("*KC*");
            for (int i = 0; i < SAMPLES; i++) {
                SerialBT.print("*KX");
                SerialBT.print(t[i]);
                SerialBT.print("Y");
                SerialBT.print(accXvec[i], 5);
                SerialBT.print(",X");
                SerialBT.print(t[i]);
                SerialBT.print("Y");
                SerialBT.print(accYvec[i], 5);
                SerialBT.print(",X");
                SerialBT.print(t[i]);
                SerialBT.print("Y");
                SerialBT.print(accZvec[i], 5);
                SerialBT.println("*");
            }

            SerialBT.print("*HC*");
            for (int i = 0; i < (SAMPLES / 2); i++) {
                float f = (i * 1.0 * samplingFrequency) / (float)SAMPLES;
                SerialBT.print("*HX");
                SerialBT.print(f, 5);
                SerialBT.print("Y");
                SerialBT.print(vRealX[i], 5);
                SerialBT.print(",X");
                SerialBT.print(f, 5);
                SerialBT.print("Y");
                SerialBT.print(vRealY[i], 5);
                SerialBT.print(",X");
                SerialBT.print(f, 5);
                SerialBT.print("Y");
                SerialBT.print(vRealZ[i], 5);
                SerialBT.println("*");
            }

            SerialBT.print("*X");
            SerialBT.print(xM);
            SerialBT.println("*");

            SerialBT.print("*Y");
            SerialBT.print(yM);
            SerialBT.println("*");

            SerialBT.print("*Z");
            SerialBT.print(zM);
            SerialBT.println("*");
        }

        count     = 0;
        isLogging = false;
        isTrig    = false;
    }

    // --- BtnB short/long press handling ---
    static bool btnBLongHandled = false;
    const uint32_t LONG_PRESS_MS = 700;

    // Long press action
    if (M5.BtnB.pressedFor(LONG_PRESS_MS) && !btnBLongHandled) {
        btnBLongHandled = true;

        if (curDisp == 6) {
            // Long press on storage screen = clear all records
            clearAllRecords();
            drawStorageScreen();
        }
        else if (curDisp == 1) {
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
        else if ((curDisp == 2) || (curDisp == 4)) {
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
        else if ((curDisp == 3) || (curDisp == 5)) {
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
            if (curDisp > 6) curDisp = 1;

            // --- Draw screens ---
            if (curDisp == 1) {
                drawStatisticsScreen();
            }

            if (curDisp == 2) {
                drawTimeSeriesScreen();
            }

            if (curDisp == 3) {
                drawSpectrumScreen();
            }

            if (curDisp == 4) {
                drawCorrelationScreen();
            }

            if (curDisp == 5) {
                drawSpectrogramScreen();
            }

            if (curDisp == 6) {
                drawStorageScreen();
            }
        }

        btnBLongHandled = false;
    }

    // --- Sampling logic ---
    if (isTrig) {
        if (millis() - prevTime > TTNS) {
            digitalWrite(M5_LED, LOW);
            digitalWrite(CALIB_T_PIN, !digitalRead(CALIB_T_PIN));

            M5.IMU.getAccelData(&accX, &accY, &accZ);

            if (count == 0) {
                // *** CAPTURE RTC TIME AT START OF RECORDING ***
                getRtcTime(&currentRecordTime);
                
                t0     = micros();
                t[count] = 0;
            } else {
                t[count] = micros() - t0;
            }

            accXvec[count] = accX;
            accYvec[count] = accY;
            accZvec[count] = accZ;

            prevTime = millis();
            count++;
        }
    }

    // Power off on timeout when on battery
    if ((millis() > timeout) && (M5.Axp.GetBatCurrent() < -1)) {
        M5.Axp.PowerOff();
    }
}

// ============================================================================
// SCREEN DRAWING FUNCTIONS
// ============================================================================

void drawStatisticsScreen() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setRotation(1);

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
    M5.Lcd.setCursor(5, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    M5.Lcd.print(timeBuf);
    
    M5.Lcd.setCursor(100, 1);
    M5.Lcd.print("Fs:");
    M5.Lcd.print((int)samplingFrequency);
    M5.Lcd.print("Hz");
    
    // Record indicator
    M5.Lcd.setCursor(180, 1);
    if (currentViewRecord < 0) {
        M5.Lcd.setTextColor(GREEN);
        M5.Lcd.print("LIVE");
    } else {
        M5.Lcd.setTextColor(YELLOW);
        M5.Lcd.print("#");
        M5.Lcd.print(currentViewRecord);
    }
    M5.Lcd.setTextColor(WHITE);
    
    // Row 2
    M5.Lcd.setCursor(5, th);
    M5.Lcd.print("STATISTICS  N=");
    M5.Lcd.print(SAMPLES);
    
    // Row 3 - headers
    M5.Lcd.setCursor(60, th * 2);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("X");
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 2);
    M5.Lcd.print("Y");
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 2);
    M5.Lcd.print("Z");
    
    // Row 4 - MAX
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, th * 3);
    M5.Lcd.print("MAX:");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(60, th * 3);
    M5.Lcd.print(maxX, 3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 3);
    M5.Lcd.print(maxY, 3);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 3);
    M5.Lcd.print(maxZ, 3);
    
    // Row 5 - MEAN
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, th * 4);
    M5.Lcd.print("MEAN:");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(60, th * 4);
    M5.Lcd.print(meanX, 3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 4);
    M5.Lcd.print(meanY, 3);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 4);
    M5.Lcd.print(meanZ, 3);
    
    // Row 6 - SD
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, th * 5);
    M5.Lcd.print("SD:");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(60, th * 5);
    M5.Lcd.print(sdX, 3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 5);
    M5.Lcd.print(sdY, 3);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 5);
    M5.Lcd.print(sdZ, 3);
    
    // Row 7 - MIN
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, th * 6);
    M5.Lcd.print("MIN:");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(60, th * 6);
    M5.Lcd.print(minX, 3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 6);
    M5.Lcd.print(minY, 3);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 6);
    M5.Lcd.print(minZ, 3);
    
    // Row 8 - PEAK
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, th * 7);
    M5.Lcd.print("PEAK/Hz:");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(60, th * 7);
    M5.Lcd.print(xM, 3);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(120, th * 7);
    M5.Lcd.print(yM, 3);
    M5.Lcd.setTextColor(BLUE);
    M5.Lcd.setCursor(180, th * 7);
    M5.Lcd.print(zM, 3);

    M5.Lcd.setTextColor(WHITE);
}

void drawTimeSeriesScreen() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(1);
    
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
    
    M5.Lcd.fillScreen(TFT_BLACK);
    for (int i = 1; i < SAMPLES; i++) {
        int yp1 = map(accXvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        int xp1 = map(i - 1, 0, SAMPLES, 0, DW - 1);
        int yp2 = map(accXvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        int xp2 = map(i, 0, SAMPLES, 0, DW - 1);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_RED);
        yp1 = map(accYvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        yp2 = map(accYvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_GREEN);
        yp1 = map(accZvec[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 0);
        yp2 = map(accZvec[i] * 100, minXYZ, maxXYZ, DH - 1, 0);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_BLUE);
    }
    
    M5.Lcd.setCursor(2, 2);
    M5.Lcd.print(maxXYZ / 100);
    M5.Lcd.print("g");
    M5.Lcd.setCursor(2, 72);
    M5.Lcd.print(minXYZ / 100);
    M5.Lcd.print("g");
    
    int yp1 = map(0, minXYZ, maxXYZ, DH - 1, 0);
    M5.Lcd.drawLine(0, yp1, DW - 1, yp1, TFT_WHITE);
    M5.Lcd.drawLine(0, 0, 0, DH - 1, TFT_WHITE);

    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(80, 1);
    M5.Lcd.print("Timeseries");
    
    // Timestamp
    M5.Lcd.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Lcd.setTextColor(GREEN);
    } else {
        M5.Lcd.setTextColor(YELLOW);
    }
    M5.Lcd.print(timeBuf);
    M5.Lcd.setTextColor(WHITE);
}

void drawSpectrumScreen() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(1);
    
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
    
    M5.Lcd.fillScreen(TFT_BLACK);
    for (int i = 1; i < (SAMPLES / 2); i++) {
        int yp1 = map(vRealX[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        int xp1 = map(i - 1, 0, (SAMPLES / 2), 0, DW - 1);
        int yp2 = map(vRealX[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        int xp2 = map(i, 0, (SAMPLES / 2), 0, DW - 1);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_RED);
        yp1 = map(vRealY[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        yp2 = map(vRealY[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_GREEN);
        yp1 = map(vRealZ[i - 1] * 100, minXYZ, maxXYZ, DH - 1, 10);
        yp2 = map(vRealZ[i] * 100, minXYZ, maxXYZ, DH - 1, 10);
        M5.Lcd.drawLine(xp1, yp1, xp2, yp2, TFT_BLUE);
    }
    
    if (majAxis == 1) {
        int xMplot = map(xM, 0, (SAMPLES / 2), 0, DW - 1);
        M5.Lcd.setCursor(xMplot, 1);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.print(xM);
    }
    if (majAxis == 2) {
        int yMplot = map(yM, 0, (SAMPLES / 2), 0, DW - 1);
        M5.Lcd.setCursor(yMplot, 1);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.print(yM);
    }
    if (majAxis == 3) {
        int zMplot = map(zM, 0, (SAMPLES / 2), 0, DW - 1);
        M5.Lcd.setCursor(zMplot, 1);
        M5.Lcd.setTextColor(TFT_BLUE);
        M5.Lcd.print(zM);
    }
    
    int yp1 = map(0, minXYZ, maxXYZ, DH - 1, 0);
    M5.Lcd.drawLine(0, yp1, DW - 1, yp1, TFT_WHITE);
    M5.Lcd.drawLine(0, 0, 0, DH - 1, TFT_WHITE);

    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(80, 1);
    M5.Lcd.print("Spectrum");
    
    // Timestamp indicator
    M5.Lcd.setCursor(160, 1);
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Lcd.setTextColor(GREEN);
    } else {
        M5.Lcd.setTextColor(YELLOW);
    }
    M5.Lcd.print(timeBuf);
    M5.Lcd.setTextColor(WHITE);
}

void drawCorrelationScreen() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(1);
    
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
    M5.Lcd.drawLine(aX1, 15, aX1, 135, WHITE);
    M5.Lcd.drawLine(0, aY1, 119, aY1, WHITE);

    int aX2 = map(0, minXYZ, maxXYZ, 120, 239);
    int aY2 = map(0, minXYZ, maxXYZ, 135, 15);
    M5.Lcd.drawLine(aX2, 15, aX2, 135, WHITE);
    M5.Lcd.drawLine(120, aY2, 239, aY2, WHITE);
    
    for (int i = 0; i < SAMPLES; i++) {
        int yp1 = map(accYvec[i] * 100, minXYZ, maxXYZ, 135, 15);
        int xp1 = map(accXvec[i] * 100, minXYZ, maxXYZ, 0, 119);
        M5.Lcd.drawPixel(xp1, yp1, ORANGE);
        int yp2 = map(accZvec[i] * 100, minXYZ, maxXYZ, 135, 15);
        int xp2 = map(accXvec[i] * 100, minXYZ, maxXYZ, 120, 239);
        M5.Lcd.drawPixel(xp2, yp2, MAGENTA);
    }
    
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(5, 1);
    M5.Lcd.print("X-Y corr");
    M5.Lcd.setCursor(125, 1);
    M5.Lcd.print("X-Z corr");
}

void drawSpectrogramScreen() {
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.fillScreen(TFT_BLACK);

    // Title with timestamp
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(2, 2);
    M5.Lcd.print("Spectro ");
    
    char timeBuf[16];
    formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
    if (currentViewRecord < 0) {
        M5.Lcd.setTextColor(GREEN);
    } else {
        M5.Lcd.setTextColor(YELLOW);
    }
    M5.Lcd.print(timeBuf);
    M5.Lcd.setTextColor(WHITE);

    if (specSegments <= 0) {
        M5.Lcd.setCursor(2, 20);
        M5.Lcd.print("No data yet");
    } else {
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
                M5.Lcd.fillRect(x0, yTop, w, yBottom - yTop + 1, c);
            }
        }

        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(2, DH - th);
        M5.Lcd.print("0 Hz");

        M5.Lcd.setCursor(DW - 60, DH - th);
        M5.Lcd.print((int)(samplingFrequency / 2));
        M5.Lcd.print(" Hz");
    }
}

void drawStorageScreen() {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextColor(WHITE);

    // Title with current RTC time
    M5.Lcd.setCursor(5, 1);
    M5.Lcd.print("STORAGE  RTC:");
    RecordDateTime now;
    getRtcTime(&now);
    char timeBuf[16];
    formatDateTimeShort(&now, timeBuf, sizeof(timeBuf));
    M5.Lcd.print(timeBuf);

    M5.Lcd.setCursor(5, th * 2);
    M5.Lcd.print("Records: ");
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.print(storedRecordCount);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("/");
    M5.Lcd.print(MAX_RECORDS);

    M5.Lcd.setCursor(5, th * 3);
    M5.Lcd.print("Flash: ");
    if (storageReady) {
        int usedKB = LittleFS.usedBytes() / 1024;
        int totalKB = LittleFS.totalBytes() / 1024;
        int pct = (usedKB * 100) / totalKB;
        M5.Lcd.print(usedKB);
        M5.Lcd.print("/");
        M5.Lcd.print(totalKB);
        M5.Lcd.print(" KB (");
        M5.Lcd.print(pct);
        M5.Lcd.print("%)");
    } else {
        M5.Lcd.setTextColor(RED);
        M5.Lcd.print("NOT READY");
        M5.Lcd.setTextColor(WHITE);
    }

    M5.Lcd.setCursor(5, th * 4);
    M5.Lcd.print("View: ");
    if (currentViewRecord < 0) {
        M5.Lcd.setTextColor(GREEN);
        M5.Lcd.print("LIVE");
    } else {
        M5.Lcd.setTextColor(YELLOW);
        M5.Lcd.print("#");
        M5.Lcd.print(currentViewRecord);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.print(" @ ");
        formatDateTimeShort(&currentRecordTime, timeBuf, sizeof(timeBuf));
        M5.Lcd.print(timeBuf);
    }
    M5.Lcd.setTextColor(WHITE);

    // Instructions
    M5.Lcd.setCursor(5, th * 6);
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.print("BtnA:");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print(" Browse records");

    M5.Lcd.setCursor(5, th * 7);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("Long BtnB:");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print(" CLEAR ALL");
}
