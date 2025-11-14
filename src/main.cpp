#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

// --- OLED setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- IMU setup ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// --- Pins ---
#define SHOCK_1 A2
#define SHOCK_2 A13
#define BPS     A12
#define LED1_PIN 4
#define LED2_PIN 9
#define BUFFER_SIZE 50

time_t get_teensy3_time() { return Teensy3Clock.get(); }

// --- SD logging ---
char log_name[32];
File logFile;

// --- Timer ---
IntervalTimer loggerTimer;

// --- Data struct ---
struct SensorPacket {
    unsigned long timestamp;
    float ax, ay, az;
    float lax, lay, laz;
    float qw, qx, qy, qz;
    int shock1, shock2;
    int bps;
};
volatile SensorPacket latestSample;
volatile bool newSampleReady = false;

// --- ISR: fast, non-blocking ---
void logger_function() {
    latestSample.timestamp = micros();
    Serial.println(latestSample.timestamp);
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_CAL_ACCEL:
                latestSample.ax = sensorValue.un.accelerometer.x;
                latestSample.ay = sensorValue.un.accelerometer.y;
                latestSample.az = sensorValue.un.accelerometer.z;
                break;
            case SH2_LINEAR_ACCELERATION:
                latestSample.lax = sensorValue.un.linearAcceleration.x;
                latestSample.lay = sensorValue.un.linearAcceleration.y;
                latestSample.laz = sensorValue.un.linearAcceleration.z;
                break;
            case SH2_ROTATION_VECTOR:
                latestSample.qw = sensorValue.un.rotationVector.real;
                latestSample.qx = sensorValue.un.rotationVector.i;
                latestSample.qy = sensorValue.un.rotationVector.j;
                latestSample.qz = sensorValue.un.rotationVector.k;
                break;
        }
    }

    // Analog reads (fast)
    latestSample.shock1 = analogRead(SHOCK_1);
    latestSample.shock2 = analogRead(SHOCK_2);
    latestSample.bps = analogRead(BPS);

    newSampleReady = true;
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    pinMode(LED1_PIN, OUTPUT);
    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;
    if (!rtc_set) {
        Serial.println("Warning: Failed to sync time with RTC");
        logFile = SD.open("log_unknown_time.csv", FILE_WRITE);
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv", year(), month(), day(), hour(), minute(), second());
        logFile = SD.open(log_name, FILE_WRITE);
    }
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("Initializing...");
    display.display();

    // Initialize IMU
    Wire2.begin();
    if (!bno08x.begin_I2C(0x4A, &Wire2)) {
        display.println("IMU Error!");
        display.display();
    }
    bno08x.enableReport(SH2_CAL_ACCEL, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);

    // Initialize SD
    if (!SD.begin(BUILTIN_SDCARD)) {
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }

    // Create log file with timestamp
    // sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
    //         year(), month(), day(), hour(), minute(), second());
    //logFile = SD.open(log_name, FILE_WRITE);
    if (!SD.exists(log_name)) {
        logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            logFile.println("Timestamp,ax,ay,az,ForwardAccel,LateralAccel,HeaveAccel,ShockPot1,ShockPot2,BrakePressure,qw,qx,qy,qz");
            logFile.close();
        } else {
            Serial.println("Failed to create new log file");
            while(true){
                digitalWrite(LED2_PIN, HIGH);
                delay(250);
                digitalWrite(LED2_PIN, LOW);
                delay(250);
            }
        }
    }

    // Start timer at 200 Hz → 5000 µs
    loggerTimer.priority(255); // highest priority
    loggerTimer.begin(logger_function, 5000);
}

// --- Main loop ---
void loop() {
    if (newSampleReady) {
        
        newSampleReady = false;

        // Convert analog to units
        float distance1 = (latestSample.shock1 / 4095.0) * 250.0;
        float distance2 = (latestSample.shock2 / 4095.0) * 250.0;
        float pressure = ((latestSample.bps * 3.3 / 1023.0) - 0.5) * (2900.0 / 4);

        // Log to SD
        File logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            logFile.print(latestSample.timestamp); logFile.print(",");
            logFile.print(latestSample.ax); logFile.print(",");
            logFile.print(latestSample.ay); logFile.print(",");
            logFile.print(latestSample.az); logFile.print(",");
            logFile.print(latestSample.lax); logFile.print(",");
            logFile.print(latestSample.lay); logFile.print(",");
            logFile.print(latestSample.laz); logFile.print(",");
            logFile.print(distance1); logFile.print(",");
            logFile.print(distance2); logFile.print(",");
            logFile.print(pressure); logFile.print(",");
            logFile.print(latestSample.qw); logFile.print(",");
            logFile.print(latestSample.qx); logFile.print(",");
            logFile.print(latestSample.qy); logFile.print(",");
            logFile.println(latestSample.qz);
            logFile.close();
        }

        
        static unsigned long lastOLED = 0;
        if (millis() - lastOLED > 50) {
            lastOLED = millis();
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("ax: "); display.println(latestSample.ax);
            display.print("ay: "); display.println(latestSample.ay);
            display.print("az: "); display.println(latestSample.az);
            display.display();
        }

        Serial.print("ax: "); Serial.println(latestSample.ax);
    }
}
