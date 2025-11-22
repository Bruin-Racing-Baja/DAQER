#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SHOCK_1 A2
#define SHOCK_2 A13
#define LED1_PIN 4
#define LED2_PIN 9

time_t get_teensy3_time() { return Teensy3Clock.get(); }

char log_name[32];
File logFile;

IntervalTimer loggerTimer;

struct SensorPacket {
    unsigned long timestamp;
    uint16_t shock1_raw, shock2_raw;
};

volatile SensorPacket latestSample;
volatile bool newSampleReady = false;

uint16_t sample_count = 0;

void logger_function() {
    latestSample.timestamp = micros();
    latestSample.shock1_raw = analogRead(SHOCK_1);
    latestSample.shock2_raw = analogRead(SHOCK_2);
    newSampleReady = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(100);
            digitalWrite(LED1_PIN, LOW);
            delay(100);
        }
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();

    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;

    // Initialize SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("SD Card");
        display.println("Init Failed!");
        display.display();
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD Card OK");
    display.display();
    delay(500);

    // Create log file name
    if (!rtc_set) {
        strcpy(log_name, "log_unknown_time.csv");
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
                year(), month(), day(), hour(), minute(), second());
    }

    // Create log file with header
    if (!SD.exists(log_name)) {
        File f = SD.open(log_name, FILE_WRITE);
        if (f) {
            f.println("timestamp,shock1_mm,shock2_mm");
            f.close();
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Log Created");
            display.println(log_name);
            display.display();
        } else {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Failed to");
            display.println("create log file");
            display.display();
            while (true) {
                digitalWrite(LED2_PIN, HIGH);
                delay(250);
                digitalWrite(LED2_PIN, LOW);
                delay(250);
            }
        }
    }
    delay(500);

    logFile = SD.open(log_name, FILE_WRITE);
    if (!logFile) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Failed to open");
        display.println("log file");
        display.display();
        while (true);
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Logging");
    display.display();
    delay(1000);
    
    display.clearDisplay();
    display.display();

    // 200 Hz (5000 microseconds = 5ms)
    loggerTimer.priority(255);
    loggerTimer.begin(logger_function, 5000);
}

void loop() {
    if (newSampleReady) {
        newSampleReady = false;

        float distance1 = (latestSample.shock1_raw / 4095.0) * 250.0;
        float distance2 = (latestSample.shock2_raw / 4095.0) * 250.0;

        // Write to SD 
        char buffer[64];
        sprintf(buffer, "%lu,%.2f,%.2f", latestSample.timestamp, distance1, distance2);
        logFile.println(buffer);

        // Flush every 50 samples 
        sample_count++;
        if (sample_count >= 50) {
            logFile.flush();
            sample_count = 0;
        }
    }
}