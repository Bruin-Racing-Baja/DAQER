#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
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
#define BPS     A12
#define LED1_PIN 4
#define LED2_PIN 9
#define BUFFER_SIZE 50

time_t get_teensy3_time() { return Teensy3Clock.get(); }


char log_name[32];
File logFile;


IntervalTimer loggerTimer;


struct SensorPacket {
    unsigned long timestamp;
    int shock1, shock2;
};
volatile SensorPacket latestSample;
volatile bool newSampleReady = false;

// --- ISR: fast, non-blocking ---
void logger_function() {
    latestSample.timestamp = micros();
    //Serial.println(latestSample.timestamp);
    
    latestSample.shock1 = analogRead(SHOCK_1);
    latestSample.shock2 = analogRead(SHOCK_2);

    newSampleReady = true;
}


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
 
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("Initializing...");
    display.display();

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
            logFile.println("Timestamp,ShockPot1(mm),ShockPot2(mm)");
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


void loop() {
    if (newSampleReady) {
        
        newSampleReady = false;

        // Convert analog to units
        float distance1 = (latestSample.shock1 / 4095.0) * 250.0;
        float distance2 = (latestSample.shock2 / 4095.0) * 250.0;
        

        // Log to SD
        File logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            logFile.print(latestSample.timestamp); logFile.print(",");

            logFile.print(distance1); logFile.print(",");
            logFile.println(distance2); 
            logFile.close();
        }

        
        static unsigned long lastOLED = 0;
        if (millis() - lastOLED > 50) {
            lastOLED = millis();
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("s1: "); display.println(latestSample.shock1);
            display.print("s2: "); display.println(latestSample.shock2);
            display.display();
        }

        Serial.print("s1: "); Serial.println(latestSample.shock1);
        Serial.print("s2: "); Serial.println(latestSample.shock2);
    }
}