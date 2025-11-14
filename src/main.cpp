#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SHOCK_1 A2
#define SHOCK_2 A13
#define LED1_PIN 4

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

time_t get_teensy3_time() { return Teensy3Clock.get(); }
char log_name[40];
File logFile;

void setup() {
    delay(2000);
    Serial.begin(115200);

    pinMode(LED1_PIN, OUTPUT);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        while (1); 
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;

    if (!SD.begin(BUILTIN_SDCARD)) {
        while (true) {
            digitalWrite(LED1_PIN, HIGH); delay(200);
            digitalWrite(LED1_PIN, LOW);  delay(200);
        }
    }

    if (!rtc_set) {
        strcpy(log_name, "log_unknown_time.csv");
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
                year(), month(), day(),
                hour(), minute(), second());
    }

    if (!SD.exists(log_name)) {
        logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {
            logFile.println("Micros,Shock1_mm,Shock2_mm");
            logFile.close();
        }
    }

    Serial.println("Logging + OLED Ready.");
}

void loop() {
    int raw1 = analogRead(SHOCK_1);
    int raw2 = analogRead(SHOCK_2);

    float dist1_mm = (raw1 / 4095.0) * 250.0;
    float dist2_mm = (raw2 / 4095.0) * 250.0;

    unsigned long t_us = micros();

    logFile = SD.open(log_name, FILE_WRITE);
    if (logFile) {
        logFile.print(t_us); logFile.print(",");
        logFile.print(dist1_mm); logFile.print(",");
        logFile.println(dist2_mm);
        logFile.close();
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Shock Pot Distances");

    display.print("Shock 1: ");
    display.print(dist1_mm, 1);
    display.println(" mm");

    display.print("Shock 2: ");
    display.print(dist2_mm, 1);
    display.println(" mm");

    display.display();

    Serial.print("t: "); Serial.print(t_us);
    Serial.print(" | Raw1: "); Serial.print(raw1);
    Serial.print(" | Dist1(mm): "); Serial.print(dist1_mm, 2);
    Serial.print(" | Raw2: "); Serial.print(raw2);
    Serial.print(" | Dist2(mm): "); Serial.println(dist2_mm, 2);
    // ------------------------------

    delay(5); // ~200 Hz sampling
}
