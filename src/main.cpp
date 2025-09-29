#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <USBHost_t36.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IMU setup (kept for compatibility, not used in loop)
#define IMU_SCL 24
#define IMU_SDA 25
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// GPS USB setup (kept for compatibility)
USBHost myusb;
USBSerial_BigBuffer gpsSerial(myusb);

#define LED1_PIN 4
#define LED2_PIN 9
#define LED3_PIN 10
#define BPS A12

time_t get_teensy3_time() { return Teensy3Clock.get(); }

char log_name[32];
File logFile;

// For OLED throttling
unsigned long lastOled = 0;

void setup() {
    delay(2000);
    Serial.begin(115200);
    pinMode(LED1_PIN, OUTPUT);

    // Sync RTC
    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;

    // Init SD
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed!");
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }

    // Filename from RTC
    if (!rtc_set) {
        sprintf(log_name, "log_unknown_time.csv");
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
                year(), month(), day(),
                hour(), minute(), second());
    }

    // Open file once, keep it open
    logFile = SD.open(log_name, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp (us),CarRoll,CarPitch,CarYaw,"
                        "RollGrad,ax,ay,az,ForwardAccel,LateralAccel,HeaveAccel,"
                        "PitchRate,RollRate,YawRate,"
                        "qw,qx,qy,qz,"
                        "ShockPot1,ShockPot2,BrakeADC");
        logFile.flush();
    } else {
        Serial.println("Failed to create log file");
        while (true);
    }

    // Keep IMU/GPS/OLED init (not used in loop)
    Wire2.begin();
    myusb.begin();

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 init failed!");
    }
    if (!bno08x.begin_I2C(0x4A, &Wire2)) {
        Serial.println("IMU init failed (ignored)");
    }

    Serial.print("Setup complete — logging to ");
    Serial.println(log_name);
}

void loop() {
    // Brake pressure raw ADC
    int brakeADC = analogRead(BPS);
    unsigned long u_timestamp = micros();

    // Build CSV row into a buffer (all other values = 0)
    char row[128];
    sprintf(row, "%lu,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,%d",
            u_timestamp, brakeADC);

    // Write once
    if (logFile) {
        logFile.println(row);

        // Flush occasionally (not every row — too slow)
        static uint16_t counter = 0;
        if (++counter >= 100) {  // flush every 100 samples
            logFile.flush();
            counter = 0;
        }
    }

    // Serial monitor (can slow logging, optional)
    Serial.print("µs: ");
    Serial.print(u_timestamp);
    Serial.print(" | Brake ADC: ");
    Serial.println(brakeADC);

    // OLED update ~2 Hz
    if (millis() - lastOled > 500) {
        lastOled = millis();
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println("Brake ADC:");
        display.setTextSize(2);
        display.setCursor(0, 20);
        display.println(brakeADC);
        display.display();
    }

    delay(1); // keep ~1 ms pacing
}
