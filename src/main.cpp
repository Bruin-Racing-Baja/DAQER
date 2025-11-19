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
#define LED1_PIN 4
#define LED2_PIN 9
#define BUFFER_SIZE 50

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

time_t get_teensy3_time() { return Teensy3Clock.get(); }
IntervalTimer loggerTimer;

char log_name[32];
File logFile;

struct TimePacket {
    unsigned long ts;
};

volatile TimePacket buffer[BUFFER_SIZE];
volatile uint8_t head = 0;
volatile uint8_t tail = 0;

struct IMUPacket {
    float ax, ay, az;
    float qw, qx, qy, qz;
};

IMUPacket latestIMU;

void logger_function() {
    uint8_t next = (head + 1) % BUFFER_SIZE;
    if (next == tail) return;
    buffer[head].ts = micros();
    head = next;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED2_PIN, OUTPUT);

    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;

    if (!SD.begin(BUILTIN_SDCARD)) {
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }

    if (!rtc_set) {
        strcpy(log_name, "log_unknown_time.csv");
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
                year(), month(), day(), hour(), minute(), second());
    }

    if (!SD.exists(log_name)) {
        File f = SD.open(log_name, FILE_WRITE);
        if (f) {
            f.println("timestamp,ax,ay,az,qw,qx,qy,qz");
            f.close();
        } else {
            Serial.println("Failed to create new log file");
            while (true) {
                digitalWrite(LED2_PIN, HIGH);
                delay(250);
                digitalWrite(LED2_PIN, LOW);
                delay(250);
            }
        }
    }

    logFile = SD.open(log_name, FILE_WRITE);
    if (!logFile) {
        Serial.println("Failed to open log file for writing");
        while (true);
    }

    Wire2.begin();
    bno08x.begin_I2C(0x4A, &Wire2);
    bno08x.enableReport(SH2_CAL_ACCEL, 2500);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 2500);

    loggerTimer.begin(logger_function, 5000);
    loggerTimer.priority(255);
}

void loop() {
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_CAL_ACCEL:
                latestIMU.ax = sensorValue.un.accelerometer.x;
                latestIMU.ay = sensorValue.un.accelerometer.y;
                latestIMU.az = sensorValue.un.accelerometer.z;
                break;
            case SH2_ROTATION_VECTOR:
                latestIMU.qw = sensorValue.un.rotationVector.real;
                latestIMU.qx = sensorValue.un.rotationVector.i;
                latestIMU.qy = sensorValue.un.rotationVector.j;
                latestIMU.qz = sensorValue.un.rotationVector.k;
                break;
        }
    }

    while (tail != head) {
        TimePacket pkt;
        noInterrupts();
        pkt.ts = buffer[tail].ts;
        tail = (tail + 1) % BUFFER_SIZE;
        interrupts();

        logFile.print(pkt.ts); logFile.print(",");
        logFile.print(latestIMU.ax); logFile.print(",");
        logFile.print(latestIMU.ay); logFile.print(",");
        logFile.print(latestIMU.az); logFile.print(",");
        logFile.print(latestIMU.qw); logFile.print(",");
        logFile.print(latestIMU.qx); logFile.print(",");
        logFile.print(latestIMU.qy); logFile.print(",");
        logFile.println(latestIMU.qz);
    }

    static uint32_t lastFlush = 0;
    if (millis() - lastFlush > 100) {
        lastFlush = millis();
        logFile.flush();
    }
}
