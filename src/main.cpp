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

#define SHOCK_1 A2
#define SHOCK_2 A13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

time_t get_teensy3_time() { return Teensy3Clock.get(); }

char log_name[32];
File logFile;

uint64_t cur_time = 0; 
uint64_t last_analog_read_time = 0;

bool imu_a_ready = false; 
bool imu_o_ready = false; 
bool something_logged = false; 

struct IMUPacket {
    float ax, ay, az;
    float qw, qx, qy, qz;
};

struct ShockPot {
    float dist1; 
    float dist2; 
};

IMUPacket latestIMU;
ShockPot latestShockPot; 

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
            f.println("timestamp,ax,ay,az,qw,qx,qy,qz,shock1,shock2");
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
}

void loop() {
    cur_time = micros();
    char str_time[64];
    sprintf(str_time, "%llu", (unsigned long long)cur_time);
    String csv_input = String(str_time) + ","; 

    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) { 
            case SH2_CAL_ACCEL:
                imu_a_ready = true; 
                something_logged = true; 
                latestIMU.ax = sensorValue.un.accelerometer.x;
                latestIMU.ay = sensorValue.un.accelerometer.y;
                latestIMU.az = sensorValue.un.accelerometer.z;
                break;
            case SH2_ROTATION_VECTOR:
                imu_o_ready = true; 
                something_logged = true; 
                latestIMU.qw = sensorValue.un.rotationVector.real;
                latestIMU.qx = sensorValue.un.rotationVector.i;
                latestIMU.qy = sensorValue.un.rotationVector.j;
                latestIMU.qz = sensorValue.un.rotationVector.k;
                break;
        }
    }

    if (imu_a_ready) {
        csv_input += String(latestIMU.ax) + "," +
                        String(latestIMU.ay) + "," +
                        String(latestIMU.az) + ",";
    } else {
        csv_input += ",,,";
    }

    if (imu_o_ready) {
        csv_input += String(latestIMU.qw) + "," +
                        String(latestIMU.qx) + "," +
                        String(latestIMU.qy) + "," +
                        String(latestIMU.qz) + ",";
    } else {
        csv_input += ",,,,";
    }

    /* Read from all analog sensors */                 
    if ((unsigned long long)cur_time - last_analog_read_time >= 5000) {
        something_logged = true; 
        last_analog_read_time = micros(); 

        uint32_t shock1 = analogRead(SHOCK_1);
        uint32_t shock2 = analogRead(SHOCK_2);
        latestShockPot.dist1 = (shock1 / 4095.0) * 250.0;
        latestShockPot.dist2 = (shock2 / 4095.0) * 250.0;
        
        csv_input += String(latestShockPot.dist1) + "," + String(latestShockPot.dist2);
    } else {
        csv_input += ",";
    }

    if (something_logged) {
        Serial.println("Logging Something.");
        Serial.println(csv_input);
        logFile.println(csv_input);
        logFile.flush();
    }

    something_logged = false; 
}
