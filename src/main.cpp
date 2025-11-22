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
#define SHOCK_1 A2
#define SHOCK_2 A13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

time_t get_teensy3_time() { return Teensy3Clock.get(); }

char log_name[32];
File logFile;

uint64_t cur_time = 0; 
uint16_t sample_count = 0;

bool imu_initialized = false;

struct IMUPacket {
    float ax, ay, az;
    float qw, qx, qy, qz;
};

struct ShockPacket {
    float shock1_mm, shock2_mm;
};

IMUPacket latestIMU;
ShockPacket latestShock;

IntervalTimer shockTimer;


void shock_logger_function() {
    uint16_t shock1_raw = analogRead(SHOCK_1);
    uint16_t shock2_raw = analogRead(SHOCK_2);
    latestShock.shock1_mm = (shock1_raw / 4095.0) * 250.0;
    latestShock.shock2_mm = (shock2_raw / 4095.0) * 250.0;
}

void displayMessage(const char* line1, const char* line2 = "", const char* line3 = "") {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(line1);
    if (strlen(line2) > 0) display.println(line2);
    if (strlen(line3) > 0) display.println(line3);
    display.display();
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
    display.display();
    displayMessage("Initializing...");

    setSyncProvider(get_teensy3_time);
    bool rtc_set = timeStatus() == timeSet && year() > 2021;

    // Initialize SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
        displayMessage("SD Card", "Init Failed!");
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }
    displayMessage("SD Card OK");
    delay(500);

    // Create log file name
    if (!rtc_set) {
        strcpy(log_name, "log_unknown_time.csv");
    } else {
        sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
                year(), month(), day(), hour(), minute(), second());
    }

    // Create log file 
    if (!SD.exists(log_name)) {
        File f = SD.open(log_name, FILE_WRITE);
        if (f) {
            f.println("timestamp,ax,ay,az,qw,qx,qy,qz,shock1_mm,shock2_mm");
            f.close();
            displayMessage("Log Created", log_name);
        } else {
            displayMessage("Failed to", "create log file");
            while (true) {
                digitalWrite(LED1_PIN, HIGH);
                delay(500);
                digitalWrite(LED1_PIN, LOW);
                delay(500);
            }
        }
    }
    delay(500);

    logFile = SD.open(log_name, FILE_WRITE);
    if (!logFile) {
        displayMessage("Failed to open", "log file");
        while (true);
    }

    // Initialize IMU
    displayMessage("Init IMU...");
    Wire2.begin();
    Wire2.setClock(400000);
    if (bno08x.begin_I2C(0x4A, &Wire2)) {
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 1000);
        bno08x.enableReport(SH2_ROTATION_VECTOR, 1000);
        imu_initialized = true;
        displayMessage("IMU OK", "Linear Accel ", "Rotation Shock Pots");
    } else {
        displayMessage("IMU Init", "Failed!");
        while (true) {
            digitalWrite(LED2_PIN, HIGH);
            delay(100);
            digitalWrite(LED2_PIN, LOW);
            delay(100);
        }
    }
    delay(1000);
    
    display.clearDisplay();
    display.display();

    shockTimer.priority(255);
    shockTimer.begin(shock_logger_function, 2500);
}

void loop() {

    if (bno08x.getSensorEvent(&sensorValue)) {
        cur_time = micros();
        bool should_log = false;
        
        switch (sensorValue.sensorId) { 
            case SH2_LINEAR_ACCELERATION:
                latestIMU.ax = sensorValue.un.linearAcceleration.x;
                latestIMU.ay = sensorValue.un.linearAcceleration.y;
                latestIMU.az = sensorValue.un.linearAcceleration.z;
                should_log = true;
                break;
                
            case SH2_ROTATION_VECTOR:
                latestIMU.qw = sensorValue.un.rotationVector.real;
                latestIMU.qx = sensorValue.un.rotationVector.i;
                latestIMU.qy = sensorValue.un.rotationVector.j;
                latestIMU.qz = sensorValue.un.rotationVector.k;
                should_log = true;
                break;
        }
        
        if (should_log) {
            char buffer[160];
            sprintf(buffer, "%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                    (unsigned long long)cur_time,
                    latestIMU.ax, latestIMU.ay, latestIMU.az,
                    latestIMU.qw, latestIMU.qx, latestIMU.qy, latestIMU.qz,
                    latestShock.shock1_mm, latestShock.shock2_mm);
            logFile.println(buffer);
            
            sample_count++;
            if (sample_count >= 50) {
                logFile.flush();
                sample_count = 0;
            }
        }
    }
}