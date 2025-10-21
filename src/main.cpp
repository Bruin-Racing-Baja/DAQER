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

// IMU setup
#define IMU_SCL 24
#define IMU_SDA 25
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// GPS USB setup
USBHost myusb;
USBSerial_BigBuffer gpsSerial(myusb);
#define UPDATE_INTERVAL 1 //Change sampling frequency (1000 hz right now)
uint32_t lastUpdate = 0;

#define LED1_PIN 4
#define LED2_PIN 9
#define LED3_PIN 10
#define BPS A12

time_t get_teensy3_time() { return Teensy3Clock.get(); }
String lastLine = "";
bool gpsFix = false;
bool sdFail = false;
bool logFail = false;
bool imuFail = false;
int hall;
int shock_pot;
int shock_pot2;
int bps;
int voltageDividerRatio = 0.735;
float roll = 0;
float pitch = 0;
float yaw = 0;
float lateralAccel = 0;
float forwardAccel = 0;
float heaveAccel = 0;
float ax = 0;
float ay = 0;
float az = 0;
float lax = 0;
float lay = 0;
float laz = 0;
float gyx = 0;
float qw = 0;
float qx = 0;
float qy = 0;
float qz = 0;
float rw = 0;
float rx = 0;
float ry = 0;
float rz = 0;
float pitchRate = 0;
float rollRate = 0;
float yawRate = 0;
float gpsLat = 0.0;
float gpsLon = 0.0;
char log_name[32];



File logFile;


void setup() {
    delay(2000);  
    Serial.begin(115200);
    
    // Initialize I2C for IMU
    Wire2.begin();
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

    if(!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD failed!");
        while(true){
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }
    
    if (!SD.exists(log_name)) {
        logFile = SD.open(log_name, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp,ax,ay,az,ForwardAccel,LateralAccel,HeaveAccel,qw,qx,qy,qz");
        logFile.close();
    } else {
        Serial.println("Failed to create new log file");
        while(true){
            digitalWrite(LED2_PIN, HIGH);
            delay(250);
            digitalWrite(LED2_PIN, LOW);
            delay(250);
        }
        logFail = true;
        while(true);
    }
}

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 initialization failed!");
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("Initializing IMU...");
    display.display();

    // Initialize IMU
    if (!bno08x.begin_I2C(0x4A, &Wire2)) {
        Serial.println("BNO085 initialization failed!");
        display.println("IMU Error!");
        display.display();
        while(true){
            digitalWrite(LED3_PIN, HIGH);
            delay(250);
            digitalWrite(LED3_PIN, LOW);
            delay(250);
        }
    }
    bno08x.enableReport(SH2_CAL_ACCEL, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);

    
    display.clearDisplay();
    display.setCursor(10, 20);
    display.println("IMU + GPS Ready!");
    display.display();
    delay(1000);
}


void loop() {
    myusb.Task();

    // IMU data
    if (millis() - lastUpdate > UPDATE_INTERVAL) {
        lastUpdate = millis();
        while (bno08x.getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_CAL_ACCEL: {
                    ax = sensorValue.un.accelerometer.x;
                    ay = sensorValue.un.accelerometer.y;
                    az = sensorValue.un.accelerometer.z;
                break;
                }

                case SH2_ROTATION_VECTOR: {
                    //Quaternions in IMU frame
                    qw = sensorValue.un.rotationVector.real;
                    qx = sensorValue.un.rotationVector.i;
                    qy = sensorValue.un.rotationVector.j;
                    qz = sensorValue.un.rotationVector.k;
                    uint8_t acc = sensorValue.un.rotationVector.accuracy;
                    break;
                }

                case SH2_LINEAR_ACCELERATION: {
                    lax = sensorValue.un.linearAcceleration.x;
                    lay = sensorValue.un.linearAcceleration.y;
                    laz = sensorValue.un.linearAcceleration.z;
                    break;
                }
            }
        }

    
        

        // Display on OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println("Data!");
        
        display.print("ax: "); 
        display.println(ax);
        display.print("ay: "); 
        display.println(ay);
        display.print("az: "); 
        display.println(az);
        display.print("lax: "); 
        display.println(ax);
        display.print("lay: "); 
        display.println(ay);
        display.print("laz: "); 
        display.println(az);
        display.display();

        //serial prints
        char timestamp[32];
        unsigned long ms = millis();
        time_t now = ms / 1000;
        int ms_part = ms % 1000;
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
        year(now), month(now), day(now),
        hour(now), minute(now), second(now), ms_part);

        //Serial.print(timestamp);


        Serial.print(" | ax: "); Serial.print(ax, 2);
        Serial.print(" | ay: "); Serial.print(ay, 2);
        Serial.print(" | az: "); Serial.print(az, 2);

        Serial.print(" | ForAcc: "); Serial.print(lax, 2);
        Serial.print(" | LatAcc: "); Serial.print(lay, 2);
        Serial.print(" | HAcc: "); Serial.print(laz, 2);

        Serial.print(" | q_car = [");
        Serial.print(qw, 4); Serial.print(", ");
        Serial.print(qx, 4); Serial.print(", ");
        Serial.print(qy, 4); Serial.print(", ");
        Serial.println(qz, 4); Serial.print("]");


        //logging to sd card
        logFile = SD.open(log_name, FILE_WRITE);
        if (logFile) {

            logFile.print(timestamp); logFile.print(",");

            logFile.print(ax); logFile.print(",");
            logFile.print(ay); logFile.print(",");
            logFile.print(az); logFile.print(",");

            logFile.print(lax); logFile.print(",");
            logFile.print(lay); logFile.print(",");
            logFile.print(laz); logFile.print(",");

            logFile.print(qw); logFile.print(",");
            logFile.print(qx); logFile.print(",");
            logFile.print(qy); logFile.print(",");
            logFile.println(qz); 
            

            logFile.close();  
        } else {
            Serial.println("Failed to write to log");
        }

    }
}