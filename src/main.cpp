#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <USBHost_t36.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>

#define NCIR_ADDR 0x5B

uint16_t result;
float temp = 0.0;
File logFile;
char log_name[32];

time_t get_teensy3_time() { return Teensy3Clock.get(); }

void setup() {
  delay(2000);
  Serial.begin(115200);
  Wire.begin();

  if(!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("SD failed!");
      while(1);
  }

  setSyncProvider(get_teensy3_time);
  bool rtc_set = timeStatus() == timeSet && year() > 2021;

  if (!rtc_set) {
      Serial.println("Warning: Failed to sync time with RTC");
      strcpy(log_name, "log_unknown_time.csv");
  } else {
      sprintf(log_name, "log_%04d-%02d-%02d_%02d-%02d-%02d.csv",
              year(), month(), day(), hour(), minute(), second());
  }

  if (!SD.exists(log_name)) {
      logFile = SD.open(log_name, FILE_WRITE);
      if (logFile) {
          logFile.println("Timestamp,temp");
          logFile.close();
      } else {
          Serial.println("Failed to create new log file");
      }
  }
}

void loop() {
  char timestamp[32];
  unsigned long ms = millis();
  time_t now = ms / 1000;
  int ms_part = ms % 1000;

  sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          year(now), month(now), day(now),
          hour(now), minute(now), second(now), ms_part);

  Wire.beginTransmission(NCIR_ADDR);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom(NCIR_ADDR, 2);

  result = Wire.read();
  result |= Wire.read() << 8;
  temp = result * 0.02 - 273.15;

  Serial.println(temp);

  logFile = SD.open(log_name, FILE_WRITE);
  if (logFile) {
      logFile.print(micros());
      logFile.print(",");
      logFile.println(temp);
      logFile.close();
  } else {
      Serial.println("Failed to write to log");
  }
}
