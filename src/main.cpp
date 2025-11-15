#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>

#include <operation_header.pb.h>
#include <logger_state.pb.h>

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

volatile bool logging_disconnected = false; 

char log_name[32];
File logFile;

IntervalTimer loggerTimer;
struct SensorPacket {
    unsigned long timestamp;
    uint32_t shock1, shock2;
};
volatile SensorPacket latestSample;
volatile bool newSampleReady = false;

constexpr size_t LOG_BUFFER_SIZE = 65536;
constexpr size_t MESSAGE_BUFFER_SIZE = 512; 
constexpr size_t PROTO_DELIMITER_LENGTH = 5;
constexpr size_t PROTO_HEADER_MESSAGE_ID = 0;
struct LogBuffer {
    char buffer[LOG_BUFFER_SIZE]; 
    size_t idx;
    bool full; 
};

/**** Status Variables ****/
bool sd_initialized = false;

uint8_t cur_buffer_num = 0; 
LogBuffer double_buffer[2]; 
uint8_t message_buffer[MESSAGE_BUFFER_SIZE];

LoggerState logger_state = LoggerState_init_default;

size_t encode_pb_message(uint8_t buffer[], 
                         size_t buffer_length, 
                         uint8_t id,
                         const pb_msgdesc_t *fields,
                         const void *message_struct) 
{
    // Serialize message
    pb_ostream_t ostream = pb_ostream_from_buffer(
        buffer + PROTO_DELIMITER_LENGTH, buffer_length - PROTO_DELIMITER_LENGTH);
    pb_encode(&ostream, fields, message_struct);

    size_t message_length = ostream.bytes_written;

    // Create message delimiter
    char delimiter[PROTO_DELIMITER_LENGTH + 1];
    snprintf(delimiter, PROTO_DELIMITER_LENGTH + 1, "%01X%04X", id,
            message_length);
    memcpy(buffer, delimiter, PROTO_DELIMITER_LENGTH);
    message_length += PROTO_DELIMITER_LENGTH;

    return message_length;
}

constexpr uint8_t DOUBLE_BUFFER_SUCCESS = 0;
constexpr uint8_t DOUBLE_BUFFER_FULL_ERROR = 1;
constexpr uint8_t DOUBLE_BUFFER_INDEX_ERROR = 2;

uint8_t write_to_double_buffer(uint8_t data[], 
                          size_t data_length,
                          LogBuffer double_buffer[2], 
                          uint8_t *buffer_num,
                          bool split) 
{
    LogBuffer *cur_buffer = &double_buffer[*buffer_num];

    if (cur_buffer->full) 
    {
        // If current buffer is full then something is wrong
        return DOUBLE_BUFFER_FULL_ERROR;
    } 
    else if (cur_buffer->idx + data_length > LOG_BUFFER_SIZE) 
    {
    // If data_length exceeds remaining space in buffer
    size_t remaining_space = 0;
    if (split) {
        // Split data across the two buffers
        remaining_space = LOG_BUFFER_SIZE - cur_buffer->idx;
        memcpy(cur_buffer->buffer + cur_buffer->idx, data, remaining_space);
        cur_buffer->idx = LOG_BUFFER_SIZE;
    }
    // Switch to the other buffer
    cur_buffer->full = true;
    *buffer_num = !(*buffer_num);
    cur_buffer = &double_buffer[*buffer_num];

    if (cur_buffer->idx != 0) 
    {
        // If new buffer doesn't start at the beginning then something is wrong
        return DOUBLE_BUFFER_INDEX_ERROR;
    } 
    else 
    {
        // Write data to new buffer
        memcpy(cur_buffer->buffer, data + remaining_space,
                data_length - remaining_space);
        cur_buffer->idx += data_length;
    }
    } else {
    // If data fits in current buffer then write it
    memcpy(cur_buffer->buffer + cur_buffer->idx, data, data_length);
    cur_buffer->idx += data_length;
    }

    return DOUBLE_BUFFER_SUCCESS;
}


// --- ISR: fast, non-blocking ---
void logger_function() 
{
    latestSample.timestamp = micros();    
    latestSample.shock1 = analogRead(SHOCK_1);
    latestSample.shock2 = analogRead(SHOCK_2);

    newSampleReady = true;
}

void setup() 
{
    Serial.begin(115200);
    pinMode(LED1_PIN, OUTPUT);
    setSyncProvider(get_teensy3_time);

    sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
    if (!sd_initialized) {
        Serial.println("Warning: SD failed to initialize");
    } else {
        /* Create log file and appropriate name based on RTC. */
        bool rtc_set = timeStatus() == timeSet && year() > 2021;
        if (!rtc_set) {
            Serial.println("Warning: Failed to sync time with RTC");
            strncpy(log_name, "log_unknown_time.bin", sizeof(log_name));
            logFile = SD.open("log_unknown_time.bin", FILE_WRITE);
        } else {
            sprintf(log_name, 
                    "log_%04d-%02d-%02d_%02d-%02d-%02d.bin", 
                    year(),
                    month(), 
                    day(), 
                    hour(), 
                    minute(), 
                    second());
            logFile = SD.open(log_name, FILE_WRITE);
        }

        // Create log file with timestamp
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

        Serial.printf("Info: Logging to %s\n", log_name); 
        Serial.printf("Info: Logging to %s\n", log_name);
        logFile = SD.open(log_name, FILE_WRITE);
        if (!logFile) {
            Serial.println("Warning: Log file was not opened! (Sarah)");
        }

        OperationHeader operation_header;
        operation_header.timestamp = now();
        operation_header.clock_us = micros();
    }
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
       /* TODO: Add timeout here */
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
        /* TODO: Add timeout here */
        while (true) {
            digitalWrite(LED1_PIN, HIGH);
            delay(250);
            digitalWrite(LED1_PIN, LOW);
            delay(250);
        }
    }

    // Start timer at 200 Hz → 5000 µs
    loggerTimer.priority(255); // highest priority

    OperationHeader operation_header; 
    operation_header.timestamp = now(); 
    operation_header.clock_us = micros(); 

    size_t message_length = encode_pb_message(message_buffer, 
                                              MESSAGE_BUFFER_SIZE, 
                                              PROTO_HEADER_MESSAGE_ID, 
                                              &OperationHeader_msg, 
                                              &operation_header);

    size_t num_bytes_written = logFile.write(message_buffer, message_length);
    logFile.flush(); 

    /* Attach logging interrupt */
    loggerTimer.begin(logger_function, 5000);
}

void loop() 
{
    if (newSampleReady) 
    {
        newSampleReady = false;

        logger_state = LoggerState_init_default;
        logger_state.cycle_start_us = micros();
        
        // Convert analog to units
        float distance1 = (latestSample.shock1 / 4095.0) * 250.0;
        float distance2 = (latestSample.shock2 / 4095.0) * 250.0;
        
        // Log to SD
        if (logFile) {
            logger_state.shock1_mm = distance1; 
            logger_state.shock2_mm = distance2; 
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