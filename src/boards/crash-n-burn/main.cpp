#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <cmath>

//----------------------------------------
// Include sensors
//----------------------------------------
#include "BMI088Accel.hpp"
#include "BMI088Gyro.hpp"
#include "BMP390.hpp"

//----------------------------------------
// Include flash and logging
//----------------------------------------
#include "FlashMemory.hpp"
#include "log.hpp"

//----------------------------------------
// Include Kalman filter and rocket flight logic
//----------------------------------------
#include "stm32pinouts.hpp"
#include "consts.hpp"     // LAUNCH_ACCEL, LAUNCH_VELOCITY, etc.
#include "kalman.hpp"
#include "avghistory.hpp"

// Optional USB serial
#if defined(USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

//----------------------------------------
// Global Sensor Objects
//----------------------------------------
BMI088Accel accel(ACCEL_CS_PIN);
BMI088Gyro  gyro(GYRO_CS_PIN);
BMP390      barometer(BARO_CS_PIN);

//----------------------------------------
// Flash Memory for Logging
//----------------------------------------
FlashMemory flash(FLASH_CS_PIN);

//----------------------------------------
// Shared Sensor Data
//----------------------------------------
float accelData[3];  // X, Y, Z acceleration
float gyroData[3];   // X, Y, Z gyro
float baroData[3];   // altitude, pressure, temperature

// Mutexes to protect sensor data
SemaphoreHandle_t xAccelDataMutex;
SemaphoreHandle_t xGyroDataMutex;
SemaphoreHandle_t xBaroDataMutex;

//----------------------------------------
// Kalman Filter
//----------------------------------------
static KalmanFilter kf(
    KALMAN_PERIOD / 1000.0f,   // time step in seconds
    ALTITUDE_SIGMA,           // altitude sigma
    ACCELERATION_SIGMA,       // accel sigma
    MODEL_SIGMA               // model sigma
);

// Mutex for Kalman
SemaphoreHandle_t xKalmanMutex;

//----------------------------------------
// Rocket Flight Phases
//----------------------------------------
enum class FlightPhase {
    Startup,
    Idle,
    Launched,
    DescendingWithDrogue,
    DescendingWithMain,
    Landed
};

static FlightPhase phase = FlightPhase::Startup;
static bool launched = false;
static uint32_t land_time = 0;
static kfloat_t apogee = 0.0f;

//----------------------------------------
// Gravity & Ground-Level Calibration
//----------------------------------------
static AvgHistory<float, EST_HISTORY_SAMPLES, 3> gravity_est_state;
static AvgHistory<float, EST_HISTORY_SAMPLES, 3> ground_level_est_state;

//----------------------------------------
// Logging Control
//----------------------------------------
volatile bool dropDetected = false;
volatile bool loggingActive = false;
volatile uint32_t loggingStartTime = 0;

//----------------------------------------
// Utility
//----------------------------------------
uint32_t delta(uint32_t start, uint32_t end) {
    if (end >= start) return end - start;
    return (UINT32_MAX - start) + end + 1;
}

//----------------------------------------
// Task Prototypes
//----------------------------------------
void TaskReadAndLogSensors(void* pvParameters);
void TaskLogStep(void* pvParameters);
void TaskConditionMonitor(void* pvParameters);
// void TaskPrintSensors(void* pvParameters);
void TaskDeployment(void* pvParameters);

//----------------------------------------
// Setup
//----------------------------------------
void setup() {
    // Init serial (USB or hardware)
#if defined(USBCON) && defined(USBD_USE_CDC)
    usb_serial.begin();
    while (!usb_serial) { /* wait for USB serial */ }
#else
    Serial.begin(115200);
    while (!Serial) { /* wait for hardware serial */ }
#endif

    // Init SPI & I2C
    SPI.begin();
    Wire.begin();

    // Setup sensors
    accel.setup();
    gyro.setup();
    barometer.setup();

    // Setup flash memory & logging
    flash.setup();
    log_setup();       // Reads flight_num from EEPROM, sets up ring buffers
    log_print_all();   // Prints the previous flight log at startup

    // Create mutexes
    xAccelDataMutex = xSemaphoreCreateMutex();
    xGyroDataMutex  = xSemaphoreCreateMutex();
    xBaroDataMutex  = xSemaphoreCreateMutex();
    xKalmanMutex    = xSemaphoreCreateMutex();

    if (!xAccelDataMutex || !xGyroDataMutex || !xBaroDataMutex || !xKalmanMutex) {
        Serial.println(F("Error creating mutexes."));
        while (1) { /* halt */ }
    }

    // (Optional) Set pins for e-match channels if needed
    // pinMode(RAPTOR_PIN, OUTPUT);
    // digitalWrite(RAPTOR_PIN, LOW);
    // pinMode(PIRANHA_PIN, OUTPUT);
    // digitalWrite(PIRANHA_PIN, LOW);

    // Create FreeRTOS tasks
    xTaskCreate(TaskReadAndLogSensors,  "ReadLogSensors",  2048, NULL, 2, NULL);
    xTaskCreate(TaskLogStep,            "LogStep",         2048, NULL, 2, NULL);    
    xTaskCreate(TaskConditionMonitor,   "ConditionMonitor",2048, NULL, 3, NULL);
    // xTaskCreate(TaskPrintSensors,       "PrintSensors",    2048, NULL, 3, NULL);
    xTaskCreate(TaskDeployment,         "Deployment",      2048, NULL, 1, NULL);

    // Start scheduler
    vTaskStartScheduler();

}

//----------------------------------------
// Loop (unused with FreeRTOS)
//----------------------------------------
void loop() {}

//----------------------------------------
// Task: Read Sensors & Log
//----------------------------------------
void TaskReadAndLogSensors(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        //--- Read Accelerometer ---
        accel.step();
        float localAccel[3];
        accel.get(localAccel);

        // Save to global array safely
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            accelData[0] = localAccel[0];
            accelData[1] = localAccel[1];
            accelData[2] = localAccel[2];
            xSemaphoreGive(xAccelDataMutex);
        }

        //--- Read Gyroscope ---
        gyro.step();
        float localGyro[3];
        gyro.get(localGyro);

        if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
            gyroData[0] = localGyro[0];
            gyroData[1] = localGyro[1];
            gyroData[2] = localGyro[2];
            xSemaphoreGive(xGyroDataMutex);
        }

        //--- Read Barometer ---
        barometer.step();
        float altitude    = barometer.getAltitude();
        float pressure    = barometer.getPressure();
        float temperature = barometer.getTemperature() / 100.0f; // convert to °C

        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            baroData[0] = altitude;
            baroData[1] = pressure;
            baroData[2] = temperature;
            xSemaphoreGive(xBaroDataMutex);
        }

        //--- If logging is active, create LogMessage & push to buffer ---
        if (loggingActive) {
            uint32_t time_ms = millis();

            // Acquire Kalman data safely
            float kPos = 0.0f;
            float kRate = 0.0f;
            float kAccel = 0.0f;
            if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
                kPos   = kf.pos();
                kRate  = kf.rate();
                kAccel = kf.accel();
                xSemaphoreGive(xKalmanMutex);
            }

            // Form the log message
            LogMessage logMsg(
                time_ms,
                // Gyro
                localGyro[0], localGyro[1], localGyro[2],
                // Accel
                localAccel[0], localAccel[1], localAccel[2],
                // Temp & Pressure
                temperature, pressure,
                // Kalman states
                kPos, kRate, kAccel
            );

            // Add to logging ring buffer
            log_add(logMsg);
        }

        // Adjust your sampling rate (50 ms => 20 Hz)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//----------------------------------------
// Task: Write Logs to Flash
//----------------------------------------
void TaskLogStep(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Let log.cpp handle moving from ring buffer to flash
        log_step();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//----------------------------------------
// Task: Detect Free-Fall & Start/Stop Logging
//----------------------------------------
void TaskConditionMonitor(void* pvParameters) {
    (void) pvParameters;

    const float accelerationThreshold = 1.0f;  // near free-fall in g's
    const uint32_t loggingDuration    = 25000; // 25 seconds

    for (;;) {
        // Check Z acceleration
        float zAccel = 0.0f;
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            zAccel = accelData[2];
            xSemaphoreGive(xAccelDataMutex);
        }

        // If not yet detected drop & zAccel near 0 => free-fall
        if (!dropDetected && fabs(zAccel) <= accelerationThreshold && fabs(zAccel) > 0.0f) {
            Serial.println("Detected Z Acceleration: " + String(zAccel));
            dropDetected = true;
            loggingActive = true;
            loggingStartTime = millis();

            log_start();
            Serial.println(F("Drop detected! Logging started."));
        }

        // Stop logging after a fixed duration
        if (loggingActive && (millis() - loggingStartTime >= loggingDuration)) {
            loggingActive = false;
            log_stop();
            Serial.println(F("Logging duration elapsed. Logging stopped."));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//----------------------------------------
// Task: Print Sensors & Kalman for Debug
//----------------------------------------
// void TaskPrintSensors(void* pvParameters) {
//     (void) pvParameters;
//     for (;;) {
//         //--- Print Accelerometer ---
//         if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
//             Serial.print(F("Accel: "));
//             Serial.print(accelData[0]); Serial.print(F(", "));
//             Serial.print(accelData[1]); Serial.print(F(", "));
//             Serial.print(accelData[2]); Serial.print(F(" ("));
//             float mag = sqrtf(accelData[0]*accelData[0] +
//                               accelData[1]*accelData[1] +
//                               accelData[2]*accelData[2]);
//             Serial.print(mag);
//             Serial.println(F(") m/s^2"));
//             xSemaphoreGive(xAccelDataMutex);
//         }

//         //--- Print Gyroscope ---
//         if (xSemaphoreTake(xGyroDataMutex, portMAX_DELAY) == pdTRUE) {
//             Serial.print(F("Gyro: "));
//             Serial.print(gyroData[0]); Serial.print(F(", "));
//             Serial.print(gyroData[1]); Serial.print(F(", "));
//             Serial.print(gyroData[2]); Serial.println(F(" deg/s"));
//             xSemaphoreGive(xGyroDataMutex);
//         }

//         //--- Print Barometer ---
//         if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
//             Serial.print(F("Altitude: "));
//             Serial.print(baroData[0]); Serial.println(F(" m"));

//             Serial.print(F("Pressure: "));
//             Serial.print(baroData[1]); Serial.println(F(" Pa"));

//             Serial.print(F("Temp: "));
//             Serial.print(baroData[2]); Serial.println(F(" °C"));
//             xSemaphoreGive(xBaroDataMutex);
//         }

//         //--- Print Kalman Filter state ---
//         if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
//             Serial.print(F("Kalman [pos, rate, accel]: "));
//             Serial.print(kf.pos());   Serial.print(F(" m, "));
//             Serial.print(kf.rate());  Serial.print(F(" m/s, "));
//             Serial.print(kf.accel()); Serial.println(F(" m/s^2"));
//             xSemaphoreGive(xKalmanMutex);
//         }

//         // Debug printing rate
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

//----------------------------------------
// Task: Rocket Flight Logic (Deployment)
//----------------------------------------
void TaskDeployment(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        float currentAccel[3];
        float currentBaro[3];

        // Acquire sensor data
        if (xSemaphoreTake(xAccelDataMutex, portMAX_DELAY) == pdTRUE) {
            currentAccel[0] = accelData[0];
            currentAccel[1] = accelData[1];
            currentAccel[2] = accelData[2];
            xSemaphoreGive(xAccelDataMutex);
        }

        if (xSemaphoreTake(xBaroDataMutex, portMAX_DELAY) == pdTRUE) {
            currentBaro[0] = baroData[0]; // altitude
            currentBaro[1] = baroData[1]; // pressure
            currentBaro[2] = baroData[2]; // temperature
            xSemaphoreGive(xBaroDataMutex);
        }

        float raw_alt = currentBaro[0];
        float accel_mag = sqrtf(currentAccel[0]*currentAccel[0] +
                                currentAccel[1]*currentAccel[1] +
                                currentAccel[2]*currentAccel[2]);

        // Calibrate gravity and ground altitude in early phase
        if (phase < FlightPhase::Launched) {
            gravity_est_state.add(accel_mag);
            ground_level_est_state.add(raw_alt);
        }

        // Transition from Startup to Idle after enough samples
        if (phase == FlightPhase::Startup) {
            if (!ground_level_est_state.full() || !gravity_est_state.full()) {
                vTaskDelay(pdMS_TO_TICKS(KALMAN_PERIOD));
                continue;
            }
            phase = FlightPhase::Idle;
        }

        // Subtract baseline gravity
        accel_mag -= gravity_est_state.old_avg();
        float alt = raw_alt - ground_level_est_state.old_avg();

        // Update Kalman
        kf.step(accel_mag, raw_alt);

        // Get current Kalman state
        kfloat_t pos = 0.0f, vel = 0.0f, accel_est = 0.0f;
        if (xSemaphoreTake(xKalmanMutex, portMAX_DELAY) == pdTRUE) {
            pos       = kf.pos();
            vel       = kf.rate();
            accel_est = kf.accel();
            xSemaphoreGive(xKalmanMutex);
        }

        // Flight logic
        if (phase == FlightPhase::Idle) {
            // Detect launch
            if (vel > LAUNCH_VELOCITY && accel_est > LAUNCH_ACCEL) {
                phase = FlightPhase::Launched;
                launched = true;
                Serial.println(F("=== Launch detected!"));
            }
        }
        else if (phase == FlightPhase::Launched) {
            // Detect apogee
            if (vel < 0) {
                apogee = pos;
                // fireChannel(RAPTOR_PIN);
                phase = FlightPhase::DescendingWithDrogue;
                Serial.println(F("=== Apogee detected! Drogue deployed."));
            }
        }
        else if (phase == FlightPhase::DescendingWithDrogue) {
            // Deploy main parachute if conditions are met
            if ((pos < MAIN_DEPLOY_ALTITUDE
            #ifdef FAILSAFE_VELOCITY
                || vel < -FAILSAFE_VELOCITY
            #endif
            )) {
                // fireChannel(PIRANHA_PIN);
                phase = FlightPhase::DescendingWithMain;
                Serial.println(F("=== Main parachute deployed."));
            }
        }
        else if (phase == FlightPhase::DescendingWithMain) {
            // Detect landing
            if (pos < LANDED_ALT && fabs(vel) < LANDED_VELOCITY && fabs(accel_est) < LANDED_ACCEL) {
                if (land_time == 0) {
                    land_time = millis();
                    if (land_time == 0) land_time = 1;
                } else if (delta(land_time, millis()) > LANDED_TIME_MS) {
                    phase = FlightPhase::Landed;
                    Serial.println(F("=== Landed safely!"));
                }
            } else {
                land_time = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(KALMAN_PERIOD));
    }
}
