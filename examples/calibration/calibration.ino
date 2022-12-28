#include "MPU6500.h"

MPU6500 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    print_calibration();
    mpu.verbose(false);
}

void loop() {
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias: ");
    Serial.print(mpu.getAccBiasX());
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY());
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ());
    Serial.println();
    Serial.println("gyro bias: ");
    Serial.print(mpu.getGyroBiasX());
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY());
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ());
    Serial.println();
}