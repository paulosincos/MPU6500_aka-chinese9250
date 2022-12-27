#pragma once
#ifndef MPU6500_H
#define MPU6500_H

#include <Wire.h>

#include "MPU6500_Raw/MPU6500RegisterMap_Raw.h"

enum class ACCEL_FS_SEL {
    A2G,
    A4G,
    A8G,
    A16G
};
enum class GYRO_FS_SEL {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
};

enum class FIFO_SAMPLE_RATE : uint8_t {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t {
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t {
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71};
static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE {0x73};
static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE {0x70};

struct MPU6500Setting {
    ACCEL_FS_SEL accel_fs_sel {ACCEL_FS_SEL::A16G};
    GYRO_FS_SEL gyro_fs_sel {GYRO_FS_SEL::G2000DPS};
    FIFO_SAMPLE_RATE fifo_sample_rate {FIFO_SAMPLE_RATE::SMPL_200HZ};
    uint8_t gyro_fchoice {0x03};
    GYRO_DLPF_CFG gyro_dlpf_cfg {GYRO_DLPF_CFG::DLPF_41HZ};
    uint8_t accel_fchoice {0x01};
    ACCEL_DLPF_CFG accel_dlpf_cfg {ACCEL_DLPF_CFG::DLPF_45HZ};
};

template <typename WireType>
class MPU6500_ {
    static constexpr uint8_t MPU6500_DEFAULT_ADDRESS {0x68};  // Device address when ADO = 0
    uint8_t mpu_i2c_addr {MPU6500_DEFAULT_ADDRESS};

    // settings
    MPU6500Setting setting;
    // TODO: this should be configured!!
    float acc_resolution {0.f};                // scale resolutions per LSB for the sensors
    float gyro_resolution {0.f};               // scale resolutions per LSB for the sensors

    // Calibration Parameters
    float acc_bias[3] {0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
    float gyro_bias[3] {0., 0., 0.};  // gyro calibration value in GYRO_FS_SEL: 250dps

    // Temperature
    int16_t temperature_count {0};  // temperature raw count output
    float temperature {0.f};        // Stores the real internal chip temperature in degrees Celsius

    // Self Test
    float self_test_result[6] {0.f};  // holds results of gyro and accelerometer self test

    // IMU Data
    float a[3] {0.f, 0.f, 0.f};
    float g[3] {0.f, 0.f, 0.f};

    // Other settings
    bool has_connected {false};
    bool b_ahrs {true};
    bool b_verbose {false};
    bool silent {true};

    // I2C
    WireType* wire;
    uint8_t i2c_err_;

public:
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     // LSB/degrees/sec
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

    bool setup(const uint8_t addr, const MPU6500Setting& mpu_setting = MPU6500Setting(), WireType& w = Wire) {
        // addr should be valid for MPU
        if ((addr < MPU6500_DEFAULT_ADDRESS) || (addr > MPU6500_DEFAULT_ADDRESS + 7)) {
            if (!silent) {
                Serial.print(F("I2C address 0x"));
                Serial.print(addr, HEX);
                Serial.println(F(" is not valid for MPU. Please check your I2C address."));
            }
            return false;
        }
        mpu_i2c_addr = addr;
        setting = mpu_setting;
        wire = &w;

        if (isConnectedMPU6500()) {
            initMPU6500();
        } else {
            if (b_verbose)
                Serial.println(F("Could not connect to MPU6500"));
            has_connected = false;
            return false;
        }
        has_connected = true;
        return true;
    }

    void verbose(const bool b) {
        b_verbose = b;
    }

    void ahrs(const bool b) {
        b_ahrs = b;
    }

    void calibrateAccelGyro() {
        calibrate_acc_gyro_impl();
    }

    bool isConnected() {
        has_connected = isConnectedMPU6500();
        return has_connected;
    }

    bool isConnectedMPU6500() {
        byte c = read_byte(mpu_i2c_addr, WHO_AM_I);
        if (b_verbose) {
            Serial.print(F("WHO AM I = "));
            Serial.println(c, HEX);
        }
        bool b = (c == MPU6500_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU9250_WHOAMI_DEFAULT_VALUE);
        return b;
    }

    bool available() {
        return has_connected && (read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
    }

    bool update() {
        if (!available()) return false;
        update_accel_gyro();

        float an = -a[0];
        float ae = +a[1];
        float ad = +a[2];
        float gn = +g[0] * DEG_TO_RAD;
        float ge = -g[1] * DEG_TO_RAD;
        float gd = -g[2] * DEG_TO_RAD;

        return true;
    }

    float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }

    float getAccX() const { return a[0]; }
    float getAccY() const { return a[1]; }
    float getAccZ() const { return a[2]; }
    float getGyroX() const { return g[0]; }
    float getGyroY() const { return g[1]; }
    float getGyroZ() const { return g[2]; } 

    float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
    float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }

    float getAccBiasX() const { return acc_bias[0]; }
    float getAccBiasY() const { return acc_bias[1]; }
    float getAccBiasZ() const { return acc_bias[2]; }
    float getGyroBiasX() const { return gyro_bias[0]; }
    float getGyroBiasY() const { return gyro_bias[1]; }
    float getGyroBiasZ() const { return gyro_bias[2]; }

    float getTemperature() const { return temperature; }

    void setAccBias(const float x, const float y, const float z) {
        acc_bias[0] = x;
        acc_bias[1] = y;
        acc_bias[2] = z;
        write_accel_offset();
    }
    void setGyroBias(const float x, const float y, const float z) {
        gyro_bias[0] = x;
        gyro_bias[1] = y;
        gyro_bias[2] = z;
        write_gyro_offset();
    }

    bool selftest() {
        return self_test_impl();
    }

private:
    void initMPU6500() {
        acc_resolution = get_acc_resolution(setting.accel_fs_sel);
        gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);

        // reset device
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // wake up device
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
        delay(100);                                  // Wait for all registers to reset

        // get stable time source
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
        delay(200);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;
        write_byte(mpu_i2c_addr, MPU_CONFIG, mpu_config);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
        write_byte(mpu_i2c_addr, SMPLRT_DIV, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                            // determined inset in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = read_byte(mpu_i2c_addr, GYRO_CONFIG);  // get current GYRO_CONFIG register value
        c = c & ~0xE0;                                     // Clear self-test bits [7:5]
        c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
        c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.gyro_fs_sel) << 3);       // Set full scale range for the gyro
        c = c | (uint8_t(~setting.gyro_fchoice) & 0x03);   // Set Fchoice for the gyro
        write_byte(mpu_i2c_addr, GYRO_CONFIG, c);          // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = read_byte(mpu_i2c_addr, ACCEL_CONFIG);     // get current ACCEL_CONFIG register value
        c = c & ~0xE0;                                 // Clear self-test bits [7:5]
        c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.accel_fs_sel) << 3);  // Set full scale range for the accelerometer
        write_byte(mpu_i2c_addr, ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = read_byte(mpu_i2c_addr, ACCEL_CONFIG2);        // get current ACCEL_CONFIG2 register value
        c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | (~(setting.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
        c = c | (uint8_t(setting.accel_dlpf_cfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        write_byte(mpu_i2c_addr, ACCEL_CONFIG2, c);        // Write new ACCEL_CONFIG2 register value

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        // write_byte(mpu_i2c_addr, INT_PIN_CFG, 0x22);
        // write_byte(mpu_i2c_addr, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
        delay(100);
    }

    void update_accel_gyro() {
        int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
        read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        a[0] = (float)raw_acc_gyro_data[0] * acc_resolution;  // get actual g value, this depends on scale being set
        a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
        a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

        temperature_count = raw_acc_gyro_data[3];                  // Read the adc values
        temperature = ((float)temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

        // Calculate the gyro value into actual degrees per second
        g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution;  // get actual gyro value, this depends on scale being set
        g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
        g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;
    }

    void read_accel_gyro(int16_t* destination) {
        uint8_t raw_data[14];                                                 // x/y/z accel register data stored here
        read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
        destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
        destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
        destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
        destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
        destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
    }

    int16_t read_temperature_data() {
        uint8_t raw_data[2];                                    // x/y/z gyro register data stored here
        read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, &raw_data[0]);  // Read the two raw data registers sequentially into data array
        return ((int16_t)raw_data[0] << 8) | raw_data[1];       // Turn the MSB and LSB into a 16-bit value
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    // ACCEL_FS_SEL: 2g (maximum sensitivity)
    // GYRO_FS_SEL: 250dps (maximum sensitivity)
    void calibrate_acc_gyro_impl() {
        set_acc_gyro_to_calibration();
        collect_acc_gyro_data_to(acc_bias, gyro_bias);
        write_accel_offset();
        write_gyro_offset();
        delay(100);
        initMPU6500();
        delay(1000);
    }

    void set_acc_gyro_to_calibration() {
        // reset device
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);
        write_byte(mpu_i2c_addr, PWR_MGMT_2, 0x00);
        delay(200);

        // Configure device for bias calculation
        write_byte(mpu_i2c_addr, INT_ENABLE, 0x00);    // Disable all interrupts
        write_byte(mpu_i2c_addr, FIFO_EN, 0x00);       // Disable FIFO
        write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00);    // Turn on internal clock source
        write_byte(mpu_i2c_addr, I2C_MST_CTRL, 0x00);  // Disable I2C master
        write_byte(mpu_i2c_addr, USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
        write_byte(mpu_i2c_addr, USER_CTRL, 0x0C);     // Reset FIFO and DMP
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        write_byte(mpu_i2c_addr, MPU_CONFIG, 0x01);    // Set low-pass filter to 188 Hz
        write_byte(mpu_i2c_addr, SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
        write_byte(mpu_i2c_addr, GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        write_byte(mpu_i2c_addr, USER_CTRL, 0x40);  // Enable FIFO
        write_byte(mpu_i2c_addr, FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        delay(40);                                  // accumulate 40 samples in 40 milliseconds = 480 bytes
    }

    void collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
        // At end of sample accumulation, turn off FIFO sensor read
        uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
        write_byte(mpu_i2c_addr, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
        read_bytes(mpu_i2c_addr, FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
        uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
        uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

        for (uint16_t ii = 0; ii < packet_count; ii++) {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            read_bytes(mpu_i2c_addr, FIFO_R_W, 12, &data[0]);              // read data for averaging
            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

            a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            a_bias[1] += (float)accel_temp[1];
            a_bias[2] += (float)accel_temp[2];
            g_bias[0] += (float)gyro_temp[0];
            g_bias[1] += (float)gyro_temp[1];
            g_bias[2] += (float)gyro_temp[2];
        }
        a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
        a_bias[1] /= (float)packet_count;
        a_bias[2] /= (float)packet_count;
        g_bias[0] /= (float)packet_count;
        g_bias[1] /= (float)packet_count;
        g_bias[2] /= (float)packet_count;

        if (a_bias[2] > 0L) {
            a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
        }  // Remove gravity from the z-axis accelerometer bias calculation
        else {
            a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
        }
    }

    void write_accel_offset() {
        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        uint8_t read_data[2] = {0};
        int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
        read_bytes(mpu_i2c_addr, XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
        acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
        read_bytes(mpu_i2c_addr, YA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
        read_bytes(mpu_i2c_addr, ZA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

        int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
        for (int i = 0; i < 3; i++) {
            if (acc_bias_reg[i] % 2) {
                mask_bit[i] = 0;
            }
            acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
            if (mask_bit[i]) {
                acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
            } else {
                acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
            }
        }

        uint8_t write_data[6] = {0};
        write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
        write_data[1] = (acc_bias_reg[0]) & 0xFF;
        write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
        write_data[3] = (acc_bias_reg[1]) & 0xFF;
        write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
        write_data[5] = (acc_bias_reg[2]) & 0xFF;

        // Push accelerometer biases to hardware registers
        write_byte(mpu_i2c_addr, XA_OFFSET_H, write_data[0]);
        write_byte(mpu_i2c_addr, XA_OFFSET_L, write_data[1]);
        write_byte(mpu_i2c_addr, YA_OFFSET_H, write_data[2]);
        write_byte(mpu_i2c_addr, YA_OFFSET_L, write_data[3]);
        write_byte(mpu_i2c_addr, ZA_OFFSET_H, write_data[4]);
        write_byte(mpu_i2c_addr, ZA_OFFSET_L, write_data[5]);
    }

    void write_gyro_offset() {
        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        uint8_t gyro_offset_data[6] {0};
        gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
        gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
        gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
        gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
        gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

        // Push gyro biases to hardware registers
        write_byte(mpu_i2c_addr, XG_OFFSET_H, gyro_offset_data[0]);
        write_byte(mpu_i2c_addr, XG_OFFSET_L, gyro_offset_data[1]);
        write_byte(mpu_i2c_addr, YG_OFFSET_H, gyro_offset_data[2]);
        write_byte(mpu_i2c_addr, YG_OFFSET_L, gyro_offset_data[3]);
        write_byte(mpu_i2c_addr, ZG_OFFSET_H, gyro_offset_data[4]);
        write_byte(mpu_i2c_addr, ZG_OFFSET_L, gyro_offset_data[5]);
    }

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    bool self_test_impl()  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    {
        uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
        int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
        float factoryTrim[6];
        uint8_t FS = 0;

        write_byte(mpu_i2c_addr, SMPLRT_DIV, 0x00);       // Set gyro sample rate to 1 kHz
        write_byte(mpu_i2c_addr, MPU_CONFIG, 0x02);       // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        write_byte(mpu_i2c_addr, GYRO_CONFIG, FS << 3);   // Set full scale range for the gyro to 250 dps
        write_byte(mpu_i2c_addr, ACCEL_CONFIG2, 0x02);    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        write_byte(mpu_i2c_addr, ACCEL_CONFIG, FS << 3);  // Set full scale range for the accelerometer to 2 g

        for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

            read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 6, &raw_data[0]);          // Read the six raw data registers into data array
            aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            read_bytes(mpu_i2c_addr, GYRO_XOUT_H, 6, &raw_data[0]);           // Read the six raw data registers sequentially into data array
            gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0xE0);  // Enable self test on all three axes and set accelerometer range to +/- 2 g
        write_byte(mpu_i2c_addr, GYRO_CONFIG, 0xE0);   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25);                                     // Delay a while to let the device stabilize

        for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

            read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 6, &raw_data[0]);            // Read the six raw data registers into data array
            aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            read_bytes(mpu_i2c_addr, GYRO_XOUT_H, 6, &raw_data[0]);             // Read the six raw data registers sequentially into data array
            gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0x00);
        write_byte(mpu_i2c_addr, GYRO_CONFIG, 0x00);
        delay(25);  // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        uint8_t self_test_data[6];
        self_test_data[0] = read_byte(mpu_i2c_addr, SELF_TEST_X_ACCEL);  // X-axis accel self-test results
        self_test_data[1] = read_byte(mpu_i2c_addr, SELF_TEST_Y_ACCEL);  // Y-axis accel self-test results
        self_test_data[2] = read_byte(mpu_i2c_addr, SELF_TEST_Z_ACCEL);  // Z-axis accel self-test results
        self_test_data[3] = read_byte(mpu_i2c_addr, SELF_TEST_X_GYRO);   // X-axis gyro self-test results
        self_test_data[4] = read_byte(mpu_i2c_addr, SELF_TEST_Y_GYRO);   // Y-axis gyro self-test results
        self_test_data[5] = read_byte(mpu_i2c_addr, SELF_TEST_Z_GYRO);   // Z-axis gyro self-test results

        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[0] - 1.0)));  // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[1] - 1.0)));  // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[2] - 1.0)));  // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[3] - 1.0)));  // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[4] - 1.0)));  // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[5] - 1.0)));  // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++) {
            self_test_result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;          // Report percent differences
            self_test_result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.;  // Report percent differences
        }

        if (b_verbose) {
            Serial.print(F("x-axis self test: acceleration trim within : "));
            Serial.print(self_test_result[0], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("y-axis self test: acceleration trim within : "));
            Serial.print(self_test_result[1], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("z-axis self test: acceleration trim within : "));
            Serial.print(self_test_result[2], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("x-axis self test: gyration trim within : "));
            Serial.print(self_test_result[3], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("y-axis self test: gyration trim within : "));
            Serial.print(self_test_result[4], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("z-axis self test: gyration trim within : "));
            Serial.print(self_test_result[5], 1);
            Serial.println(F("% of factory value"));
        }

        bool b = true;
        for (uint8_t i = 0; i < 6; ++i) {
            b &= fabs(self_test_result[i]) <= 14.f;
        }
        return b;
    }

    float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const {
        switch (accel_af_sel) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case ACCEL_FS_SEL::A2G:
                return 2.0 / 32768.0;
            case ACCEL_FS_SEL::A4G:
                return 4.0 / 32768.0;
            case ACCEL_FS_SEL::A8G:
                return 8.0 / 32768.0;
            case ACCEL_FS_SEL::A16G:
                return 16.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const {
        switch (gyro_fs_sel) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GYRO_FS_SEL::G250DPS:
                return 250.0 / 32768.0;
            case GYRO_FS_SEL::G500DPS:
                return 500.0 / 32768.0;
            case GYRO_FS_SEL::G1000DPS:
                return 1000.0 / 32768.0;
            case GYRO_FS_SEL::G2000DPS:
                return 2000.0 / 32768.0;
            default:
                return 0.;
        }
    }

    void write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
        wire->beginTransmission(address);    // Initialize the Tx buffer
        wire->write(subAddress);             // Put slave register address in Tx buffer
        wire->write(data);                   // Put data in Tx buffer
        i2c_err_ = wire->endTransmission();  // Send the Tx buffer
        if (i2c_err_) print_i2c_error();
    }

    uint8_t read_byte(uint8_t address, uint8_t subAddress) {
        uint8_t data = 0;                         // `data` will store the register data
        wire->beginTransmission(address);         // Initialize the Tx buffer
        wire->write(subAddress);                  // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) print_i2c_error();
        wire->requestFrom(address, (size_t)1);       // Read one byte from slave register address
        if (wire->available()) data = wire->read();  // Fill Rx buffer with result
        return data;                                 // Return data read from slave register
    }

    void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
        wire->beginTransmission(address);         // Initialize the Tx buffer
        wire->write(subAddress);                  // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) print_i2c_error();
        uint8_t i = 0;
        wire->requestFrom(address, count);  // Read bytes from slave register address
        while (wire->available()) {
            dest[i++] = wire->read();
        }  // Put read results in the Rx buffer
    }

    void print_i2c_error() {
        if (i2c_err_ == 7 || silent) return;  // to avoid stickbreaker-i2c branch's error code
        Serial.print(F("I2C ERROR CODE : "));
        Serial.println(i2c_err_);
    }
};

using MPU6500 = MPU6500_<TwoWire>;

#endif  // MPU6500_H
