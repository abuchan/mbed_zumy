#ifndef MPU6050_H
#define MPU6050_H
 
#include "mbed.h"
#include "math.h"
 
/**
 * Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 
 * 08/19/2013 6 DOF Motion sensor fusion device. Invensense Inc.,
 * www.invensense.com.  See also MPU-6050 Register Map and Descriptions, 
 * Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in 
 * above document; the MPU6050 and MPU 9150 are virtually identical but the 
 * latter has an on-board magnetic sensor
 */

// Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define XGOFFS_TC        0x00 
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
// [7:0] fine gain
#define X_FINE_GAIN      0x03
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
// User-defined trim values for accelerometer
#define XA_OFFSET_H      0x06
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
// User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRH     0x13 
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
// Free-fall
#define FF_THR           0x1D
#define FF_DUR           0x1E
// Motion detection threshold bits [7:0]
#define MOT_THR          0x1F
// Duration counter thres. for motion interrupt generation, 1 kHz, LSB = 1 ms
#define MOT_DUR          0x20
// Zero-motion detection threshold bits [7:0]
#define ZMOT_THR         0x21
// Duration counter thres. for 0 motion interrupt generation, 16 Hz, LSB = 64 ms
#define ZRMOT_DUR        0x22
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
// Set read/write pointer to a specific start address in specified DMP bank
#define DMP_RW_PNT       0x6E
// Register in DMP from which to read or to which to write
#define DMP_REG          0x6F
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7
// resistor. Seven-bit device address is 110100 for ADO = 0 and 110101 
// for ADO = 1
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif

#define PI 3.14159265358979323846f

class MPU6050 {
 
  public:
  
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    enum AccelScale {AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G};

    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    enum GyroScale {GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS};

    MPU6050(PinName i2c_sda_pin, PinName i2c_scl_pin,
      uint32_t i2c_freq = 400000,
      AccelScale accel_scale = AFS_2G, GyroScale gyro_scale = GFS_250DPS);

    void readAccelData(int16_t * destination);
    
    void readGyroData(int16_t * destination);
    
    bool readCalibAccelGyroData(float* accel_out, float* gyro_out);
    
    int16_t readTempData(void);
    
    // Configure the motion detection control for low power accelerometer mode
    void lowPowerAccelOnly(void);
    
    // reset device
    void reset(void);
    
    // Initialize MPU6050 device
    void init(void);
    
    // Function which accumulates gyro and accelerometer data after device 
    // initialization. It calculates the average of the at-rest readings.
    // then loads the resulting offsets into accelerometer and gyro bias 
    // registers.
    void calibrate(void);

  private:
    // Indicates IMU has completed startup and calibration
    bool imu_ready_;

    // I2C interface
    I2C i2c_;
    
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    char readByte(uint8_t address, uint8_t subAddress);
    void readBytes(
      uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
    
    int gyro_scale_;
    int accel_scale_;

    // scale resolutions per LSB for the sensors
    float aRes_, gRes_;

    // Bias corrections for gyro and accelerometer
    float gyroBias_[3], accelBias_[3];

    float selfTest_[6];
    
    // Accelerometer and gyroscope self test, check calibration wrt 
    // factory settings
    void runSelfTest(void);

    // parameters for 6 DoF sensor fusion calculations
    // gyroscope measurement error in rads/s (start at 60 deg/s), 
    // then reduce after ~10 s to 3
    float gyroMeasError_, beta_;
    
    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float gyroMeasDrift_, zeta_;
    
    // vector to hold quaternion
    float q_[4];
    
    // Implementation of Sebastian Madgwick's "...efficient orientation filter 
    // for... inertial/magnetic sensor arrays"
    void MadgwickQuaternionUpdate(
      float ax, float ay, float az, float gx, float gy, float gz, float deltat);

};

#endif
