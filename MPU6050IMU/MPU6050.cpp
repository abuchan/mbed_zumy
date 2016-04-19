#include "MPU6050.h"

MPU6050::MPU6050(
  PinName i2c_sda_pin, PinName i2c_scl_pin, uint32_t i2c_freq,
  AccelScale accel_scale, GyroScale gyro_scale) :
  
  i2c_(i2c_sda_pin, i2c_scl_pin),
  gyro_scale_(gyro_scale), accel_scale_(accel_scale),
  
  gyroMeasError_(PI * (60.0f / 180.0f)),
  beta_(sqrt(3.0f / 4.0f) * gyroMeasError_),
  gyroMeasDrift_(PI * (1.0f / 180.0f)),
  zeta_(sqrt(3.0f / 4.0f) * gyroMeasDrift_) {
  
  // Initialize quaternion scalar element
  q_[0] = 1.0f;

  switch (accel_scale_) {
    case AFS_2G:
      aRes_ = 2.0/32768.0;
      break;
    case AFS_4G:
      aRes_ = 4.0/32768.0;
      break;
    case AFS_8G:
      aRes_ = 8.0/32768.0;
      break;
    case AFS_16G:
      aRes_ = 16.0/32768.0;
      break;
  }
   
  switch (gyro_scale_) {
    case GFS_250DPS:
      gRes_ = 250.0/32768.0;
      break;
    case GFS_500DPS:
      gRes_ = 500.0/32768.0;
      break;
    case GFS_1000DPS:
      gRes_ = 1000.0/32768.0;
      break;
    case GFS_2000DPS:
      gRes_ = 2000.0/32768.0;
      break;
  }
      
  i2c_.frequency(i2c_freq);
  
  // Reset the IMU and wait for boot
  reset();

  uint8_t whoami = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);

  // WHO_AM_I should always be 0x68
  if (whoami == 0x68) {
    
    runSelfTest();

    if( selfTest_[0] < 1.0f && selfTest_[1] < 1.0f &&
        selfTest_[2] < 1.0f && selfTest_[3] < 1.0f &&
        selfTest_[4] < 1.0f && selfTest_[5] < 1.0f) {

      // Reset registers to default in preparation for device calibration
      reset();
      
      // Calibrate gyro and accelerometers, load biases in bias registers  
      calibrate();
      
      // Set to active read mode
      init();
      
      imu_ready_ = true;
    }
  }
}

void MPU6050::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  char data_write[2];
  data_write[0] = subAddress;
  data_write[1] = data;
  i2c_.write(address, data_write, 2, 0);
}

char MPU6050::readByte(uint8_t address, uint8_t subAddress) {
  char data[1]; // `data` will store the register data     
  char data_write[1];
  data_write[0] = subAddress;
  i2c_.write(address, data_write, 1, 1); // no stop
  i2c_.read(address, data, 1, 0); 
  return data[0]; 
}

void MPU6050::readBytes(
  uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {     

  char data[14];
  char data_write[1];
  data_write[0] = subAddress;
  i2c_.write(address, data_write, 1, 1); // no stop
  i2c_.read(address, data, count, 0); 
  for(int ii = 0; ii < count; ii++) {
   dest[ii] = data[ii];
  }
} 

void MPU6050::readAccelData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  
  // Read the six raw data registers into data array
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU6050::readGyroData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  
  // Read the six raw data registers sequentially into data array
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

// Gyro biases are written to IMU, so don't need to subtract them
bool MPU6050::readCalibAccelGyroData(float* accel_out, float* gyro_out) {
  int16_t accel[3];
  int16_t gyro[3];
  if (imu_ready_ && (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)) {
    readAccelData(accel);
    readGyroData(gyro);
    accel_out[0] = ((float)accel[0]) * aRes_ - accelBias_[0];
    accel_out[1] = ((float)accel[1]) * aRes_ - accelBias_[1];
    accel_out[2] = ((float)accel[2]) * aRes_ - accelBias_[2];
    gyro_out[0] = ((float)gyro[0]) * gRes_; // - gyroBias_[0];
    gyro_out[1] = ((float)gyro[1]) * gRes_; // - gyroBias_[1];
    gyro_out[2] = ((float)gyro[2]) * gRes_; // - gyroBias_[2];
    return true;
  } else {
    return false;
  }
}

int16_t MPU6050::readTempData() {
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  
  // Read the two raw data registers sequentially into data array 
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  
  // Turn the MSB and LSB into a 16-bit value
  return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;
}

/**
 * The sensor has a high-pass filter necessary to invoke to allow the sensor
 * motion detection algorithms work properly. Motion detection occurs on 
 * free-fall (acceleration below a threshold for some time for all axes), motion
 * (acceleration above a threshold for some time on at least one axis), and
 * zero-motion toggle (acceleration on each axis less than a threshold for some 
 * time sets this flag, motion above the threshold turns it off). The high-pass 
 * filter takes gravity out consideration for these threshold evaluations;
 * otherwise, the flags would be set all the time!
 */
void MPU6050::lowPowerAccelOnly() {
  uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  
  // Clear sleep and cycle bits [5:6]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); 
  // Set sleep and cycle bits [5:6] to zero to make sure accel is running
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); 

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  // Clear standby XA, YA, and ZA bits [3:5]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38);
  // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); 
    
  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  // Clear high-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07);
  // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz,
  // ) 0.63 Hz, or 7) Hold. Set ACCEL_HPF to 0; reset mode disbaling high-pass 
  // filter
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);

  c = readByte(MPU6050_ADDRESS, CONFIG);
  // Clear low-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07);
  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate
  writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);
    
  c = readByte(MPU6050_ADDRESS, INT_ENABLE);
  // Clear all interrupts
  writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);
  // Enable motion threshold (bits 5) interrupt only
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);
  
  // Motion detection interrupt requires the absolute value of any axis to lie
  // above the detection threshold
  // for at least the counter duration
  // Set motion detection to 0.256 g; LSB = 2 mg
  writeByte(MPU6050_ADDRESS, MOT_THR, 0x80);
  // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01);
  
  wait(0.1);  // Add delay for accumulation of samples
  
  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  // Clear high-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07);
  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);
   
  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7);
  // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])  
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47);

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  // Clear sleep and cycle bit 5
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20);
  // Set cycle bit 5 to begin low power accelerometer motion interrupts
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20);
}

void MPU6050::reset(void) {
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); 
  wait(0.1);
}

void MPU6050::init(void) {  
  // Initialize MPU6050 device
  // wake up device
  // Clear sleep mode bit (6), enable all sensors 
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);

  // Delay 100 ms for PLL to get established on x-axis gyro; should check for
  // PLL ready interrupt  
  wait(0.1);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz,
  // respectively; 
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);  
 
  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; the same rate set in CONFIG above
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);
 
  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are 
  // left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0);
  // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);
  // Set full scale range for the gyro
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | gyro_scale_ << 3);
   
  // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0);
  // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);
  // Set full scale range for the accelerometer 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | accel_scale_ << 3);

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS,
  // enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
  // Enable data ready (bit 0) interrupt    
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);
}

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then 
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050::calibrate(void) {
  // data array to hold accelerometer and gyro x, y, z, data
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  // reset device, reset all registers, clear gyro and accelerometer bias 
  // registers.
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
  wait(0.1);  
   
  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00); 
  wait(0.2);
  
  // Configure device for bias calculation
  // Disable all interrupts
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);
  // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
  // Disable I2C master
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00);
  // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);
  wait(0.015);
  
  // Configure MPU6050 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);
  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  // Enable FIFO
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);
  // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in 
  // MPU-6050) 
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);
  wait(0.08); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);
  // read FIFO sample count
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    
    // read data for averaging
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]);

    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];          
  }
  
  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count; 
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
 
  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias format
  // Biases are additive, so change sign on calculated average gyro biases
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; 
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]); 
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);
  
  // construct gyro bias in deg/s for later manual subtraction
  gyroBias_[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias_[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias_[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
      
  // Remove gravity from the z-axis accelerometer bias calculation
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  /**
   * Construct the accelerometer biases for push to the hardware accelerometer 
   * bias registers. These registers contain factory trim values which must be 
   * added to the calculated accelerometer biases; on boot up these registers 
   * will hold non-zero values. In addition, bit 0 of the lower byte must be 
   * preserved since it is used for temperature compensation calculations. 
   * Accelerometer bias registers expect bias input as 2048 LSB per g, so that
   * the accelerometer biases calculated above must be divided by 8.
   */
  
  /*
  // Read factory accelerometer trim values.
  int32_t accel_bias_reg[3] = {0, 0, 0};
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  // Define mask for temperature compensation bit 0 of lower byte of 
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};
  
  // If temperature compensation bit is set, record that fact in mask_bit
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;
  }

  // Construct total accelerometer bias, including calculated average 
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g 
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  // preserve temperature compensation bit when writing back to accelerometer 
  // bias registers
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; 
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2];

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);  
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);  
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);
  */

  // Output scaled accelerometer biases for manual subtraction in the main 
  // program
  accelBias_[0] = (float)accel_bias[0]/(float)accelsensitivity; 
  accelBias_[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias_[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values,
// +/- 14 or less deviation is a pass
void MPU6050::runSelfTest(void) {
  uint8_t rawData[4] = {0, 0, 0, 0};
  uint8_t selfTest[6];
  float factoryTrim[6];

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0);
  wait(0.25);  // Delay a while to let the device execute the self-test
  
  // X-axis, Y-axis, Z-axis, Mixed-axis self-test results
  rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X);
  rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y);
  rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z);
  rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A);
  
  // Extract the acceleration test results first (five-bit unsigned integers)
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ;
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ;
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ;
  
  // Extract the gyration test results first (five-bit unsigned integers)
  selfTest[3] = rawData[0]  & 0x1F ;
  selfTest[4] = rawData[1]  & 0x1F ;
  selfTest[5] = rawData[2]  & 0x1F ;
  
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0f*0.34f) * (
    pow((0.92f/0.34f), ((selfTest[0] - 1.0f)/30.0f)));
  factoryTrim[1] = (4096.0f*0.34f) * (
    pow((0.92f/0.34f), ((selfTest[1] - 1.0f)/30.0f)));
  factoryTrim[2] = (4096.0f*0.34f) * (
    pow((0.92f/0.34f), ((selfTest[2] - 1.0f)/30.0f)));
  factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));
  factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));
  factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));
   
  //  Output self-test results and factory trim calculation if desired
  /*
  Serial.println(selfTest[0]); Serial.println(selfTest[1]); 
  Serial.println(selfTest[2]); Serial.println(selfTest[3]); 
  Serial.println(selfTest[4]); Serial.println(selfTest[5]);
  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); 
  Serial.println(factoryTrim[2]); Serial.println(factoryTrim[3]); 
  Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);
  */
  
  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of
  // the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    // Report percent differences
    selfTest_[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i];
  } 
}

/**
 * Implementation of Sebastian Madgwick's "...efficient orientation filter 
 * for... inertial/magnetic sensor arrays" (see 
 * http://www.x-io.co.uk/category/open-source/ for examples and more details)
 * which fuses acceleration and rotation rate to produce a quaternion-based
 * estimate of relative device orientation -- which can be converted to yaw, 
 * pitch, and roll. Useful for stabilizing quadcopters, etc. The performance of 
 * the orientation filter is at least as good as conventional Kalman-based 
 * filtering algorithms but is much less computationally intensive---it can be 
 * performed on a 3.3 V Pro Mini operating at 8 MHz!
 */
void MPU6050::MadgwickQuaternionUpdate(
  float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  
  // short name local variable for readability
  float q1 = q_[0], q2 = q_[1], q3 = q_[2], q4 = q_[3];
  // vector norm
  float norm;
  // objective function elements
  float f1, f2, f3;                                        
  // objective function Jacobian elements 
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  // gyro bias error
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  //            float _2q1q3 = 2.0f * q1 * q3;
  //            float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(
    hatDot1 * hatDot1 + hatDot2 * hatDot2 + 
    hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta_;
  gbiasy += gerry * deltat * zeta_;
  gbiasz += gerrz * deltat * zeta_;
  //           gx -= gbiasx;
  //           gy -= gbiasy;
  //           gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta_ * hatDot1)) * deltat;
  q2 += (qDot2 -(beta_ * hatDot2)) * deltat;
  q3 += (qDot3 -(beta_ * hatDot3)) * deltat;
  q4 += (qDot4 -(beta_ * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q_[0] = q1 * norm;
  q_[1] = q2 * norm;
  q_[2] = q3 * norm;
  q_[3] = q4 * norm;
}
