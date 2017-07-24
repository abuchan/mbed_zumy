#ifndef SENSORS_H
#define SENSORS_H

#include "mbed.h"

#include "MPU6050.h"
#include "QEI.h"

#include "protocol.h"

class Sensors {
  
  public:
    
    Sensors(
      Timer *system_timer,
      PinName voltage_pin,
      PinName l_enc_a_pin, PinName l_enc_b_pin,
      PinName r_enc_a_pin, PinName r_enc_b_pin, uint32_t tick_per_rev,
      PinName imu_sda_pin, PinName imu_scl_pin,
      PinName current_pin
    );
    
    float get_voltage(void);
    float get_current(void);
    void get_encoders(int32_t (&encoders)[2]);
    void get_angles(float* angles);
    bool get_imu(sensor_data_t* sensor_data);

    bool fill_sensor_packet(packet_t* packet);
  
  private:
    
    Timer* system_timer_;
    AnalogIn voltage_pin_, current_pin_;
    QEI left_qei_;
    QEI right_qei_;
    MPU6050 mpu6050_;
    bool imu_ready_;
};

#endif
