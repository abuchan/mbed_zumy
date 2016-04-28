#ifndef CONTROL_H
#define CONTROL_H

#include "mbed.h"

#include "rtos.h"
#include "PID.h"

#include "protocol.h"
#include "packet_parser.h"
#include "sensors.h"

#define MOTOR_RIGHT 1
#define MOTOR_LEFT 0

class Control {

  public:
  
    Control(
      PinName left_mot_0_pin, PinName left_mot_1_pin, 
      PinName right_mot_0_pin, PinName right_mot_1_pin, 
      Sensors *sensors, uint32_t tick_per_rev,
      float kP, float kI, float kD, float period, float velocity_max, float pid_dead_band
    );

    void set_setpoints(float left, float right);
  
    void fill_pid_packet(packet_t* pkt);
    
    void fill_sensor_packet(packet_t* pkt);

  private:
    
    Sensors* sensors_;
    
    RtosTimer control_timer_;

    PID* pids_[2];

    void pid_init(int motor, float kP, float kI, float kD, float period, float velocity_max);

    void set_motor_pwm(int motor, float value);
   
    static void control_helper(const void* p);
    void control_update(void);
    
    PwmOut* motors_[2][2];

    float tick_to_angular_velocity_;
    float pid_dead_band_;

    float last_positions_[2];
    float velocities_[2];
    float pwms_[2];
};

#endif
