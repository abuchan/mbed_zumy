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

#define LASER_N_LEVELS 4

class Control {

  public:
  
    Control(
      PinName left_mot_0_pin, PinName left_mot_1_pin, 
      PinName right_mot_0_pin, PinName right_mot_1_pin, 
      Sensors *sensors, uint32_t tick_per_rev,
      float kP, float kI, float kD, float period, float velocity_max,
      float pid_dead_band,
      PinName galvo_sel_pin, PinName galvo_dir_pin, PinName galvo_pulse_pin, PinName galvo_sense_pin,
      PinName laser_pwm_pin
    );

    void set_setpoints(float left, float right);
  
    void fill_pid_packet(packet_t* pkt);
    
    void fill_sensor_packet(packet_t* pkt);
    
    void set_galvo(int steps_0, int steps_1);

    void set_laser(float laser_power);

    typedef enum LaserState{
      LASER_STATE_IDLE, LASER_STATE_BLANK, LASER_STATE_CHANGE
    } LaserState;

    typedef enum GalvoState{
      GALVO_STATE_IDLE, GALVO_STATE_MOVE_0, GALVO_STATE_MOVE_1
    } GalvoState;

  private:
    
    Sensors* sensors_;
    
    RtosTimer control_timer_;

    PID* pids_[2];
  
    void pid_init(int motor, float kP, float kI, float kD, float period, float velocity_max);

    void set_motor_pwm(int motor, float value);
   
    static void control_helper(const void* p);
    
    void control_update(void);
    void control_galvo(void);
    void control_velocity(void);
    void control_laser(void);

    PwmOut* motors_[2][2];

    float tick_to_angular_velocity_;
    float pid_dead_band_;

    float last_positions_[2];
    float velocities_[2];
    float pwms_[2];

    DigitalOut galvo_sel_pin_, galvo_dir_pin_, galvo_pulse_pin_;
    DigitalIn galvo_sense_pin_;
    Control::GalvoState galvo_state_;
    int galvo_delta_, current_sel_;
    int galvo_count_[2];
    int galvo_sense_[2];
   
    const static int laser_periods_[LASER_N_LEVELS], laser_duties_[LASER_N_LEVELS];
    PwmOut laser_pwm_pin_;
    Control::LaserState laser_state_;
    int laser_period_, laser_duty_;
};

#endif
