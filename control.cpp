#include "control.h"

Control::Control(
  PinName left_mot_0_pin, PinName left_mot_1_pin, 
  PinName right_mot_0_pin, PinName right_mot_1_pin, 
  Sensors *sensors, uint32_t tick_per_rev,
  float kP, float kI, float kD, float period, float velocity_max, float pid_dead_band) :
  sensors_(sensors),
  control_timer_(&Control::control_helper,osTimerPeriodic,this), pid_dead_band_(pid_dead_band) {

  motors_[MOTOR_LEFT][0] = new PwmOut(left_mot_0_pin);
  motors_[MOTOR_LEFT][1] = new PwmOut(left_mot_1_pin);
  motors_[MOTOR_RIGHT][0] = new PwmOut(right_mot_0_pin);
  motors_[MOTOR_RIGHT][1] = new PwmOut(right_mot_1_pin);
  
  // For the LPC1768, all PWM channels are on the same timer, so setting one
  // period sets them all
  motors_[MOTOR_LEFT][0]->period_us(50);

  tick_to_angular_velocity_ = 2.0 * 3.14159265358979323846 / (float)(tick_per_rev * period);
  
  pid_init(MOTOR_LEFT,kP,kI,kD,period,velocity_max);
  pid_init(MOTOR_RIGHT,kP,kI,kD,period,velocity_max);

  control_timer_.start(period*1000);

  last_positions_[MOTOR_LEFT] = 0;
  last_positions_[MOTOR_RIGHT] = 0;
}

void Control::set_setpoints(float left, float right) {
  pids_[MOTOR_LEFT]->setSetPoint(left);
  pids_[MOTOR_RIGHT]->setSetPoint(right);
}

void Control::fill_pid_packet(packet_t* pkt) {
  pkt->header.type = PKT_TYPE_PID;
  pkt->header.length = sizeof(header_t) + sizeof(pid_data_t) + 1;
  pid_data_t* pid_data = (pid_data_t*)pkt->data_crc;
  pid_data->vel[MOTOR_LEFT] = velocities_[MOTOR_LEFT];
  pid_data->vel[MOTOR_RIGHT] = velocities_[MOTOR_RIGHT];
  pid_data->pwm[MOTOR_LEFT] = pwms_[MOTOR_LEFT];
  pid_data->pwm[MOTOR_RIGHT] = pwms_[MOTOR_RIGHT];
}

void Control::fill_sensor_packet(packet_t* pkt) {
  pkt->header.type = PKT_TYPE_SENSOR;
  pkt->header.length = sizeof(header_t) + sizeof(sensor_data_t) + 1;
  sensor_data_t* sensor_data = (sensor_data_t*)pkt->data_crc;
  sensor_data->velocity[MOTOR_LEFT] = velocities_[MOTOR_LEFT];
  sensor_data->velocity[MOTOR_RIGHT] = velocities_[MOTOR_RIGHT];
}

void Control::set_motor_pwm(int motor, float value) {
  if (value >= 0.0) {
    motors_[motor][0]->write(value);
    motors_[motor][1]->write(0.0);
  } else {
    motors_[motor][0]->write(0.0);
    motors_[motor][1]->write(-value);
  }
}

void Control::pid_init(int motor, float kP, float kI, float kD, float period, float velocity_max) {
  pids_[motor] = new PID(kP,kI,kD,period);
  pids_[motor]->setInputLimits(-velocity_max, velocity_max);
  pids_[motor]->setOutputLimits(-1.0,1.0);
  pids_[motor]->setBias(0.0);
  pids_[motor]->setSetPoint(0.0);
}

void Control::control_helper(const void* p) {
  Control* instance = (Control*)p;
  instance->control_update();
}

void Control::control_update(void) {
  int32_t positions[2];
  
  sensors_->get_encoders(positions);

  for (uint32_t i=0; i<2; i++) {
    velocities_[i] = tick_to_angular_velocity_ * (float)(positions[i] - last_positions_[i]);
    last_positions_[i] = positions[i];
    pids_[i]->setProcessValue(velocities_[i]);
    pwms_[i] = pids_[i]->compute();
    if (fabs(pwms_[i]) < pid_dead_band_)
      pwms_[i] = 0.0;
  }

  set_motor_pwm(MOTOR_LEFT, pwms_[MOTOR_LEFT]);
  set_motor_pwm(MOTOR_RIGHT, pwms_[MOTOR_RIGHT]);
}
