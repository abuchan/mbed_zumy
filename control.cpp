#include "control.h"

const int Control::laser_periods_[LASER_N_LEVELS] = {10, 10, 10, 10};
const int Control::laser_duties_[LASER_N_LEVELS] = {0, 30, 60, 100};

Control::Control(
    PinName left_mot_0_pin, PinName left_mot_1_pin, 
    PinName right_mot_0_pin, PinName right_mot_1_pin, 
    Sensors *sensors, uint32_t tick_per_rev,
    float kP, float kI, float kD, float period, float velocity_max,
    float pid_dead_band,
    PinName galvo_sel_pin, PinName galvo_dir_pin, PinName galvo_pulse_pin, PinName galvo_sense_pin,
    PinName laser_pwm_pin
  ) :
    sensors_(sensors),
    control_timer_(&Control::control_helper,osTimerPeriodic,this),
    pid_dead_band_(pid_dead_band),
    galvo_sel_pin_(galvo_sel_pin), galvo_dir_pin_(galvo_dir_pin), galvo_pulse_pin_(galvo_pulse_pin),
    galvo_sense_pin_(galvo_sense_pin, PullNone),
    laser_pwm_pin_(laser_pwm_pin)
  {

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

  laser_pwm_pin_.write(laser_duties_[0]);
  laser_pwm_pin_.period_us(laser_periods_[0]);
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
  pkt->header.flags = galvo_sense_[0] | (galvo_sense_[1] << 1);
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

void Control::set_galvo(int steps_0, int steps_1) {
  
  if (galvo_count_[0] == 0 && galvo_count_[1] == 0) {
    galvo_count_[0] = steps_0;
    galvo_count_[1] = steps_1;
  }
}

void Control::set_laser(float laser_power) {
  if (laser_power < 0.0) {
    laser_power = 0.0;
  } else if (laser_power > 1.0) {
    laser_power = 1.0;
  }

  int laser_idx = floor(laser_power * (LASER_N_LEVELS-1));

  if(laser_state_ == LASER_STATE_IDLE) {
    laser_period_ = laser_periods_[laser_idx];
    laser_duty_ = laser_duties_[laser_idx];
    laser_state_ = LASER_STATE_BLANK;
  }
}

void Control::control_update(void) {
  control_velocity();
  control_galvo();
  control_laser();
}

void Control::control_velocity(void) {
  float positions[2];
  
  sensors_->get_angles(positions);

  for (uint32_t i=0; i<2; i++) {
    velocities_[i] = tick_to_angular_velocity_ * (positions[i] - last_positions_[i]);
    last_positions_[i] = positions[i];
    pids_[i]->setProcessValue(velocities_[i]);
    pwms_[i] = pids_[i]->compute();
    if (fabs(pwms_[i]) < pid_dead_band_)
      pwms_[i] = 0.0;
  }

  set_motor_pwm(MOTOR_LEFT, pwms_[MOTOR_LEFT]);
  set_motor_pwm(MOTOR_RIGHT, pwms_[MOTOR_RIGHT]);
}

void Control::control_galvo(void) {
  switch(galvo_state_) {
    case GALVO_STATE_IDLE:
      current_sel_ = galvo_sel_pin_.read();
      galvo_sense_[current_sel_] = galvo_sense_pin_.read();

      if (galvo_count_[0] != 0) {
        galvo_sel_pin_.write(0);
        galvo_dir_pin_.write(galvo_count_[0] > 0 ? 1 : 0);
        galvo_pulse_pin_.write(1);
        
        galvo_delta_ = galvo_count_[0] > 0 ? -1 : 1;
        galvo_state_ = GALVO_STATE_MOVE_0;
        galvo_count_[0] += galvo_delta_;
      } else if (galvo_count_[1] != 0) {
        galvo_sel_pin_.write(1);
        galvo_dir_pin_.write(galvo_count_[1] > 0 ? 1 : 0);
        galvo_pulse_pin_.write(1);
        
        galvo_delta_ = galvo_count_[1] > 0 ? -1 : 1;
        galvo_state_ = GALVO_STATE_MOVE_1;
        galvo_count_[1] += galvo_delta_;
      } else {
        galvo_sel_pin_.write(!current_sel_);
      }
      break;
    
    case GALVO_STATE_MOVE_0:
      if (galvo_count_[0] == 0) {
        if (galvo_count_[1] != 0) {
          galvo_pulse_pin_.write(0);
          galvo_sel_pin_.write(1);
          galvo_dir_pin_.write(galvo_count_[1] > 0 ? 1 : 0);
          galvo_pulse_pin_.write(1);
          
          galvo_delta_ = galvo_count_[1] > 0 ? -1 : 1;
          galvo_state_ = GALVO_STATE_MOVE_1;
          galvo_count_[1] += galvo_delta_;
        } else {
          galvo_pulse_pin_.write(0);
          galvo_state_ = GALVO_STATE_IDLE;
        }
      } else {
        galvo_count_[0] += galvo_delta_;
      }
      break;

    case GALVO_STATE_MOVE_1:
      if (galvo_count_[1] == 0) {
        galvo_pulse_pin_.write(0);
        galvo_state_ = GALVO_STATE_IDLE;
      } else {
        galvo_count_[1] += galvo_delta_;
      }
      break;

    default:
      galvo_pulse_pin_.write(0);
      galvo_count_[0] = 0;
      galvo_count_[1] = 0;
      galvo_state_ = GALVO_STATE_IDLE;
      break;
  }
}

void Control::control_laser(void) {
  switch(laser_state_) {
    case LASER_STATE_IDLE:
      break;

    case LASER_STATE_BLANK:
      laser_pwm_pin_.write(0.0);
      laser_state_ = LASER_STATE_CHANGE;
      break;

    case LASER_STATE_CHANGE:
      laser_pwm_pin_.period_us(laser_period_);
      laser_pwm_pin_.write(((float)laser_duty_)/100.0);
      laser_state_ = LASER_STATE_IDLE;
      break;

    default:
      laser_state_ = LASER_STATE_IDLE;
      laser_pwm_pin_.write(0.0);
      break;
  }
}
