#include "sensors.h"

Sensors::Sensors(
  Timer *system_timer,
  PinName voltage_pin,
  PinName l_enc_a_pin, PinName l_enc_b_pin,
  PinName r_enc_a_pin, PinName r_enc_b_pin, uint32_t tick_per_rev,
  PinName imu_sda_pin, PinName imu_scl_pin) :

  system_timer_(system_timer),
  voltage_pin_(voltage_pin),
  left_qei_(l_enc_a_pin, l_enc_b_pin, NC, tick_per_rev),
  right_qei_(r_enc_a_pin, r_enc_b_pin, NC, tick_per_rev),
  mpu6050_(imu_sda_pin, imu_scl_pin) {}

float Sensors::get_voltage(void) {
  return voltage_pin_.read();
}

void Sensors::get_encoders(int32_t (&encoders)[2]) {
  encoders[0] = left_qei_.getPulses();
  encoders[1] = -right_qei_.getPulses();
}

bool Sensors::get_imu(sensor_data_t* sensor_data) {
  sensor_data->time = system_timer_->read_us();
  return mpu6050_.readCalibAccelGyroData(sensor_data->accel, sensor_data->gyro);
}

bool Sensors::fill_sensor_packet(packet_t* pkt) {
  pkt->header.type = PKT_TYPE_SENSOR;
  pkt->header.length = sizeof(header_t) + sizeof(sensor_data_t) + 1;
  sensor_data_t* sensor_data = (sensor_data_t*)pkt->data_crc;
  sensor_data->voltage = get_voltage();
  get_encoders(sensor_data->encoder);
  bool valid = get_imu(sensor_data);
  pkt->header.flags = valid ? 0 : 1;
  return valid;
}
