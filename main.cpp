#include "mbed.h"

#include "protocol.h"
#include "packet_parser.h"
#include "sensors.h"
#include "control.h"
#include "markers.h"

#define PID_KP      1.0f
#define PID_KI      0.1f
#define PID_KD      0.0f

#define PID_PERIOD  0.01f

#define PID_IN_MIN  (-50.0f)
#define PID_IN_MAX  50.0f
#define PID_OUT_MIN (-1.0f)
#define PID_OUT_MAX 1.0f

#define TICK_PER_REV 1200

#define PID_DEAD_BAND 0.03f

#define SERIAL_BAUDRATE 230400

#define VOLTAGE_PIN   p15

#define L_ENC_A_PIN   p29
#define L_ENC_B_PIN   p30
#define R_ENC_A_PIN   p11
#define R_ENC_B_PIN   p12

#define IMU_SDA_PIN   p9
#define IMU_SCL_PIN   p10

#define L_MOT_0_PIN   p21
#define L_MOT_1_PIN   p22
#define R_MOT_0_PIN   p23
#define R_MOT_1_PIN   p24

#define MARKER_PIN    p5

#define GALVO_SEL_PIN   p19
#define GALVO_DIR_PIN   p18
#define GALVO_PULSE_PIN p17
#define GALVO_SENSE_PIN p16

#define LASER_PWM_PIN   p25

void fill_time_packet(packet_t* pkt, uint32_t time) {
  pkt->header.type = PKT_TYPE_TIME;
  pkt->header.length = sizeof(header_t) + sizeof(time_data_t) + 1;
  time_data_t* time_data = (time_data_t*)pkt->data_crc;
  time_data->time = time;
}

extern "C" void mbed_reset();

int main() {
  
  DigitalOut led1(LED1);
  DigitalOut led4(LED4);
  
  led1 = 0;
  led4 = 0;

  Timer system_timer;

  system_timer.start();
  uint32_t last_time = system_timer.read_ms();
  uint32_t current_time = last_time;
  
  PacketParser parser(SERIAL_BAUDRATE, USBTX, USBRX, LED2, LED3);

  packet_union_t* recv_pkt = NULL;
  packet_union_t* send_pkt = NULL;
  command_data_t* command;

  send_pkt = parser.get_send_packet();
  if (send_pkt != NULL) {
    fill_time_packet(&(send_pkt->packet), system_timer.read_us());
    parser.send_packet(send_pkt);
  }
  
  Sensors sensors(
    &system_timer,
    VOLTAGE_PIN,
    L_ENC_A_PIN, L_ENC_B_PIN,
    R_ENC_A_PIN, R_ENC_B_PIN, TICK_PER_REV,
    IMU_SDA_PIN, IMU_SCL_PIN
  );
  
  Control control(
    L_MOT_0_PIN, L_MOT_1_PIN, R_MOT_0_PIN, R_MOT_1_PIN,
    &sensors, TICK_PER_REV,
    PID_KP, PID_KI, PID_KD, PID_PERIOD, PID_IN_MAX,
    PID_DEAD_BAND,
    GALVO_SEL_PIN, GALVO_DIR_PIN, GALVO_PULSE_PIN, GALVO_SENSE_PIN,
    LASER_PWM_PIN
  );
  
  Markers markers(MARKER_PIN);

  led1 = 1;

  packet_union_t* sensor_pkt = parser.get_send_packet();

  while(1) {
     
    recv_pkt = parser.get_received_packet();

    if (recv_pkt != NULL) {
      
      switch (recv_pkt->packet.header.type) {
        
        case PKT_TYPE_RESET:
          mbed_reset();
          break;
        
        case PKT_TYPE_COMMAND:
          command = (command_data_t*)recv_pkt->packet.data_crc;
          control.set_setpoints(command->left, command->right);
          break;

        case PKT_TYPE_TIME:
          led4 = ((recv_pkt->packet.header.flags) & 1);
          send_pkt = parser.get_send_packet();
          if (send_pkt != NULL) {
            fill_time_packet(&(send_pkt->packet), system_timer.read_us());
            parser.send_packet(send_pkt);
          }
          break;
        
        case PKT_TYPE_READ:
          if (sensor_pkt != NULL) {
            if(sensors.fill_sensor_packet(&(sensor_pkt->packet))) {
              control.fill_sensor_packet(&(sensor_pkt->packet));
              parser.send_packet(sensor_pkt);
              sensor_pkt = parser.get_send_packet();
            }
          } else {
            sensor_pkt = parser.get_send_packet();
          }
          break;

        case PKT_TYPE_MARKER:
          markers.update((marker_data_t*)recv_pkt->packet.data_crc);
          break;
        
        case PKT_TYPE_LASER:
          led4 = !led4;
          galvo_laser_t* galvo_laser = (galvo_laser_t*)recv_pkt->packet.data_crc;
          control.set_laser(galvo_laser->laser_cmd);
          control.set_galvo(galvo_laser->galvo_cmd[0], galvo_laser->galvo_cmd[1]);
          break;
      }

      parser.free_received_packet(recv_pkt);
    }
    
    current_time = system_timer.read_ms();

    if (current_time - last_time > 500) {
      last_time = current_time;
      led1 = !led1;
    }
  
    Thread::yield();
  }
}
