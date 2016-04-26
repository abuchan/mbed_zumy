#include "mbed.h"

#include "protocol.h"
#include "packet_parser.h"
#include "sensors.h"
#include "control.h"

#define PID_KP      1.0f
#define PID_KI      0.1f
#define PID_KD      0.0f

#define PID_PERIOD  0.01f

#define PID_IN_MIN  (-50.0f)
#define PID_IN_MAX  50.0f
#define PID_OUT_MIN (-1.0f)
#define PID_OUT_MAX 1.0f

#define TICK_PER_REV 600

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
  
  led1 = 1;

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
    PID_KP, PID_KI, PID_KD, PID_PERIOD, PID_IN_MAX, PID_DEAD_BAND
  );
 
  led4 = 1;

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
      }

      parser.free_received_packet(recv_pkt);
    }
    
    current_time = system_timer.read_ms();

    if (current_time - last_time > 500) {
      last_time = current_time;
      led1 = !led1;
      led4 = !led4;
    }
  
    Thread::yield();
  }
}
