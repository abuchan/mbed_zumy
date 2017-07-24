#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
/**
 * Packet type characters.
 */
#define PKT_TYPE_COMMAND  'C'
#define PKT_TYPE_SENSOR   'S'
#define PKT_TYPE_READ     'G'
#define PKT_TYPE_RESET    'R'
#define PKT_TYPE_TIME     'T'
#define PKT_TYPE_PID      'P'
#define PKT_TYPE_MARKER   'M'
#define PKT_TYPE_LASER    'L'

/**
 * Defines the total maximum size of a packet, including header
 */
#define MAX_PACKET_LENGTH 256

#define MARKER_N_PIXEL    6

/**
 * Packet structure definitions
 */
typedef struct header_t {
  uint8_t start;
  uint8_t length;
  char type;
  uint8_t flags;
  uint32_t sequence;
} header_t;

typedef struct packet_t {
  header_t header;
  uint8_t data_crc[MAX_PACKET_LENGTH-sizeof(header_t)];
} packet_t;

typedef union packet_union_t {
  packet_t packet;
  char raw[MAX_PACKET_LENGTH];
} packet_union_t;

typedef struct command_data_t {
  float left;
  float right;
} command_data_t;

typedef struct sensor_data_t {
  uint32_t time;
  float accel[3];
  float gyro[3];
  int32_t encoder[2];
  float velocity[2];
  float voltage;
  float current;
} sensor_data_t;

typedef struct read_data_t {
  int32_t period;
} read_data_t;

typedef struct time_data_t {
  uint32_t time;
} time_data_t;

typedef struct pid_data_t {
  float vel[2];
  float pwm[2];
} pid_data_t;

typedef struct marker_data_t {
  uint32_t colors[MARKER_N_PIXEL];
} marker_data_t;

typedef struct galvo_laser_t {
  float laser_cmd;
  int16_t galvo_cmd[2];
} galvo_laser_t;

#endif
