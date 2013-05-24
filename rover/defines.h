#pragma once

#define DEBUG
#define RX 10
#define TX 11
#define BAUD_RATE  9600
#define BASE_PACKET_LENGTH  6
#define BYTE_SHIFT  8
#define AND_MSB  0xFF00
#define AND_LSB  0x00FF
#define ARG_LENGTH  3
#define HEADER_A  0xFA
#define HEADER_B  0xFB
#define HEADER_LENGTH 2
#define CHECKSUM_LENGTH 2
#define JOYSTICK_RIGHT_X  A0
#define JOYSTICK_RIGHT_Y  A1
#define JOYSTICK_LEFT_Y  A2
#define JOYSTICK_LEFT_X  A3
#define SONAR_FREQUENCY  A4
#define DIRECTION  A5
#define KILL_SWITCH  4
#define MANUAL_AUTO  2
#define SYNC_DELAY  25
#define PACKET_DELAY  5
#define KILL_DELAY  20
#define MAX_SPEED  1200
#define MAX_INTEGER 35000
#define FORWARD_THRESHOLD 700
#define REVERSE_THRESHOLD  2000
#define SPEED_THRESHOLD  1100
#define SPEED_COMPENSATOR  50
#define HALT_THRESHOLD  1100
#define ANGLE_THRESHOLD  1100
#define DEAD_ZONE 3
#define KILLSWITCH_THRESHOLD  1100
#define ANGLE_MULTIPLIER  1.5
#define BAUD_RATE  9600
#define PIONEER_BAUD_RATE  9600

#define MESSAGE_COMPLETE 0
#define MESSAGE_INCOMPLETE 1
#define MESSAGE_INCORRECT 2

#define PACKET_COUNT_POSITION 2
#define PACKET_HEADER_LENGTH 3

#define SIP_MOTOR_STOPPED 0
#define SIP_MOTOR_MOVING 1
#define MAX_DATA_SIZE 128

#define POS_HEADER_A 0
#define POS_HEADER_B 1

#define SONAR_ARRAY (int[]) { 0, 0, 45, 80, 80, 45, 0, 0 }
#define SONAR_KILLSWITCH_THRESHOLD 500
#define SONAR_SIDE_KS_THRESHOLD 300
#define SONAR_DANGER_THRESHOLD 1000
#define MAX_ROTATION 300

#define SONAR_DANGER_BASE 200
#define SONAR_ROTATION_BASE 180
#define KILLSWITCH_THRESHOLD_BASE 200
#define SONAR_ROTATION_MULTIPLIER 4
#define SONAR_DANGER_MULTIPLIER 2
#define KILLSWITCH_THRESHOLD_DIVIDER 3
#define KILLSWITCH_SIDE_THRESHOLD_DIVIDER 2

#define POS_COMMAND 3
#define POS_SIP_XPOS_1 4
#define POS_SIP_XPOS_2 5
#define POS_SIP_YPOS_1 6
#define POS_SIP_YPOS_2 7
#define POS_SIP_THPOS_1 8
#define POS_SIP_THPOS_2 9
#define POS_SIP_LVEL_1 10
#define POS_SIP_LVEL_2 11
#define POS_SIP_RVEL_1 1
#define POS_SIP_RVEL_2 13
#define POS_SIP_BATTERY 14
#define POS_SIP_STALL_AND_BUMPERS_1 15
#define POS_SIP_STALL_AND_BUMPERS_2 16
#define POS_SIP_CONTROL_1 17
#define POS_SIP_CONTROL_2 18
#define POS_SIP_FLAGS_1 19
#define POS_SIP_FLAGS_2 20
#define POS_SIP_COMPASS 21
#define POS_SIP_SONAR_COUNT 22
#define POS_SIP_SONAR_BEGIN 23
#define POS_SIP_SONAR_NUMBER 0
#define POS_SIP_SONAR_RANGE 1
#define POS_SIP_GRIP_STATE 23
#define POS_SIP_AN_PORT 24
#define POS_SIP_ANALOG 25
#define POS_SIP_DIG_IN 26
#define POS_SIP_DIG_OUT 27
#define POS_SIP_BATTERY_1 28
#define POS_SIP_BATTERY_2 29
#define POS_SIP_CHARGE_STATE 30
#define POS_SIP_ROT_VEL_1 31
#define POS_SIP_ROT_VEL_2 32
#define POS_SIP_FAULT_FLAGS_1 33
#define POS_SIP_FAULT_FLAGS_2 34

