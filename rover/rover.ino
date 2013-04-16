#include <SoftwareSerial.h>
#include "commands.h"

#define DEBUG
#define RX 10
#define TX 11
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
#define KILL_DELAY  25
#define MAX_SPEED  35000
#define FORWARD_THRESHOLD 700
#define REVERSE_THRESHOLD  2000
#define SPEED_THRESHOLD  1100
#define SPEED_COMPENSATOR  50
#define HALT_THRESHOLD  1100
#define ANGLE_THRESHOLD  1100
#define DEAD_ZONE 3
#define KILLSWITCH_THRESHOLD  1100
#define ANGLE_MULTIPLIER  1.5
#define SPEED_MULTIPLIER  4
#define BASE_PACKET_LENGTH  6
#define BYTE_SHIFT  8
#define AND_MSB  0xFF00
#define AND_LSB  0x00FF
#define ARG_LENGTH  3
#define HEADER_A  0xFA
#define HEADER_B  0xFB
#define bit9600Delay  84
#define BAUD_RATE  9600

SoftwareSerial softSerial(RX, TX); // RX, TX
int number, n, fd, anglePulse, directionPulse, killPulse;
int current = 0;
boolean backward = false;
boolean halt = false;
unsigned int spd;

void setup() 
{
  pinMode(RX,INPUT);
  pinMode(TX, OUTPUT);
  pinMode(JOYSTICK_RIGHT_X, INPUT);
  pinMode(JOYSTICK_RIGHT_Y, INPUT);
  pinMode(JOYSTICK_LEFT_X, INPUT);
  pinMode(JOYSTICK_LEFT_X, INPUT);
  pinMode(SONAR_FREQUENCY, INPUT);
  pinMode(DIRECTION, INPUT);
  pinMode(KILL_SWITCH, INPUT);
  pinMode(MANUAL_AUTO, INPUT);
  digitalWrite(TX, HIGH);
  digitalWrite(RX, LOW);
  Serial.begin(BAUD_RATE);
  softSerial.begin(BAUD_RATE);
  sendPacket(SYNC0);
  delay(SYNC_DELAY);
  sendPacket(SYNC1);
  delay(SYNC_DELAY);
  sendPacket(SYNC2);
  delay(SYNC_DELAY);
  sendPacket(OPEN);
  delay(SYNC_DELAY);
  sendPacket(SONAR, 0);
  delay(PACKET_DELAY);
  sendPacket(BUMPSTALL, 0);
  delay(PACKET_DELAY);
  sendPacket(ENABLE, 1);
  delay(PACKET_DELAY);
}

void readFromRover()
{
  while(softSerial.available() > 0)
  {
    int i = softSerial.read();
    char str[256];
    sprintf(str, "%x ", i);
    Serial.println(str);
  }
}

/*
* Checksum generator for the buffer which will be send
 */
int getChecksum(unsigned char* buf)
{
  int i = 3;
  unsigned char n = buf[2] - 2;
  int c = 0;
  while (n > 1)
  {
    c += ((unsigned char) buf[i] << BYTE_SHIFT) | (unsigned char) buf[i+1];
    c = c & 0xFFFF;
    n -= 2;
    i += 2;
  }
  if (n > 0)
    c ^= (int)((unsigned char) buf[i]); //TODO: Replace with ^= if possible.
  return c;
}

int writeSerial(unsigned char* buf, int length)
{
  for(int i = 0; i < length; i++)
    softSerial.write(buf[i]);
  return length;
}

/**
 * Send a packet without arguments.
 * For example, SYNC0, OPEN, RESET, CLOSE
 */
void sendPacket(char command)
{
  // create array. Length = 6 because we don't have any arguments
  unsigned char buf[BASE_PACKET_LENGTH];
  buf[0] = HEADER_A;
  buf[1] = HEADER_B;
  // length
  buf[2] = ARG_LENGTH; // command + 2 byte checksum
  // command
  buf[3] = command;
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char msb = (checksum & AND_MSB) >> BYTE_SHIFT;
  char lsb = checksum & AND_LSB;
  // add checksum to array, high byte is first
  buf[4] = msb;
  buf[5] = lsb;
  // send the array, print resultcode
  writeSerial(buf, BASE_PACKET_LENGTH);
}

/*
 * Send a packet with 1 argument: ENABLE, SETA, SONAR
 */
void sendPacket(char command, int argument)
{
  //create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
  //2 checksum
  unsigned char buf[BASE_PACKET_LENGTH + ARG_LENGTH];
  /* ================= HEADER ================= */
  buf[0] = HEADER_A;
  buf[1] = HEADER_B;
  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = BASE_PACKET_LENGTH; 
  /* ================= COMMAND ================= */
  buf[3] = command;
  /* ============== ARGUMENT TYPE ============== */
  if(argument >= 0)
    buf[4] = (unsigned char) ARG_POS_INT;
  else
  {
    argument *= -1;
    buf[4] = (unsigned char) ARG_NEG_INT;
  }
  /* ================= ARGUMENT ================= */
  // parse argument high and low
  char highArgument = ((argument & AND_MSB) >> BYTE_SHIFT);
  char lowArgument = (argument & AND_LSB);
  // add argument to array. 
  // Low byte first(instead of high-first for checksum, because f*ck consistency)
  buf[5] = lowArgument;
  buf[6] = highArgument;
  /* ================= CHECKSUM ================= */
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char highChecksum = (checksum & AND_MSB) >> BYTE_SHIFT;
  char lowChecksum = checksum & AND_LSB;
  // add checksum to array, high byte is first
  buf[7] = highChecksum;
  buf[8] = lowChecksum;
  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  writeSerial(buf, 9);
}

/*
 * Send a packet with a string as argument
 */
void sendPacket(char command, char* argument, int size)
{
  // create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
  //2 checksum
  int bytes = size + 7;
  unsigned char buf[bytes];
  /* ================= HEADER ================= */
  buf[0] = HEADER_A;
  buf[1] = HEADER_B;
  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = BASE_PACKET_LENGTH; 
  /* ================= COMMAND ================= */
  buf[3] = command;
  /* ============== ARGUMENT TYPE ============== */
  buf[4] = (unsigned char) ARG_STRING;
  /* ================= ARGUMENT ================= */
  buf[5] = size;
  // parse argument high and low
  printf("parsing\n");
  char c = *argument;
  int idx = 6;
  while(c != '\0' && (idx - 6) != size)
  {
    printf("P: %c\n", c);
    buf[idx] = c;
    argument++;
    idx++;
    c = *argument;
  }
  /* ================= CHECKSUM ================= */
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char highChecksum = (checksum & AND_MSB) >> BYTE_SHIFT;
  char lowChecksum = checksum & AND_LSB;
  // add checksum to array, high byte is first
  buf[bytes-2] = highChecksum;
  buf[bytes-1] = lowChecksum;
  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  writeSerial(buf, bytes);
}

void loop()
{
  sendPacket(PULSE);
  anglePulse = pulseIn(JOYSTICK_RIGHT_X, HIGH);
  if(anglePulse > ANGLE_THRESHOLD)
  {
    int angle = 100 - ((anglePulse - ANGLE_THRESHOLD) / 8);
    angle -= 50;
    if(-DEAD_ZONE < angle && DEAD_ZONE > angle)
    {
      angle = 0;
    }
    sendPacket(ROTATE, angle * ANGLE_MULTIPLIER);
  }
  directionPulse = pulseIn(DIRECTION, HIGH);
  if(directionPulse > FORWARD_THRESHOLD && directionPulse < HALT_THRESHOLD)
  {
    halt = false;
    backward = false;
  }
  else if(directionPulse > HALT_THRESHOLD && directionPulse < REVERSE_THRESHOLD)
  {
    halt = true;
    backward = false;
  }
  else if(directionPulse > REVERSE_THRESHOLD)
  {
    backward = true;
    halt = false;
  }
  spd = pulseIn(JOYSTICK_LEFT_Y, HIGH);
  if(spd > SPEED_THRESHOLD)
  {
    spd = spd - (SPEED_THRESHOLD + SPEED_COMPENSATOR);
    if(spd > 0 && spd < MAX_SPEED)
    {
      spd *= SPEED_MULTIPLIER;
      if(halt)
        spd = 0;
      if(backward)
        sendPacket(VEL, -spd);
      else
        sendPacket(VEL, spd);
    }
    else
      sendPacket(VEL, 0);
  }
  else
    sendPacket(PULSE);
  killPulse = pulseIn(KILL_SWITCH, HIGH);
  if(killPulse > KILLSWITCH_THRESHOLD)
  {
    sendPacket(E_STOP);
    delay(KILL_DELAY);
  }
}
