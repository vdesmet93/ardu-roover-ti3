
#include <SoftwareSerial.h>
#include "commands.h"

#define DEBUG
#define RX 10
#define TX 11
#define JOYSTICK_RIGHT_X  A0
#define JOYSTICK_RIGHT_Y  A1
#define JOYSTICK_LEFT_Y  A2
#define JOYSTICK_LEFT_X  A3
#define SONAR_FREQUENCY  A3
#define DIRECTION  A5
#define KILL_SWITCH  4
#define MANUAL_AUTO  2
#define SYNC_DELAY  100
#define KILL_DELAY  25
#define MAX_SPEED  35000
#define REVERSE_THRESHOLD  2000
#define SPEED_THRESHOLD  1100
#define SPEED_COMPENSATOR  50
#define HALT_THRESHOLD  1100
#define ANGLE_THRESHOLD  1100
#define KILLSWITCH_THRESHOLD  1100
#define bit9600Delay 84  
#define BAUD_RATE  9600

SoftwareSerial mySerial(RX, TX); // RX, TX
int number, n, fd;
int current = 0;
boolean backward = false;
boolean halt = false;
int anglePulse;
int directionPulse;
unsigned int spd;
int killPulse;

void readFromRover()
{
  while(mySerial.available() > 0)
  {
    int i = mySerial.read();
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
  int i;
  unsigned char n;
  int c = 0;
  i = 3;
  n = buf[2] - 2;
  while (n > 1)
  {
    c += ((unsigned char)buf[i]<<8) | (unsigned char)buf[i+1];
    c = c & 0xffff;
    n -= 2;
    i += 2;
  }
  if (n > 0)
    c = c ^ (int)((unsigned char) buf[i]);
  return c;
}

int writeSerial(unsigned char* buf, int length)
{
  for(int i = 0; i < length; i++)
    mySerial.write(buf[i]);
  return length;
}

/**
 * Send a packet without arguments.
 * For example, SYNC0, OPEN, RESET, CLOSE
 */
void sendPacket(char command)
{
  // create array. Length = 6 because we don't have any arguments
  unsigned char buf[6];
  // header
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251
  // length
  buf[2] = 3; // command + 2 byte checksum
  // command
  buf[3] = command;
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char high = (checksum & 0xFF00) >> 8;
  char low = checksum & 0x00FF;
  // add checksum to array, high byte is first
  buf[4] = high;
  buf[5] = low;
  // send the array, print resultcode
  int resultcode = writeSerial (buf, 6);
}

/*
* Send a packet with 1 argument: ENABLE, SETA, SONAR
 */
void sendPacket(char command, int argument)
{
  //create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
  //2 checksum
  unsigned char buf[9];
  /* ================= HEADER ================= */
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251
  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = 6; 
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
  char highArgument = ( (argument & 0xFF00) >> 8);
  char lowArgument = (argument & 0x00FF);
  // add argument to array. 
  // Low byte first(instead of high-first for checksum, because f*ck consistency)
  buf[5] = lowArgument;
  buf[6] = highArgument;
  /* ================= CHECKSUM ================= */
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char highChecksum = (checksum & 0xFF00) >> 8;
  char lowChecksum = checksum & 0x00FF;
  // add checksum to array, high byte is first
  buf[7] = highChecksum;
  buf[8] = lowChecksum;
  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  int resultcode = writeSerial (buf, 9);
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
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251
  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = 6; 
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
  char highChecksum = (checksum & 0xFF00) >> 8;
  char lowChecksum = checksum & 0x00FF;
  // add checksum to array, high byte is first
  buf[bytes-2] = highChecksum;
  buf[bytes-1] = lowChecksum;
  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  int resultcode = writeSerial (buf, bytes);
}

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
  mySerial.begin(BAUD_RATE);
  delay(SYNC_DELAY);
  sendPacket(SYNC0);
  delay(SYNC_DELAY);
  sendPacket(SYNC1);
  delay(SYNC_DELAY);
  sendPacket(SYNC2);
  delay(SYNC_DELAY);
  sendPacket(OPEN);
  delay(SYNC_DELAY);
  sendPacket(SONAR, 0);
  delay(SYNC_DELAY);
  sendPacket(BUMPSTALL, 0);
  delay(SYNC_DELAY);
  sendPacket(ENABLE, 1);
  delay(SYNC_DELAY);
}

void loop()
{
  sendPacket(PULSE);
  anglePulse = pulseIn(JOYSTICK_RIGHT_X, HIGH);
  if(anglePulse > ANGLE_THRESHOLD)
  {
    int angle = 100 - ((anglePulse - ANGLE_THRESHOLD) / 8);
    angle -= 50;
    sendPacket(ROTATE, angle);
  }
  directionPulse = pulseIn(DIRECTION, HIGH);
  if(directionPulse < HALT_THRESHOLD)
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
      spd *= 4;
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
