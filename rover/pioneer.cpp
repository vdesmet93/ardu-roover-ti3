#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pioneer.h"
#include "commands.h"
#include "defines.h"

SoftwareSerial softSerial(RX, TX);

void initializeConnection()
{
  softSerial.begin(BAUD_RATE);
  sendPacket(SYNC0);
  delay(SYNC_DELAY);
  sendPacket(SYNC1);
  delay(SYNC_DELAY);
  sendPacket(SYNC2);
  delay(SYNC_DELAY);
  sendPacket(OPEN);
  delay(SYNC_DELAY); 

  sendPacket(BUMPSTALL, 0);
  delay(PACKET_DELAY);
  sendPacket(ENABLE, 1);
  delay(PACKET_DELAY);
}

/**
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

/**
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

/**
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

void enableSonar()
{
  sendPacket(SONAR, 1);
  delay(PACKET_DELAY);
}

void sendSonarSpeed(int interval)
{
  sendPacket(SONARCYCLE, interval);
  delay(PACKET_DELAY);
}

int writeSerial(unsigned char* buf, int length)
{
  for(int i = 0; i < length; i++)
    softSerial.write(buf[i]);
  return length;
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

