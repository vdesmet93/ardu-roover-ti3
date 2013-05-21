#include <Arduino.h>

#include "pioneer.h"
#include "commands.h"
#include "defines.h"

struct SipMessage lastMessage;

unsigned char receivedBytes[MAX_DATA_SIZE];
int bytesRead = 0;
int packetSize = 0;

void initializeConnection()
{
  sendPacket(SYNC0);
  delay(SYNC_DELAY);
  sendPacket(SYNC1);
  delay(SYNC_DELAY);
  sendPacket(SYNC2);
  delay(SYNC_DELAY);
  sendPacket(OPEN);
  delay(SYNC_DELAY);

  sendPacket(ENABLE, 1);
  delay(PACKET_DELAY);

  sendPacket(BUMPSTALL, 0);
  delay(PACKET_DELAY);
}

/**
 * Checksum generator for the buffer which will be send
 */
inline int getChecksum(unsigned char* buf)
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
    c ^= (int)((unsigned char) buf[i]); 
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

void sendSonarSpeed(int interval)
{
  sendPacket(SONARCYCLE, interval);
  delay(PACKET_DELAY);
}

int writeSerial(unsigned char* buf, int length)
{
  for(int i = 0; i < length; i++)
    Serial1.write(buf[i]);
  return length;
}

void receiveData()
{
  while(Serial1.available() > 0) {

    receivedBytes[bytesRead] = Serial1.read();

    if(bytesRead == POS_HEADER_A && receivedBytes[bytesRead] != HEADER_A) 
    {
      continue;
    }
    else if(bytesRead == POS_HEADER_B && receivedBytes[bytesRead] != HEADER_B) 
    {
      bytesRead = 0;
      continue;
    }
    else if(bytesRead == PACKET_COUNT_POSITION) 
    {
      packetSize = PACKET_HEADER_LENGTH + receivedBytes[bytesRead];
    }

    bytesRead++;

    if(bytesRead == packetSize)
    {
      // Packet is complete
      parseMessage();
    }
  }
}

void parseMessage()
{
  // Check the message
  switch(checkMessage(receivedBytes, bytesRead))
  {
  case MESSAGE_COMPLETE:
    Serial.println("Message correct!");
    Serial.print("Length: ");
    Serial.println(bytesRead);
    free(lastMessage.sonar);
    lastMessage = convertToSipMessage(receivedBytes);
    break;
  case MESSAGE_INCORRECT:
    Serial.println("Message incorrect"); 

    // Check for new header bytes in current packet
    for(int i = HEADER_LENGTH; i < (bytesRead - 1); i++)
    {
      if(receivedBytes[i] == HEADER_A &&
        receivedBytes[i + 1] == HEADER_B) 
      {
        shiftBytes(i);
        return;
      } 
    }

    break;
  case MESSAGE_INCOMPLETE:
    Serial.println("Message incomplete");
    break;
  }

  bytesRead = 0;
  packetSize = 0;
}

void shiftBytes(int startPosOfPacket)
{
  // Valid header found
  bytesRead -= startPosOfPacket;

  // Move all the bytes
  for(int i = 0; i < bytesRead; i++)
  {
    receivedBytes[i] = receivedBytes[i + startPosOfPacket]; 
  }

  // Update Packet Size
  packetSize = PACKET_HEADER_LENGTH + receivedBytes[PACKET_COUNT_POSITION];
}

int checkMessage(unsigned char receivedBytes[], int count)
{
  
  //calculate checksum and check
  int calculatedChecksum = getChecksum(receivedBytes);
  int recievedCheckSum = (receivedBytes[count - 2] << BYTE_SHIFT) | receivedBytes[count - 1];
  
  if (calculatedChecksum != recievedCheckSum)
  {
    return MESSAGE_INCORRECT;
  }
  return MESSAGE_COMPLETE;
}

struct SipMessage convertToSipMessage(unsigned char receivedBytes[])
{
  struct SipMessage message;
  message.motorStatus = receivedBytes[POS_COMMAND] == 0x32 ? SIP_MOTOR_STOPPED : SIP_MOTOR_MOVING;
  message.xPos	= receivedBytes[POS_SIP_XPOS_1];
  message.xPos	|= receivedBytes[POS_SIP_XPOS_2] << 8;
  message.yPos	= receivedBytes[POS_SIP_YPOS_1];
  message.yPos	|= receivedBytes[POS_SIP_YPOS_2] << 8;
  message.thPos	= receivedBytes[POS_SIP_THPOS_1];
  message.thPos	|= receivedBytes[POS_SIP_THPOS_2] << 8;
  message.lVel	= receivedBytes[POS_SIP_LVEL_1];
  message.lVel	|= receivedBytes[POS_SIP_LVEL_2] << 8;
  message.rVel	= receivedBytes[POS_SIP_RVEL_1];
  message.rVel	|= receivedBytes[POS_SIP_RVEL_2] << 8;
  message.batteryLevel = receivedBytes[POS_SIP_BATTERY];
  message.stallAndBumper = receivedBytes[POS_SIP_STALL_AND_BUMPERS_1];
  message.stallAndBumper |= (unsigned char)(receivedBytes[POS_SIP_STALL_AND_BUMPERS_2] << 8);
  message.control = receivedBytes[POS_SIP_CONTROL_1];
  message.control |= receivedBytes[POS_SIP_CONTROL_2] << 8;
  message.flags = receivedBytes[POS_SIP_FLAGS_1];
  message.flags = (unsigned char) (receivedBytes[POS_SIP_FLAGS_2] << 8);
  message.compass = receivedBytes[POS_SIP_COMPASS];

  int numberOfSensors = receivedBytes[POS_SIP_SONAR_COUNT];
  message.sonar = (int *) malloc(sizeof(int) * numberOfSensors);
  int index = POS_SIP_SONAR_BEGIN;
  for (int i = 0; i < numberOfSensors; ++i)
  {
    byte sensorNumber = receivedBytes[index++];
    byte sensorLValue = receivedBytes[index++];
    byte sensorHValue = receivedBytes[index++];
    int sensorValue = (int)(sensorLValue + (sensorHValue << 8));
    message.sonar[i] = sensorValue;
  }

  message.gripState = receivedBytes[index + POS_SIP_GRIP_STATE - POS_SIP_SONAR_BEGIN];
  message.anPort = receivedBytes[index + POS_SIP_AN_PORT - POS_SIP_SONAR_BEGIN];
  message.analog = receivedBytes[index + POS_SIP_ANALOG - POS_SIP_SONAR_BEGIN];
  message.digIn = receivedBytes[index + POS_SIP_DIG_IN - POS_SIP_SONAR_BEGIN];
  message.digOut = receivedBytes[index + POS_SIP_DIG_OUT - POS_SIP_SONAR_BEGIN];
  message.batteryX10 = receivedBytes[index + POS_SIP_BATTERY_1 - POS_SIP_SONAR_BEGIN];
  message.batteryX10 |= receivedBytes[index + POS_SIP_BATTERY_2 - POS_SIP_SONAR_BEGIN] << 8;
  message.chargeState = receivedBytes[index + POS_SIP_CHARGE_STATE - POS_SIP_SONAR_BEGIN];

  return message;
}

byte getMessageType(unsigned char receivedBytes[])
{
  return receivedBytes[POS_COMMAND];
}

void readFromRover()
{
  while(Serial1.available() > 0)
  {
    int i = Serial1.read();

    char str[256];
    sprintf(str, "%x ", i);
    Serial.println(str);
  }
}




