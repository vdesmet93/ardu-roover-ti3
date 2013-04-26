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

void receiveData()
{
  unsigned char receivedBytes[MAX_DATA_SIZE];
  int bytesRead = 0;
  while(softSerial.available() > 0 && bytesRead < MAX_DATA_SIZE)
  {
    receivedBytes[bytesRead++] = softSerial.read();
  }

  switch(checkMessage(receivedBytes, bytesRead))
  {
  case MESSAGE_COMPLETE:
    //  procesPacket(receivedBytes);
    //    byteCounter = 0;
    break;
  case MESSAGE_INCORRECT:
    //    byteCounter = 0;
    //    P1acketsDropped++;
    //    if (IncorrectMessage != null)
    //      IncorrectMessage(this, new EventArgs());
    break;
  case MESSAGE_INCOMPLETE:
    //wait for to be complete of incorrect
    //    Console.Out.WriteLine("notcomplete");
    break;
  }
}

int checkMessage(unsigned char receivedBytes[], int count)
{
  //check headers
  if (receivedBytes[0] != HEADER_A ||
    receivedBytes[1] != HEADER_B)
  {
    return MESSAGE_INCORRECT;
  }
  if (count < BASE_PACKET_LENGTH)
  {
    return MESSAGE_INCOMPLETE;
  }
  //check if all bytes are recieved
  int packetSize = receivedBytes[PACKET_COUNT_POSITION];
  if( count == packetSize + PACKET_HEADER_LENGTH)
  {
    return MESSAGE_INCOMPLETE;
  }

  //calculate checksum and check
  int calculatedChecksum = getChecksum(receivedBytes);
  int recievedCheckSum = (receivedBytes[count - 2] << 8) | receivedBytes[count - 1];
  if (calculatedChecksum != recievedCheckSum)
  {
    return MESSAGE_INCORRECT;
  }
  return MESSAGE_COMPLETE;
}

struct SipMessage ConvertToSipMessage(byte receivedBytes[])
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
  /**
   * int numberOfSensors = receivedBytes[POS_SIP_SonarCount];
   * message.Sonar = new SonarValue[numberOfSensors];
   * int index = POS_SIP_SonarBegin;
   * for (int i = 0; i < numberOfSensors; ++i)
   * {
   * byte sensorNumber = receivedBytes[index++];
   * byte sensorLValue = receivedBytes[index++];
   * byte sensorHValue = receivedBytes[index++];
   * ushort sensorValue = (ushort)(sensorLValue + (sensorHValue << 8));
   * message.Sonar[i] = new SonarValue()
   * {
   * SonarNumber = sensorNumber,
   * Range = sensorValue
   * };
   * }
   * 
   * message.GripState = receivedBytes[index + POS_SIP_GripState - POS_SIP_SonarBegin];
   * message.AnPort = receivedBytes[index + POS_SIP_AnPort - POS_SIP_SonarBegin];
   * message.Analog = receivedBytes[index + POS_SIP_Analog - POS_SIP_SonarBegin];
   * message.DigIn = receivedBytes[index + POS_SIP_DigIn - POS_SIP_SonarBegin];
   * message.DigOut = receivedBytes[index + POS_SIP_DigOut - POS_SIP_SonarBegin];
   * message.BatteryX10 = receivedBytes[index + POS_SIP_BatteryX10_1 - POS_SIP_SonarBegin];
   * message.BatteryX10 |= receivedBytes[index + POS_SIP_BatteryX10_2 - POS_SIP_SonarBegin] << 8;
   * message.ChargeState = receivedBytes[index + POS_SIP_ChargeState - POS_SIP_SonarBegin];
   * message.RotVel = receivedBytes[index + POS_SIP_RotVel1 - POS_SIP_SonarBegin];
   * message.RotVel |= receivedBytes[index + POS_SIP_RotVel2 - POS_SIP_SonarBegin] << 8;
   * message.FaultFlags = receivedBytes[index + POS_SIP_FaultFlags1 - POS_SIP_SonarBegin];
   * message.FaultFlags |= receivedBytes[index + POS_SIP_FaultFlags2 - POS_SIP_SonarBegin] << 8;
   **/
  return message;
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
