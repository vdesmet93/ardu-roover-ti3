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

  sendPacket(SONAR, 0);
  delay(PACKET_DELAY);
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

void receiveData()
{
  byte bytesRead = (byte)stream.Read(receivedBytes, byteCounter, Messages.MaxSize - byteCounter);
  byteCounter += bytesRead;
  switch (Messages.CheckMessage(receivedBytes, byteCounter))
  {
  case MessageCorrectness.CompleteAndCorrect:
    procesPacket(receivedBytes);
    byteCounter = 0;
    break;
  case MessageCorrectness.Incorrect:
    byteCounter = 0;
    PacketsDropped++;
    if (IncorrectMessage != null)
      IncorrectMessage(this, new EventArgs());
    break;
  case MessageCorrectness.NotComplete:
    //wait for to be complete of incorrect
    Console.Out.WriteLine("notcomplete");
    break;
  }
}

void int CheckMessage(byte[] receivedBytes, byte count)
{
  //check headers
  if (receivedBytes[PosHeader1] != Header1)
  {
    return MESSAGE_INCORRECT;
  }
  if (count < HeaderSize)
  {
    return MESSAGE_INCOMPLETE;
  }
  if (receivedBytes[PosHeader2] != Header2)
  {
    return MESSAGE_INCORRECT;
  }

  //check minimal size of a message
  if (count < MinMessageSizeRecieve)
  {
    return MESSAGE_INCOMPLETE;
  }

  //check if all bytes are recieved
  if( count < receivedBytes[PosCount]+3)
  {
    return MESSAGE_INCOMPLETE;
  }

  //calculate checksum and check
  int calculatedChecksum = calcChecksum(receivedBytes);
  int recievedCheckSum = (receivedBytes[count - 2] << 8) | receivedBytes[count - 1];
  if (calculatedChecksum != recievedCheckSum)
  {
    return MessageCorrectness.Incorrect;
  }
  return MessageCorrectness.CompleteAndCorrect;
}

private void procesPacket(byte[] newData)
{
  MessageCommandRecieve t = Messages.GetMessageType(newData);
  switch (t)
  {
  case MessageCommandRecieve.SipMoving:
  case MessageCommandRecieve.SipStopped:
    if (NewSipMessage != null)
    {
      NewSipMessage(this, new NewMessageEventArgs()
      {
        Message = Messages.ConvertToSipMessage(newData)
      }
      );
    }
    break;
  case MessageCommandRecieve.Sync0:
  case MessageCommandRecieve.Sync1:
    InitilizeFlag.Set();
    break;
  case MessageCommandRecieve.Sync2:
    Sync2Message sync2Message = Messages.ConvertToSync2Message(newData);
    this.Name = sync2Message.Name;
    this.Type = sync2Message.Type;
    this.SubType = sync2Message.Subtype;
    InitilizeFlag.Set();
    break;
  default:
    Console.WriteLine("Unkown message recieved of type: {0}", t);
    break;
  }
}

public static SipMessage ConvertToSipMessage(byte[] receivedBytes)
{
  SipMessage message = new SipMessage();
  message.MotorStatus = receivedBytes[PosCommand] == 0x32 ? MotorsStatus.Stopped : MotorsStatus.Moving;
  message.XPos	= receivedBytes[PosSipXPos1];
  message.XPos	|= receivedBytes[PosSipXPos2] << 8;
  message.YPos	= receivedBytes[PosSipYPos1];
  message.YPos	|= receivedBytes[PosSipYPos2] << 8;
  message.ThPos	= receivedBytes[PosSipThPos1];
  message.ThPos	|= receivedBytes[PosSipThPos2] << 8;
  message.LVel	= receivedBytes[PosSipLVel1];
  message.LVel	|= receivedBytes[PosSipLVel2] << 8;
  message.RVel	= receivedBytes[PosSipRVel1];
  message.RVel	|= receivedBytes[PosSipRVel2] << 8;
  message.Battery = receivedBytes[PosSipBattery];
  message.StallAndBumper = receivedBytes[PosSipStallAndBumpers1];
  message.StallAndBumper |= (ushort)(receivedBytes[PosSipStallAndBumpers2] << 8);
  message.Control = receivedBytes[PosSipControl1];
  message.Control |= receivedBytes[PosSipControl2] << 8;
  message.Flags = receivedBytes[PosSipFlags1];
  message.Flags = (ushort) (receivedBytes[PosSipFlags2] << 8);
  message.compass = receivedBytes[PosSipCompass];

  int numberOfSensors = receivedBytes[PosSipSonarCount];
  message.Sonar = new SonarValue[numberOfSensors];
  int index = PosSipSonarBegin;
  for (int i = 0; i < numberOfSensors; ++i)
  {
    byte sensorNumber = receivedBytes[index++];
    byte sensorLValue = receivedBytes[index++];
    byte sensorHValue = receivedBytes[index++];
    ushort sensorValue = (ushort)(sensorLValue + (sensorHValue << 8));
    message.Sonar[i] = new SonarValue()
    {
      SonarNumber = sensorNumber,
      Range = sensorValue
    };
  }

  message.GripState = receivedBytes[index + PosSipGripState - PosSipSonarBegin];
  message.AnPort = receivedBytes[index + PosSipAnPort - PosSipSonarBegin];
  message.Analog = receivedBytes[index + PosSipAnalog - PosSipSonarBegin];
  message.DigIn = receivedBytes[index + PosSipDigIn - PosSipSonarBegin];
  message.DigOut = receivedBytes[index + PosSipDigOut - PosSipSonarBegin];
  message.BatteryX10 = receivedBytes[index + PosSipBatteryX10_1 - PosSipSonarBegin];
  message.BatteryX10 |= receivedBytes[index + PosSipBatteryX10_2 - PosSipSonarBegin] << 8;
  message.ChargeState = receivedBytes[index + PosSipChargeState - PosSipSonarBegin];
  message.RotVel = receivedBytes[index + PosSipRotVel1 - PosSipSonarBegin];
  message.RotVel |= receivedBytes[index + PosSipRotVel2 - PosSipSonarBegin] << 8;
  message.FaultFlags = receivedBytes[index + PosSipFaultFlags1 - PosSipSonarBegin];
  message.FaultFlags |= receivedBytes[index + PosSipFaultFlags2 - PosSipSonarBegin] << 8;

  return message;
}

