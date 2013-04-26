#pragma once


void initializeConnection();
void sendPacket(char command);
void sendPacket(char command, int argument);
void sendPacket(char command, char* argument, int size);
int writeSerial(unsigned char* buf, int length);
void readFromRover();
int checkMessage(unsigned char receivedBytes[], int count);

struct SipMessage {
  unsigned char motorStatus;
  int xPos;
  int yPos;
  int thPos;
  int lVel;
  int rVel;
  int batteryLevel;
  int stallAndBumper;
  int control;
  int flags;
  int compass;
  
  
};

