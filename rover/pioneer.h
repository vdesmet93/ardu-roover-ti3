#pragma once

void initializeConnection();
void sendPacket(char command);
void sendPacket(char command, int argument);
void sendPacket(char command, char* argument, int size);
void enableSonar();
void sendSonarSpeed(int interval);
int writeSerial(unsigned char* buf, int length);
int checkMessage(unsigned char receivedBytes[], int count);
void readFromRover();

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
  int* sonar;
  
  int gripState;
  int anPort;
  int analog;
  int digIn;
  int digOut;
  int batteryX10;
  int chargeState;
  int rotVel;
  int faultFlags;
};
