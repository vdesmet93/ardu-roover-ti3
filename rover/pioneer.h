#pragma once

void initializeConnection();
void sendPacket(char command);
void sendPacket(char command, int argument);
void sendPacket(char command, char* argument, int size);
int writeSerial(unsigned char* buf, int length);
void readFromRover();