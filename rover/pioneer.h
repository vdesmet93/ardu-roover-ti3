#pragma once

void initializeConnection();
void sendPacket(char command);
void sendPacket(char command, int argument);
void sendPacket(char command, char* argument, int size);
void enableSonar();
void sendSonarSpeed(int interval);
int writeSerial(unsigned char* buf, int length);
int checkMessage(unsigned char receivedBytes[], int count);
void receiveData();
void shiftBytes(int startPosOfPacket);
void parseMessage();
struct SipMessage convertToSipMessage(unsigned char receivedBytes[]);

struct SipMessage {
  /** Motor status
   * 0x32 -> Motors stopped
   * 0x33 -> Robot moving
   */
  unsigned char motorStatus;

  /** Wheel-encoder integrated coordinates in milliemeters */
  int xPos;
  int yPos;

  /** Orientation in degrees */
  int thPos;

  /* Wheel velocities
   * in millimeters per second
   */
  int lVel;
  int rVel;
  
  /** Average velocity of both wheels **/
  int vel;

  /** Battery charge
   * In tenths of volts 
   * 101 -> 10.1 volts
   */
  int batteryLevel;

  /** Motor stall and bumper indicators
   * Bit 0 is left wheel stall indicator, set to 1 if stalled
   * Bits 1-7 are the first bumper input states
   * Bit 8 is the right wheel stall
   * Bits 9-15 are the second bumper input states
   */
  int stallAndBumper;

  /** Setpoint of the servo's angular position in degrees */
  int control;

  /** Flags
   * Bit 0 is the motor status
   * Bits 1-4 is the sonar array status
   * Bits 5-6 is STOP
   * bits 7-8 ledge-sense IRs
   * bit 9 is joystick fire button
   * bit 10 is auto-charger power-good
   */
  int flags;

  /** Compass heading in 2-degree units */
  int compass;

  /** Array of sonar readings
   * Values are distance in millimeters 
   * Old Sonar are the previous sonar values
   */
  int* sonar;
  int* oldSonar;

  /** Gripper state byte */
  int gripState;

  /** Selected analog port number 
   * Value is between 1-5
   */
  int anPort;

  /** User analog input reading on selected port
   * 0-255=0-5 VDC
   */
  int analog;

  /** Byte-encoded user digital I/O */
  int digIn;
  int digOut;

  /** Actual battery voltage 
   * in 0.1V
   * 101 -> 10.1 Volts
   * Useful for battery voltages > 25.5
   */
  int batteryX10;

  /** Automated recharging state byte
   * -1 is unknown
   * 0 is not charging
   * 1 is bulk charging
   * 2 is overcharging
   * 3 is float 
   */
  int chargeState;
};

extern struct SipMessage lastMessage;

