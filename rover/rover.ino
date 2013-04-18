#include <SoftwareSerial.h>
#include "commands.h"
#include "controller.h"
#include "pioneer.h"
#include "defines.h"

int controllerDirection = CONTROLLER_DIRECTION_FORWARD;

void setup() 
{
  // initialize serial connection with Pioneer
  initializeConnection();

  initializeController();
  Serial.begin(BAUD_RATE);
}

void checkAngle()
{
  int angle = getControllerAngle();
  if(angle != -1)
    sendPacket(ROTATE, angle * ANGLE_MULTIPLIER);

  Serial.print("angle: ");
  Serial.println(angle);
}

void checkDirection()
{
  controllerDirection = getControllerDirection();
  Serial.print("Direction: ");
  Serial.println(controllerDirection);
}

void checkSpeed()
{
  int spd = getControllerSpeed();

  if(spd != -1) {

    if(controllerDirection == CONTROLLER_DIRECTION_HALT)
      spd = 0;

    if(controllerDirection == CONTROLLER_DIRECTION_REVERSE)
      spd = -spd;

    sendPacket(VEL, spd);
  }
}

void checkKillswitch()
{
  if(isKillSwitch())
  {
    sendPacket(E_STOP);
    delay(KILL_DELAY);

  }
}

void loop()
{   
 sendPacket(PULSE);
 checkAngle();
 checkDirection();
 checkSpeed();
 checkKillswitch();
}






