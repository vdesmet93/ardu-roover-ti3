#include <Arduino.h>
#include "controller.h"
#include "defines.h"


void initializeController()
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
}

int getControllerAngle()
{
  int anglePulse = pulseIn(JOYSTICK_RIGHT_X, HIGH);
  if(anglePulse > ANGLE_THRESHOLD)
  {
    int angle = 100 - ((anglePulse - ANGLE_THRESHOLD) / 8);
    angle -= 50;
    if(-DEAD_ZONE < angle && DEAD_ZONE > angle)
    {
      angle = 0;
    } 
    return angle;
  }
  return -1;
}
int getControllerDirection()
{

  int directionPulse = pulseIn(DIRECTION, HIGH);
  if(directionPulse > FORWARD_THRESHOLD && directionPulse < HALT_THRESHOLD)
  {
    return CONTROLLER_DIRECTION_FORWARD;
  }
  else if(directionPulse >= HALT_THRESHOLD && directionPulse < REVERSE_THRESHOLD)
  {
    return CONTROLLER_DIRECTION_HALT;
  }
  else if(directionPulse >= REVERSE_THRESHOLD)
  {
    return CONTROLLER_DIRECTION_REVERSE;
  }
}
int getControllerSpeed()
{
  int spd = pulseIn(JOYSTICK_LEFT_Y, HIGH);
  if(spd > SPEED_THRESHOLD)
  {
    spd = spd - (SPEED_THRESHOLD + SPEED_COMPENSATOR);
    spd *= SPEED_MULTIPLIER;

    if(spd > MAX_SPEED)
      return MAX_SPEED;
    else if(spd >= 0)
      return spd;
    else 
      return 0;
  }
  else
    return -1;

}

bool isKillSwitch()
{
  int killPulse = pulseIn(KILL_SWITCH, HIGH);
  return killPulse > KILLSWITCH_THRESHOLD;

}




