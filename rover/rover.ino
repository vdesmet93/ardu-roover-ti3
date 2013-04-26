#include <SoftwareSerial.h>
#include "commands.h"
#include "pioneer.h"
#include "defines.h"

int number, n, fd, anglePulse, directionPulse, killPulse;
int current = 0;
boolean backward = false;
boolean halt = false;
unsigned int spd;

void setup() 
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
  Serial.begin(BAUD_RATE);
  initializeConnection();
  
  sendPacket(SONAR, 0);
  delay(PACKET_DELAY);
  sendPacket(BUMPSTALL, 0);
  delay(PACKET_DELAY);
  sendPacket(ENABLE, 1);
  delay(PACKET_DELAY);
}

void checkAngle()
{
  anglePulse = pulseIn(JOYSTICK_RIGHT_X, HIGH);
  if(anglePulse > ANGLE_THRESHOLD)
  {
    int angle = 100 - ((anglePulse - ANGLE_THRESHOLD) / 8);
    angle -= 50;
    if(-DEAD_ZONE < angle && DEAD_ZONE > angle)
    {
      angle = 0;
    }
    sendPacket(ROTATE, angle * ANGLE_MULTIPLIER);
  }
}

void checkDirection()
{
  directionPulse = pulseIn(DIRECTION, HIGH);
  if(directionPulse > FORWARD_THRESHOLD && directionPulse < HALT_THRESHOLD)
  {
    halt = false;
    backward = false;
  }
  else if(directionPulse > HALT_THRESHOLD && directionPulse < REVERSE_THRESHOLD)
  {
    halt = true;
    backward = false;
  }
  else if(directionPulse > REVERSE_THRESHOLD)
  {
    backward = true;
    halt = false;
  }
}

void checkSpeed()
{
  spd = pulseIn(JOYSTICK_LEFT_Y, HIGH);
  if(spd > SPEED_THRESHOLD)
  {
    spd = spd - (SPEED_THRESHOLD + SPEED_COMPENSATOR);
    if(spd > 0 && spd < MAX_SPEED)
    {
      spd *= SPEED_MULTIPLIER;
      //if(halt)
      //  spd = 0;
      if(backward)
        sendPacket(VEL, -spd);
      else
        sendPacket(VEL, spd);
    }
    else
      sendPacket(VEL, 0);
  }
  else
    sendPacket(PULSE);
}

void checkKillswitch()
{
  killPulse = pulseIn(KILL_SWITCH, HIGH);
  if(killPulse > KILLSWITCH_THRESHOLD)
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


