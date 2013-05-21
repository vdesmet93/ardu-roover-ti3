#include "commands.h"
#include "pioneer.h"
#include "defines.h"

int number, n, fd, anglePulse, directionPulse, killPulse;
int current = 0;
int currentSpeed = 0;
bool isForcedOverride = false;

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
  Serial1.begin(PIONEER_BAUD_RATE);
  initializeConnection();
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


void checkSpeed()
{
  int spd = pulseIn(JOYSTICK_LEFT_Y, HIGH);
  if(spd > SPEED_THRESHOLD)
  {
    spd = spd - (SPEED_THRESHOLD + SPEED_COMPENSATOR);
    if(spd > 0 && spd < MAX_SPEED)
    {
      spd *= SPEED_MULTIPLIER;
      currentSpeed = spd;

      sendPacket(VEL, spd);
    }
    else {
      sendPacket(VEL, 0);
      currentSpeed = 0;
    }
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

bool checkEmergencyFront() 
{
  bool isDangerous = false;

  if(lastMessage.sonar[3] < SONAR_KILLSWITCH_THRESHOLD)
    isDangerous = true;
  else if(lastMessage.sonar[4] < SONAR_KILLSWITCH_THRESHOLD)
    isDangerous = true;
  else {
    int threshold;
    
    if(isForcedOverride) 
      threshold = SONAR_SIDE_KS_THRESHOLD;
    else 
      threshold = SONAR_SIDE_KILLSWITCH_THRESHOLD;
      
    
  }
  else if(isForcedOverride) {
    if(lastMessage.sonar[1] < SONAR_SIDE_KS_THRESHOLD)
      isDangerous = true;
    if(lastMessage.sonar[2] < SONAR_SIDE_KS_THRESHOLD)
      isDangerous = true;
    else if(lastMessage.sonar[5] < SONAR_SIDE_KS_THRESHOLD)
      isDangerous = true;
    else if(lastMessage.sonar[6] < SONAR_SIDE_KS_THRESHOLD)
      isDangerous = true;
  }
  else { // !isForcedOverride
  
  }
  else if((lastMessage.sonar[2] + lastMessage.sonar[5]) / 2 < SONAR_SIDE_KS_THRESHOLD && isForcedOverride)
    isDangerous = true;
  else if((lastMessage.sonar[2] + lastMessage.sonar[5]) / 2 <  SONAR_KILLSWITCH_THRESHOLD && !isForcedOverride)
    isDangerous = true;
  else if((lastMessage.sonar[1] + lastMessage.sonar[6]) / 2 < SONAR_SIDE_KS_THRESHOLD && isForcedOverride)
    isDangerous = true;
  else if((lastMessage.sonar[1] + lastMessage.sonar[6]) / 2 < SONAR_KILLSWITCH_THRESHOLD && !isForcedOverride)
    isDangerous = true;

  if(isDangerous) 
  {
    // Danger ahead

    if(currentSpeed > 0 ) 
    {
      sendPacket(E_STOP);
      delay(KILL_DELAY);

      currentSpeed = 0;
    } 
    else 
    { 
      // currentSpeed == 0

      // Check left side
      int leftSum = lastMessage.sonar[1] + lastMessage.sonar[2];
      int rightSum = lastMessage.sonar[5] + lastMessage.sonar[6];

      int angle = 180;
      if(rightSum > leftSum) {
        // Left side is safer, rotate to left
        angle *= -1;
      }
      sendPacket(ROTATE, angle);
    }
    return true;

  }

  return false;
}

bool checkFront() {
  for(int sonarPos = 2; sonarPos <= 5; sonarPos++) {
    if(lastMessage.sonar[sonarPos] < SONAR_DANGER_THRESHOLD) {
      // Danger ahead

      if(currentSpeed > 0 ) {
        // Check left side
        // high value is object nearby, low value is object far away
        int leftAvg = 2000 - (lastMessage.sonar[1] + lastMessage.sonar[2]) / 2;
        int rightAvg = 2000 - (lastMessage.sonar[5] + lastMessage.sonar[6]) / 2;

        int angle = 100;
        if(leftAvg < rightAvg) {
          // Left side is safer, rotate to left
          angle += (leftAvg / 100) * 2;
        } 
        else {
          // Rotate to right 
          angle += (rightAvg / 100) * 2;
          angle *= -1;
        }
        sendPacket(ROTATE, angle);

        return true;

      } 

    }
  }

  return false;
}

bool checkSonar() {

  // Emergency stop front, it's not safe
  if(checkEmergencyFront())
    return true;

  if(checkFront()) 
    return true;



  // Return false if everything is safe
  return false;
}

void loop()
{
  checkSpeed();
  checkAngle();
  checkKillswitch();

  receiveData();

  while(checkSonar()) {
    // Update data and check again
    receiveData();    

    // Check Killswitch
    checkKillswitch();

    isForcedOverride = true;
  }
  isForcedOverride = false;
}















