#include "commands.h"
#include "pioneer.h"
#include "defines.h"
int sonarArray[] = { 
  0, 0, 45, 80, 80, 45, 0, 0 };

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
    if(spd > 0 && spd < MAX_INTEGER)
    {
      if(spd > MAX_SPEED)
        spd = MAX_SPEED;

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
  int threshold = 200 + ( lastMessage.vel / 2);

  if(threshold > SONAR_KILLSWITCH_THRESHOLD)
    threshold = SONAR_KILLSWITCH_THRESHOLD;

  bool isDangerous = false;
  if(lastMessage.sonar[3] < threshold)
    isDangerous = true;
  else if(lastMessage.sonar[4] < threshold)
    isDangerous = true;
  else {

    if(isForcedOverride) {
      threshold = 200 + (lastMessage.vel / 3);
      if(threshold > SONAR_SIDE_KS_THRESHOLD)
        threshold = SONAR_SIDE_KS_THRESHOLD;
    }

    if(lastMessage.sonar[1] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[2] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[5] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[6] < threshold)
      isDangerous = true;
  }

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
      // Check left and right side.
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
  int dangerSonarPos = 0;
  int dangerSonarValue = 0;

  int sonarDangerThreshold = 200 + (lastMessage.vel * 2);
  if(sonarDangerThreshold > SONAR_DANGER_THRESHOLD)
    sonarDangerThreshold = SONAR_DANGER_THRESHOLD;

  for(int sonarPos = 2; sonarPos <= 5; sonarPos++) {
    if(lastMessage.sonar[sonarPos] < sonarDangerThreshold &&
        lastMessage.sonar[sonarPos] < lastMessage.oldSonar[sonarPos]) {
      // Danger ahead

        if(dangerSonarValue > lastMessage.sonar[sonarPos] || dangerSonarValue == 0) 
      {
        // This sonar position is more in danger than the other ones 
        dangerSonarPos = sonarPos;
        dangerSonarValue = lastMessage.sonar[sonarPos];
      }
    }
  }
  if(currentSpeed > 0 && dangerSonarValue > 0) {
    int sonarCorner = sonarArray[dangerSonarPos];
    int pioneerSpeed = lastMessage.vel;

    int distance = dangerSonarValue - SONAR_KILLSWITCH_THRESHOLD;

    float seconds = ((float)distance) / ((float)pioneerSpeed);

    float degreesPerSecond = dangerSonarValue * seconds;

    int maxRotation = sonarCorner * 3;
    if(maxRotation > MAX_ROTATION)
      maxRotation = MAX_ROTATION;
      
    if(degreesPerSecond > maxRotation)
      degreesPerSecond = maxRotation;

    if(dangerSonarPos < 4) {
      // Rotate to right
      degreesPerSecond *= -1;
    } 
    sendPacket(ROTATE, (int)degreesPerSecond);
    return true;

  }

  return false;
}

bool checkSonar() 
{
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
//    checkKillswitch();
    isForcedOverride = true;
  }
  if(isForcedOverride) 
  {
    isForcedOverride = false;
  }
}


















