#include "commands.h"
#include "pioneer.h"
#include "defines.h"

int  anglePulse, directionPulse, killPulse;
int currentSpeed = 0, currentRotation = 0;
bool isForcedOverride = false, isSpeedForcedOverride = false;

/**
 *Sets pins for APM input and starts the Serial bus
 */
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

/**
 * Checks rotation input from APM and forwards converted value to Pioneer
 */
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
    currentRotation = angle * ANGLE_MULTIPLIER;
    sendPacket(ROTATE, currentRotation);
  }
}

/**
 * Checks accelleration input from APM and forwards converted value to Pioneer
 */
void checkSpeed()
{
  int spd = pulseIn(JOYSTICK_LEFT_Y, HIGH);
  if(spd > SPEED_THRESHOLD)
  {  
    spd = spd - (SPEED_THRESHOLD + SPEED_COMPENSATOR);
    spd *= 2;
    if(spd > 0 && spd < MAX_INTEGER)
    {
      if(spd > MAX_SPEED)
        spd = MAX_SPEED;

      currentSpeed = spd;
      
      if(!isSpeedForcedOverride) 
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

/**
 * Checks for killswitch signal from controller and forwards to Pioneer
 */
void checkKillswitch()
{
  killPulse = pulseIn(KILL_SWITCH, HIGH);
  if(killPulse > KILLSWITCH_THRESHOLD)
  {
    sendPacket(E_STOP);
    delay(KILL_DELAY);
  }
}

/**
 * Checks if there is anything directly in front of the Pioneer through the sonar arrays. 
 * If there is something in the way, it triggers emergency brake
 */
 bool checkEmergencyFront() 
{
  int threshold = KILLSWITCH_THRESHOLD_BASE + ( lastMessage.vel / KILLSWITCH_SIDE_THRESHOLD_DIVIDER);

  if(threshold > SONAR_KILLSWITCH_THRESHOLD)
    threshold = SONAR_KILLSWITCH_THRESHOLD;

  bool isDangerous = false;
  if(lastMessage.sonar[SONAR_CENTER_LEFT] < threshold)
    isDangerous = true;
  else if(lastMessage.sonar[SONAR_CENTER_RIGHT] < threshold)
    isDangerous = true;
  else {

    if(isForcedOverride) {
      threshold = KILLSWITCH_THRESHOLD_BASE + (lastMessage.vel / KILLSWITCH_THRESHOLD_DIVIDER);
      if(threshold > SONAR_SIDE_KS_THRESHOLD)
        threshold = SONAR_SIDE_KS_THRESHOLD;
    }

    if(lastMessage.sonar[SONAR_SIDE_LEFT] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[SONAR_FRONT_LEFT] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[SONAR_FRONT_RIGHT] < threshold)
      isDangerous = true;
    else if(lastMessage.sonar[SONAR_SIDE_RIGHT] < threshold)
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
      int leftSum = lastMessage.sonar[SONAR_SIDE_LEFT] + lastMessage.sonar[SONAR_FRONT_LEFT];
      int rightSum = lastMessage.sonar[SONAR_FRONT_RIGHT] + lastMessage.sonar[SONAR_SIDE_RIGHT];

      int angle = SONAR_KS_ROTATION_BASE;
      if(rightSum > leftSum) {
        // Left side is safer, rotate to left
        angle *= -1;
      }
      currentRotation = angle;
      sendPacket(ROTATE, angle);
    }
    return true;
  }
  return false;
}

/** 
 * Checks if there is anything directly in front of the Pioneer through the sonar arrays. 
 * If there is something in the way, it calculates where it is using avarages from the left and right sides. 
 * After that, it steers to the side with the lowest value
 */
bool checkFront() {
  int dangerSonarPos = 0;
  int dangerSonarValue = 0;

  int sonarDangerThreshold = SONAR_DANGER_BASE + (lastMessage.vel * SONAR_DANGER_MULTIPLIER);
  if(sonarDangerThreshold > SONAR_DANGER_THRESHOLD)
    sonarDangerThreshold = SONAR_DANGER_THRESHOLD;

  for(int sonarPos = SONAR_FRONT_LEFT; sonarPos <= SONAR_FRONT_RIGHT; sonarPos++) {
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

    int sonarCorner = SONAR_ARRAY[dangerSonarPos];
    int pioneerSpeed = lastMessage.vel;

    int distance = dangerSonarValue - SONAR_KILLSWITCH_THRESHOLD - SAFE_ZONE;

    if(distance > 0) {
      float seconds = ((float)distance) / ((float)pioneerSpeed);

      float degreesPerSecond = SONAR_KS_ROTATION_BASE + (dangerSonarValue * seconds);

      int maxRotation = sonarCorner * SONAR_ROTATION_MULTIPLIER;
      if(maxRotation > MAX_ROTATION)
        maxRotation = MAX_ROTATION;

      if(degreesPerSecond > maxRotation) {
        degreesPerSecond = maxRotation;
      }

      if(dangerSonarPos < 4) {
        // Rotate to right
        degreesPerSecond *= -1;
      } 
      sendPacket(ROTATE, (int)degreesPerSecond);

    } 
    else if(currentRotation == 0){
      int angle = SONAR_KS_ROTATION_BASE;

      currentRotation = angle;

      if(dangerSonarPos < 4) {
        // Rotate to right
        angle *= -1;
      } 
      sendPacket(ROTATE, angle);

    }
    return true;  

  }

  return false;
}

/**
 * Checks if there is anything in front of the Pioneer. If there is something in front of it, it lowers its speed so it can make tighter turns.
 */
int checkFrontSpeed()
{
  if(currentSpeed > 0) {
    int dangerSonarPos = 0;
    int dangerSonarValue = 0;

    int sonarDangerThreshold = SONAR_REDUCE_SPD_BASE + (lastMessage.vel * SONAR_REDUCE_SPD_MULTIPLIER);
    if(sonarDangerThreshold > SONAR_REDUCE_SPD_MAX)
      sonarDangerThreshold = SONAR_REDUCE_SPD_MAX;

    for(int sonarPos = SONAR_FRONT_LEFT; sonarPos <= SONAR_FRONT_RIGHT; sonarPos++) {
      if(lastMessage.sonar[sonarPos] < sonarDangerThreshold &&
        lastMessage.sonar[sonarPos] < lastMessage.oldSonar[sonarPos]) {
        // Reduce speed

        sendPacket(VEL, currentSpeed * 0.5);
        isSpeedForcedOverride = true;
        return 1; 
      }
    }
  }

  isSpeedForcedOverride = false;


  return 0;
}

/**
 * General function to trigger checks for sonar
 */
bool checkSonar() 
{
  // Emergency stop front, it's not safe
  if(checkEmergencyFront())
    return true;
  if(checkFront()) 
    return true;

  // Reduce speed when something is near
  checkFrontSpeed();

  // Return false if everything is safe
  return false;
}

/**
 * Loops all the time, checks input from APM, check killswitch control, read package from Pioneer, check sonar.
 */
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
    isSpeedForcedOverride = false;
    currentRotation = 0;
  }
}






























