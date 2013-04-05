
#include <SoftwareSerial.h>
#include "commands.h"

#define RX 10
#define TX 11
#define JOYSTICK_RIGHT_X  A0
#define JOYSTICK_RIGHT_Y  A1
#define JOYSTICK_LEFT_Y  A2
#define JOYSTICK_LEFT_X  A3
#define SONAR_FREQUENCY  A3
#define EASTER_EGG  A5
#define KILL_SWITCH  4
#define MANUAL_AUTO  2

#define bit9600Delay 84  

SoftwareSerial mySerial(RX, TX); // RX, TX
int number,n, fd;

void readFromRover() 
{
  while(mySerial.available() > 0) {

    int i = mySerial.read();
    char str[256];
    sprintf(str, "%x ", i);
    Serial.println(str);
  }
}

/*
* Checksum generator for the buffer which will be send
 */
int getChecksum(unsigned char* buf) 
{
  int i; 
  unsigned char n; 
  int c = 0; 
  i = 3; 
  n = buf[2] - 2; 
  while (n > 1) 
  { 
    c += ((unsigned char)buf[i]<<8) | (unsigned char)buf[i+1]; 
    c = c & 0xffff; 
    n -= 2; 
    i += 2; 
  } 
  if (n > 0) 
    c = c ^ (int)((unsigned char) buf[i]); 
  return c;
}

int writeSerial(unsigned char* buf, int length) {
  for(int i = 0; i < length; i++)  
  {
    mySerial.write(buf[i]);
    // SWprint(buf[i]);
  }

  return length;
}

void SWprint(int data)
{
  byte mask;
  //startbit
  digitalWrite(TX,LOW);
  delayMicroseconds(bit9600Delay);
  for (mask = 0x01; mask>0; mask <<= 1) {
    if (data & mask){ // choose bit
      digitalWrite(TX,HIGH); // send 1
    }
    else{
      digitalWrite(TX,LOW); // send 0
    }
    delayMicroseconds(bit9600Delay);
  }
  //stop bit
  digitalWrite(TX, HIGH);
  delayMicroseconds(bit9600Delay);
}

/**
 * Send a packet without arguments.
 * For example, SYNC0, OPEN, RESET, CLOSE
 */
void sendPacket(char command) 
{
  // create array. Length = 6 because we don't have any arguments
  unsigned char buf[6];
  // header
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251
  // length
  buf[2] = 3; // command + 2 byte checksum
  // command
  buf[3] = command;
  // generate checksum
  int checksum = getChecksum(buf);
  // parse the checksum
  char high = (checksum & 0xFF00) >> 8;
  char low = checksum & 0x00FF;
  // add checksum to array, high byte is first
  buf[4] = high;
  buf[5] = low;
  // send the array, print resultcode
  int resultcode = writeSerial (buf, 6);

  //char str[256];
  //sprintf(str, "Resultcode: %d", resultcode);
 // Serial.println(str);
}

/*
* Send a packet with 1 argument: ENABLE, SETA, SONAR
 */
void sendPacket(char command, int argument) 
{
  // create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
  //														2 checksum
  unsigned char buf[9];
  /* ================= HEADER ================= */
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251

  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = 6; 

  /* ================= COMMAND ================= */
  buf[3] = command;

  /* ============== ARGUMENT TYPE ============== */
  if(argument >= 0)
    buf[4] = (unsigned char) ARG_POS_INT;
  else {
    argument *= -1;
    buf[4] = (unsigned char) ARG_NEG_INT;
  }

  /* ================= ARGUMENT ================= */
  // parse argument high and low
  char highArgument = ( (argument & 0xFF00) >> 8);
  char lowArgument = (argument & 0x00FF);
  

  // add argument to array. 
  // Low byte first(instead of high-first for checksum, because f*ck consistency)
  buf[5] = lowArgument;
  buf[6] = highArgument;

  /* ================= CHECKSUM ================= */
  // generate checksum
  int checksum = getChecksum(buf);

  // parse the checksum
  char highChecksum = (checksum & 0xFF00) >> 8;
  char lowChecksum = checksum & 0x00FF;

  // add checksum to array, high byte is first
  buf[7] = highChecksum;
  buf[8] = lowChecksum;

  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  int resultcode = writeSerial (buf, 9);

  //char str[256];
 // sprintf(str, "Resultcode: %d", resultcode);
  //Serial.println(str);
}
/*
* Send a packet with a string as argument
 */
void sendPacket(char command, char* argument, int size) {
  // create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
  //														2 checksum
  int bytes = size + 7;

  unsigned char buf[bytes];

  /* ================= HEADER ================= */
  buf[0] = 0xFA; // 250
  buf[1] = 0xFB; // 251

  /* =============== PACKET LENGTH =============== */
  // command + 2 byte checksum + 2 byte message
  buf[2] = 6; 

  /* ================= COMMAND ================= */
  buf[3] = command;

  /* ============== ARGUMENT TYPE ============== */
  buf[4] = (unsigned char) ARG_STRING;

  /* ================= ARGUMENT ================= */
  buf[5] = size;

  // parse argument high and low
  printf("parsing\n");
  char c = *argument;
  int idx = 6;
  while(c != '\0' && (idx - 6) != size) 
  {
    printf("P: %c\n", c);
    buf[idx] = c;
    argument++;
    idx++;
    c = *argument;
  }

  /* ================= CHECKSUM ================= */
  // generate checksum
  int checksum = getChecksum(buf);

  // parse the checksum
  char highChecksum = (checksum & 0xFF00) >> 8;
  char lowChecksum = checksum & 0x00FF;

  // add checksum to array, high byte is first
  buf[bytes-2] = highChecksum;
  buf[bytes-1] = lowChecksum;

  /* ================= TRANSFER ================= */
  // send the array, print resultcode
  int resultcode = writeSerial (buf, bytes);
//  printf("Resultcode: %d\n", resultcode);
}

void setup() 
{
  // define pin modes for tx, rx:
  pinMode(RX,INPUT);
  pinMode(TX, OUTPUT);
  pinMode(JOYSTICK_RIGHT_X, INPUT);
  pinMode(JOYSTICK_RIGHT_Y, INPUT);
  pinMode(JOYSTICK_LEFT_X, INPUT);
  pinMode(JOYSTICK_LEFT_X, INPUT);
  pinMode(SONAR_FREQUENCY, INPUT);
  pinMode(EASTER_EGG, INPUT);
  pinMode(KILL_SWITCH, INPUT);
  pinMode(MANUAL_AUTO, INPUT);
  digitalWrite(TX, HIGH);
  digitalWrite(RX, LOW);
  
  Serial.begin(9600);

  mySerial.begin(9600);
  delay(100);

  //pinMode(9, INPUT);
  sendPacket(SYNC0);
  delay(100);  

  sendPacket(SYNC1);
  delay(100);
  //readFromRover();
  sendPacket(SYNC2);
  delay(100);

  sendPacket(OPEN);
  delay(100);


 // sendPacket(SONAR, 0);
   sendPacket(BUMPSTALL, 0);
  delay(100);

  // enable the motors	
  sendPacket(ENABLE, 1);
  delay(100);    
}

int current = 0;

void loop()
{
    sendPacket(PULSE);

    // 1500 avg
    // 1100 min
    // 1900 max
    int x = pulseIn(JOYSTICK_RIGHT_X, HIGH);
    if(x > 1100) {
     // 0-100
     int percentage = (x - 1100) / 8;
     percentage = 100 - percentage;
     percentage -= 50;
    Serial.println(percentage);     
     sendPacket(ROTATE, percentage);
     delay(50);
    }
    
    
    
    unsigned int s = pulseIn(JOYSTICK_LEFT_Y, HIGH);
    
//     Serial.print("S: ");
 //   Serial.println(s);
    
    if(s > 1100) {
      s = s - 1120;
      if(s > 0 && s < 35000) {
       s *= 4;
       sendPacket(VEL, s);      
  //     Serial.println(s);
      }
      else {
        sendPacket(VEL, 0);
    //     Serial.println("Stop");   
      }
    }
    else {
      sendPacket(PULSE);
    }

    int killswitch = pulseIn(KILL_SWITCH, HIGH);
    //Serial.print("Kill: ");
   // Serial.println(killswitch);
    
    if(killswitch > 1100) {
      sendPacket(E_STOP);
      delay(10);
    }    
   
    delay(50);


  
  /**
  // rotate
  sendPacket(DHEAD, 180);
  delay(1000);
  sendPacket(PULSE);
  delay(1000);
  sendPacket(PULSE);
  delay(1000);

  // start driving
  sendPacket(VEL, 10000);
  delay(1000);
  while(1) 
  {
    readFromRover();

    sendPacket(PULSE);
    delay(100);
  }**/
}


