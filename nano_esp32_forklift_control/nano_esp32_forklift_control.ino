#include <HardwareSerial.h>
#include <ESP32Servo.h>

#define PWM_FREQ 5000      // PWM frequency in Hz
#define PWM_RES 8   // 8-bit resolution (0-255)
#define PWM_CHANNELS 8

Servo myservo;

int pos = 90;
int servoPin = 6 ;//can be any GPIO for esp32 nano
int motorForwardPin = 3;//digitalWrite will go to D3
int motorReversePin = 4;//digitalWrite will go to D4


int angle = 0;
int maxSpeed = 0;
char bluetoothData[16];

const byte numChars = 64;
char receivedChars [numChars];
char tempChars[numChars];

boolean newData = false;

int tiltFloat = 90;
int verticalMovementFloat = 0;
int forkliftVerticalMovement = 0;
int forkliftTilt = 90;


void setup() {
  //Serial1.begin(9600, SERIAL_8N1, D9, D8);
  Serial0.begin(9600);
  Serial.begin(9600);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 750, 2250); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep

  forkliftTilt=90;
  int pos = 90;

  analogWrite(motorReversePin, 0);
  analogWrite(motorForwardPin, 0);
}

void loop() {

  //sendToArduino();
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    Serial.print(tiltFloat);
    Serial.print("   ");
    Serial.print(verticalMovementFloat);
    Serial.print("   ");
    Serial.print(angle);
    Serial.print("   ");
    Serial.println(maxSpeed);
    forkliftMovement();
    newData = false;
  }
  /*
  analogWrite(motorForwardPin,255);
  delay(100);
  analogWrite(motorForwardPin,0);
  analogWrite(motorReversePin,255);
  delay(100);
  analogWrite(motorReversePin,0);
  //delay(100);
  */
  
  
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
 
  while (Serial0.available() > 0 && newData == false) {
      rc = Serial0.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = false;
              ndx = 0;
              newData = true;
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
  }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
    tiltFloat = atoi(strtokIndx);     // convert this part to a float
    //tiltFloat = tiltFloat*(90/512);

    strtokIndx = strtok(NULL, ",");
    verticalMovementFloat = atoi(strtokIndx);   // convert this part to a float
    //verticalMovementFloat = verticalMovementFloat*(255/512);

    strtokIndx = strtok(NULL, ",");
    angle = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    maxSpeed = atoi(strtokIndx);

}

void forkliftMovement()  {
  /*
  if(tiltFloat>400)
  {

    if(pos<180)
    {
      pos+=2;//adjust value after testing
    }
    myservo.write(pos);
    //delay(5);

  }
  
  if(tiltFloat<-400)
  {

    if(pos>0)
    {
      pos-=2;
    }
    myservo.write(pos);
    //delay(5);
  }
  
    //myservo.write(tiltFloat);
  //forkliftVerticalMovement = int(trunc(tiltFloat);
  
  */
  
  if(verticalMovementFloat>400)//needs to be changed for deadspace on joystick
  {
    analogWrite(motorForwardPin, 255);
    analogWrite(motorReversePin, 0);
    Serial.print("Forward");
  }
  if(verticalMovementFloat<-400)//needs to be changed for deadspace
  {
    analogWrite(motorReversePin, 255); 
    analogWrite(motorForwardPin, 0);
    Serial.print("Reverse");
  }
  if(verticalMovementFloat<400 & verticalMovementFloat>-400);
  {
    analogWrite(motorReversePin, 0);
    analogWrite(motorForwardPin, 0);
    Serial.print("no-where");
  }
}

void sendToArduino()
{
    sprintf(bluetoothData, "<%i,%i>",angle,maxSpeed);
    Serial1.write(bluetoothData);
}
