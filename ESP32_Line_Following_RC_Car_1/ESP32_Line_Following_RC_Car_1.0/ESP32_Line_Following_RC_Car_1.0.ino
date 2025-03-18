/////////////////////////////////////////////////////////////////////////////////
////////////////   ESP32 Line Following RC Car  /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////   Group 10   //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


////////////// Librarys /////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Bluepad32.h>
#include <HardwareSerial.h>

//////////////// Motor pin definitions //////////////////////////////////////////

#define M1_1 7
#define M1_2 6
#define M2_1 5
#define M2_2 4
#define M3_1 37
#define M3_2 38
#define M4_1 16
#define M4_2 15 

int motorPins[] = {M1_1, M1_2, M2_1, M2_2, M3_1, M3_2, M4_1, M4_2};

// Motor channel definitions
#define M1_1_CHANNEL 0
#define M1_2_CHANNEL 1
#define M2_1_CHANNEL 2
#define M2_2_CHANNEL 3
#define M3_1_CHANNEL 4
#define M3_2_CHANNEL 5
#define M4_1_CHANNEL 6
#define M4_2_CHANNEL 7 

int motorChannels[] = {M1_1_CHANNEL, M1_2_CHANNEL, M2_1_CHANNEL, M2_2_CHANNEL, M3_1_CHANNEL, M3_2_CHANNEL, M4_1_CHANNEL, M4_2_CHANNEL};

//////////////////////////////////////////////////////////////////////////////

//////////////////// Sensor Pins ////////////////////////////////////////////

const int irPins[10] = {9, 10, 11, 12, 13, 17, 3, 1, 2, 20};  // IR sensor pins

/////////////////////////////////////////////////////////////////////////////

/////////////////// Variables ////////////////////////////////////////////////

// LEDC configuration
#define PWM_FREQ 5000      // PWM frequency in Hz
#define PWM_RES 8   // 8-bit resolution (0-255)
#define PWM_CHANNELS 8

const String forward = "forward";
const String reverse = "reverse";

ControllerPtr myControllers; // Initialize to nullptr

// Line Following Motor Speed
int motorSpeed = 80 * 2.55;

// Remote Control Motor Speed
int baseSpeed = 150;    // Default speed is 150/255 

// Remote Control PWM Variables
float PWM1 = 0;
float PWM2 = 0;

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

int sensorRawValues[10];  // Sensor values (analog readings)
int sensorWeights[10] = {-4, -3, -2, -1, 0, 0, 1, 2, 3, 4};

// PID Controls
#define Kp 22.5 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

int proportional = 0;
int error = 0;

bool RCMode = true;

//////////////////////////////////////////////////////////////////////////

///////////////// Serial Connectivity  ///////////////////////////////////

#define RXD1 48//Receiving data pin
#define TXD1 47//Transmitting data pin
char serialData[128];//Character array to send to Serial1 port

///////////////// Bluetooth Connectivity /////////////////////////////////

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) 
{
    myControllers = ctl; // Store the connected controller
}

void onDisconnectedController(ControllerPtr ctl) 
{
    myControllers = nullptr; // Clear the controller reference
}

///////////////////////////////////////////////////////////////////////////////

///////////////// Individual Motor Functions //////////////////////////////////

// Stop all motors
void stopAllMotors() 
{
    ledcWrite(M1_1_CHANNEL, 0);
    ledcWrite(M1_2_CHANNEL, 0);
    ledcWrite(M2_1_CHANNEL, 0);
    ledcWrite(M2_2_CHANNEL, 0);
    ledcWrite(M3_1_CHANNEL, 0);
    ledcWrite(M3_2_CHANNEL, 0);
    ledcWrite(M4_1_CHANNEL, 0);
    ledcWrite(M4_2_CHANNEL, 0);
}

// Line Following Movement Functions with throttle control

void LFFRMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      ledcWrite(M1_2_CHANNEL, 0);
      ledcWrite(M1_1_CHANNEL, speed);
      break;

    case 'r':  
      ledcWrite(M1_1_CHANNEL, 0);
      ledcWrite(M1_2_CHANNEL, speed);
      break;

    default:
      ledcWrite(M1_1_CHANNEL, 0);
      ledcWrite(M1_2_CHANNEL, 0);
      break;
  }
}

void LFFLMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      ledcWrite(M2_1_CHANNEL, 0);
      ledcWrite(M2_2_CHANNEL, speed);
      break;

    case 'r':  
      ledcWrite(M2_2_CHANNEL, 0);
      ledcWrite(M2_1_CHANNEL, speed);
      break;

    default:
      ledcWrite(M2_1_CHANNEL, 0);
      ledcWrite(M2_2_CHANNEL, 0);
      break;
  }
}

void LFBRMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      ledcWrite(M3_1_CHANNEL, 0);
      ledcWrite(M3_2_CHANNEL, speed);
      break;

    case 'r':  
      ledcWrite(M3_2_CHANNEL, 0);
      ledcWrite(M3_1_CHANNEL, speed);
      break;

    default:
      ledcWrite(M3_1_CHANNEL, 0);
      ledcWrite(M3_2_CHANNEL, 0);
      break;
  }
}

void LFBLMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      ledcWrite(M4_2_CHANNEL, 0);
      ledcWrite(M4_1_CHANNEL, speed);
      break;

    case 'r':  
      ledcWrite(M4_1_CHANNEL, 0);
      ledcWrite(M4_2_CHANNEL, speed);
      break;

    default:
      ledcWrite(M4_1_CHANNEL, 0);
      ledcWrite(M4_2_CHANNEL, 0);
      break;
  }
}

//////////////////////////////////////////////////////////////////////////

// RC Movement Functions with throttle control 

void RCFRMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    ledcWrite(M1_1_CHANNEL, 0);
    ledcWrite(M1_2_CHANNEL, pwm);
  }
  else if (pwm >= 0)
  { // forward
    ledcWrite(M1_2_CHANNEL, 0);
    ledcWrite(M1_1_CHANNEL, pwm);
  }
  else 
  {
    ledcWrite(M1_1_CHANNEL, 0);
    ledcWrite(M1_2_CHANNEL, 0);
  }
}

void RCFLMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    ledcWrite(M2_2_CHANNEL, 0);
    ledcWrite(M2_1_CHANNEL, pwm);
  }
  else if (pwm >= 0)
  { // forward
    ledcWrite(M2_1_CHANNEL, 0);
    ledcWrite(M2_2_CHANNEL, pwm);
  }
  else 
  {
    ledcWrite(M2_1_CHANNEL, 0);
    ledcWrite(M2_2_CHANNEL, 0);;
  }
}

void RCBRMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    ledcWrite(M3_2_CHANNEL, 0);
    ledcWrite(M3_1_CHANNEL, pwm);
  }
  else if (pwm >= 0)
  { // forward
    ledcWrite(M3_1_CHANNEL, 0);
    ledcWrite(M3_2_CHANNEL, pwm);
  }
  else 
  {
    ledcWrite(M3_1_CHANNEL, 0);
    ledcWrite(M3_2_CHANNEL, 0);
  }
}

void RCBLMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    ledcWrite(M4_1_CHANNEL, 0);
    ledcWrite(M4_2_CHANNEL, pwm);
  }
  else if (pwm >= 0)
  { // forward
    ledcWrite(M4_2_CHANNEL, 0);
    ledcWrite(M4_1_CHANNEL, pwm);
  }
  else 
  {
    ledcWrite(M4_1_CHANNEL, 0);
    ledcWrite(M4_2_CHANNEL, 0);
  }
}

/////////////////////////////////////////////////////////////////////////////////////

///////////// Calculate Motor Angle  ////////////////////////////////////////////////

float calculateAngle(int16_t lx, int16_t ly) 
{
  int resultMagnitude = sqrt((lx * lx) + (ly * ly));
  if (resultMagnitude > 480)
  {
    float angle = atan2(ly, lx);
    angle = (angle * 180 / M_PI) + 90;
    if (angle < 0) angle += 360;
    
    return angle;
  }
  else
  {
    return -1;
  }
}

////////////////////////////////////////////////////////////////////////

///////////////// IR P Controller ///////////////////////////////////////

int IR_PController() 
{
  proportional = Kp * error; 
  int controlSignal = proportional; 

  if (controlSignal < 0)
  {
    controlSignal += 360;
  }

  return controlSignal;
}

//////////////////////////////////////////////////////////////////////

///////////////// Motor Mathematics ////////////////////////////////////////////////////

// Define the motor function
float motor_pwm(float theta, float maxSpeed, bool diagonal2) 
{
  if (diagonal2)
  {
    theta = 360 - theta;
  }

  if (theta >= 0 && theta <= 90) { return maxSpeed * (1 - (theta / 45)); }
  else if (theta > 90 && theta <= 180) { return -1 * maxSpeed; }
  else if (theta > 180 && theta <= 270) { return maxSpeed * ((theta / 45) - 5); }
  else if  (theta > 270 && theta <= 360) { return maxSpeed; }
  else return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////

///////////// LF Movement Algoithm //////////////////////////////////

// Function to control motors based on Infra-Red sensors
void LFmoveCar(float angle)
{
    // 0 to 45 Degrees
    if (angle >= 0 && angle <= 45)
    {
        int angularMotorSpeed = map(angle, 0, 45, motorSpeed, 0);
        LFFRMotor(forward, angularMotorSpeed);
        LFFLMotor(forward, motorSpeed);
        LFBRMotor(forward, motorSpeed);
        LFBLMotor(forward, angularMotorSpeed);
    }
    // 45 to 90 Degrees
    else if (angle >= 45 && angle <= 90) 
    {
        int angularMotorSpeed = map(angle, 45, 90, 0, motorSpeed);
        LFFRMotor(reverse, angularMotorSpeed);
        LFFLMotor(forward, motorSpeed);
        LFBRMotor(forward, motorSpeed);
        LFBLMotor(reverse, angularMotorSpeed);
    }
    // 90 to 135 Degrees
    else if (angle >= 90 && angle <= 135) 
    {
        int angularMotorSpeed = map(angle, 90, 135, motorSpeed, 0);
        LFFRMotor(reverse, motorSpeed);
        LFFLMotor(forward, angularMotorSpeed);
        LFBRMotor(forward, angularMotorSpeed);
        LFBLMotor(reverse, motorSpeed);
    }
    // 135 to 180 Degrees
    else if (angle >= 135 && angle <= 180) 
    {
        int angularMotorSpeed = map(angle, 135, 180, 0, motorSpeed);
        LFFRMotor(reverse, motorSpeed);
        LFFLMotor(reverse, angularMotorSpeed);
        LFBRMotor(reverse, angularMotorSpeed);
        LFBLMotor(reverse, motorSpeed);
    }
    // 180 to 225 Degrees
    else if (angle >= 180 && angle <= 225)
    {
        int angularMotorSpeed = map(angle, 180, 225, motorSpeed, 0);
        LFFRMotor(reverse, angularMotorSpeed);
        LFFLMotor(reverse, motorSpeed);
        LFBRMotor(reverse, motorSpeed);
        LFBLMotor(reverse, angularMotorSpeed);
    }
    // 225 to 270 Degrees
    else if (angle >= 225 && angle <= 270)
    {
        int angularMotorSpeed = map(angle, 225, 270, 0, motorSpeed);
        LFFRMotor(forward, angularMotorSpeed);
        LFFLMotor(reverse, motorSpeed);
        LFBRMotor(reverse, motorSpeed);
        LFBLMotor(forward, angularMotorSpeed);
    }
    // 270 to 315 Degrees
    else if (angle >= 270 && angle <= 315)
    {
        int angularMotorSpeed = map(angle, 270, 315, motorSpeed, 0);
        LFFRMotor(forward, motorSpeed);
        LFFLMotor(reverse, angularMotorSpeed);
        LFBRMotor(reverse, angularMotorSpeed);
        LFBLMotor(forward, motorSpeed);
    }
    // 315 to 360 Degrees
    else if (angle >= 315 && angle <= 360)
    {
        int angularMotorSpeed = map(angle, 315, 360, 0, motorSpeed);
        LFFRMotor(forward, motorSpeed);
        LFFLMotor(forward, angularMotorSpeed);
        LFBRMotor(forward, angularMotorSpeed);
        LFBLMotor(forward, motorSpeed);
    }
  
    // Stop Motors if there is an error
    else 
    {
        stopAllMotors();
    }
}

/////////////////////////////////////////////////////////////////////////////////

//////////////// RC Movement Function ///////////////////////////////////////////

void RCmoveCar(float angle, int button, int motorSpeed)
{ 
  // Move Car Omni with throttle control
  if (angle >= 0)
  {
    RCFRMotor(PWM1);
    RCFLMotor(PWM2);
    RCBRMotor(PWM2);
    RCBLMotor(PWM1);
  }

  // Clockwise Rotation
  else if (button & 0x0020)
  {
    RCFRMotor(motorSpeed * -1);
    RCFLMotor(motorSpeed);
    RCBRMotor(motorSpeed * -1);
    RCBLMotor(motorSpeed);
  }
  // Anti-Clockwise Rotation
  else if (button & 0x0010)
  {
    RCFRMotor(motorSpeed);
    RCFLMotor(motorSpeed * -1);
    RCBRMotor(motorSpeed);
    RCBLMotor(motorSpeed * -1);
   }
   // Stop Motors if Joystick is Centered
   else 
   {
     stopAllMotors();
   }
}

///////////////////////////////////////////////////////////////////////////////////

////////////////////// Setup /////////////////////////////////////////////////////

// Setup function
void setup() 
{
    Serial.begin(115200);
    Serial1.begin(9600,SERIAL_8N1, RXD1, TXD1);
    //Serial.println("Starting Bluepad32...");

    // Motor Setup
    for (int i = 0; i < PWM_CHANNELS; i++)
    {
      ledcSetup(i, PWM_FREQ, PWM_RES); // Configure PWM
      ledcAttachPin(motorPins[i], motorChannels[i]); // Attach pin to channel
    }

    for (int i = 0; i < 10; i++) 
    {
      pinMode(irPins[i], INPUT);  // Set IR sensor pins as input
    }
   
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController); 

    
    
}

/////////////////////////////////////////////////////////////////////////

/////////////////////Remote Controll Main code //////////////////////////
void RCloop()
{

    // Read left stick for directional movement
        int16_t lx = myControllers->axisX(); // Left stick X-axis
        int16_t ly = myControllers->axisY(); // Left stick Y-axis

        int16_t rx = myControllers->axisRX();// Right stick X-axis
        int16_t ry = myControllers->axisRY();// Right stick Y-axis

        // Read left trigger for throttle control
        int16_t L2 = myControllers->throttle(); // Left trigger

        int button = myControllers->buttons();

        float angle = calculateAngle(lx, ly);

        // Map the left trigger value (L2) to an additional speed (0 to 205)
        int additionalSpeed = map(L2, 0, 1023, 0, 105);

        // Combine base speed with additional speed
        int maxSpeed = baseSpeed + additionalSpeed;

        PWM1 = motor_pwm(angle, maxSpeed, 0);
        PWM2 = motor_pwm(angle, maxSpeed, 1);

        /*
        //DEBUG
        Serial.print(angle);
        Serial.print("  ");
        Serial.print(PWM1);
        Serial.print("  ");
        Serial.println(PWM2); */
        

        // Move the car
        RCmoveCar(angle, button, maxSpeed);

        //Sending right joystick values, angle and maxSpeed to serial monitor with formatting
        /*Serial.print("<");
        Serial.print(ry);
        Serial.print(",");
        Serial.print(rx);
        //Serial.print(",");
        //Serial.print(angle);
        //Serial.print(",");
        //Serial.print(maxSpeed);
        Serial.println(">");
        */
        sprintf(serialData,"<%i,%i,&i,%i>",ry,rx,angle,maxSpeed);
        Serial1.write(serialData);
        //Serial.write(serialData);

}

/////////////////////////////////////////////////////////////////////////

///////////////////// Line Following Main Code //////////////////////////

void LFloop()
{

    int minValue = 4096; // Initialize with the maximum possible analog value
        int minIndex;  // To store the index of the pin with the lowest value

        // Read sensor values and find the lowest
        for (int i = 0; i < 10; i++) 
        {
          sensorRawValues[i] = analogRead(irPins[i]);
          Serial.print(sensorRawValues[i]);
          Serial.print(" ");
          if (sensorRawValues[9] <= 3840)
          {
            minValue = sensorRawValues[9];
            minIndex = 9;
          }
          else if (sensorRawValues[i] <= (minValue - 0) && sensorRawValues[i] < 4000 )
          {
            minValue = sensorRawValues[i];
            minIndex = i;
          }
          if (minValue > 3950)         
          {
            minIndex = 100;
          }
        }

        //Serial.print(minValue);
        //Serial.print("  ");

        Serial.print(minIndex);
        Serial.print("  ");
      
      
        float controlSignal;
        

        if (minIndex > 0 && minIndex < 9)
        {
          error = sensorWeights[minIndex];
          controlSignal = IR_PController();
          LFmoveCar(controlSignal);
        }
        else if (minIndex == 0)
        {
          controlSignal = 260;
          LFmoveCar(controlSignal);
          delay(500);
        }
        else if (minIndex == 9)
        {
          controlSignal = 100;
          LFmoveCar(controlSignal);
          delay(200);
        }
        else 
        {
          error = 0;
          controlSignal = 0;
          LFmoveCar(controlSignal);
        }

        Serial.print(error);
        Serial.print("  ");

        

        Serial.println(controlSignal);

        // Move Car with throttle control
        //moveCar(controlSignal);

        delay(0);

}

///////////////////// Main Loop /////////////////////////////////////////
void loop() 
{
    BP32.update();
    if (myControllers && myControllers->isConnected()) 
    {
        int button = myControllers->buttons();

        if (button & 0x00000001)//changes car between line following and remote controll
        {
            RCMode = !RCMode;
            delay(500);
        }

        if (RCMode==true)
        {
            RCloop();
        }

        else
        {
            LFloop();
        }
    }
    
    else
    {
        stopAllMotors();
    }

}
