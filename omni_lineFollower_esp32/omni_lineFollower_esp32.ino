/////////////////////////////////////////////////////////////////////////////////
////////////////   ESP32 VERSION LINE FOLLOWER  /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////   JAMIE SMITH   ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////////////   Queen's University Belfast   //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////// Librarys ////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Bluepad32.h>

//////////////////////////////////////////////////////////
///////////////////////

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

// Motor Speed
int motorSpeed = 60 * 2.55;

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

int sensorRawValues[10];  // Sensor values (analog readings)
int sensorWeights[10] = {-4, -3, -2, -1, 0, 0, 1, 2, 3, 4};

// PID Controls
#define Kp 25 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

int proportional = 0;
float error = 0;

//////////////////////////////////////////////////////////////////////////

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

// Movement Functions with throttle control

void FRMotor(String movement, int speed)
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

void FLMotor(String movement, int speed)
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

void BRMotor(String movement, int speed)
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

void BLMotor(String movement, int speed)
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

///////////// Calculate Motor Angle from Controller ///////////////////////

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
  float controlSignal = proportional; 

  if (controlSignal < 0)
  {
    controlSignal += 360;
  }

  return controlSignal;
}

//////////////////////////////////////////////////////////////////////

///////////// Main Movement Algoithm //////////////////////////////////

// Function to control motors based on joystick input
void moveCar(float angle)
{
    // 0 to 45 Degrees
    if (angle >= 0 && angle <= 45)
    {
        int angularMotorSpeed = map(angle, 0, 45, motorSpeed, 0);
        FRMotor(forward, angularMotorSpeed);
        FLMotor(forward, motorSpeed);
        BRMotor(forward, motorSpeed);
        BLMotor(forward, angularMotorSpeed);
    }
    // 45 to 90 Degrees
    else if (angle >= 45 && angle <= 90) 
    {
        int angularMotorSpeed = map(angle, 45, 90, 0, motorSpeed);
        FRMotor(reverse, angularMotorSpeed);
        FLMotor(forward, motorSpeed);
        BRMotor(forward, motorSpeed);
        BLMotor(reverse, angularMotorSpeed);
    }
    // 90 to 135 Degrees
    else if (angle >= 90 && angle <= 135) 
    {
        int angularMotorSpeed = map(angle, 90, 135, motorSpeed, 0);
        FRMotor(reverse, motorSpeed);
        FLMotor(forward, angularMotorSpeed);
        BRMotor(forward, angularMotorSpeed);
        BLMotor(reverse, motorSpeed);
    }
    // 135 to 180 Degrees
    else if (angle >= 135 && angle <= 180) 
    {
        int angularMotorSpeed = map(angle, 135, 180, 0, motorSpeed);
        FRMotor(reverse, motorSpeed);
        FLMotor(reverse, angularMotorSpeed);
        BRMotor(reverse, angularMotorSpeed);
        BLMotor(reverse, motorSpeed);
    }
    // 180 to 225 Degrees
    else if (angle >= 180 && angle <= 225)
    {
        int angularMotorSpeed = map(angle, 180, 225, motorSpeed, 0);
        FRMotor(reverse, angularMotorSpeed);
        FLMotor(reverse, motorSpeed);
        BRMotor(reverse, motorSpeed);
        BLMotor(reverse, angularMotorSpeed);
    }
    // 225 to 270 Degrees
    else if (angle >= 225 && angle <= 270)
    {
        int angularMotorSpeed = map(angle, 225, 270, 0, motorSpeed);
        FRMotor(forward, angularMotorSpeed);
        FLMotor(reverse, motorSpeed);
        BRMotor(reverse, motorSpeed);
        BLMotor(forward, angularMotorSpeed);
    }
    // 270 to 315 Degrees
    else if (angle >= 270 && angle <= 315)
    {
        int angularMotorSpeed = map(angle, 270, 315, motorSpeed, 0);
        FRMotor(forward, motorSpeed);
        FLMotor(reverse, angularMotorSpeed);
        BRMotor(reverse, angularMotorSpeed);
        BLMotor(forward, motorSpeed);
    }
    // 315 to 360 Degrees
    else if (angle >= 315 && angle <= 360)
    {
        int angularMotorSpeed = map(angle, 315, 360, 0, motorSpeed);
        FRMotor(forward, motorSpeed);
        FLMotor(forward, angularMotorSpeed);
        BRMotor(forward, angularMotorSpeed);
        BLMotor(forward, motorSpeed);
    }
  
    // Stop Motors if Joystick is Centered
    else 
    {
        stopAllMotors();
    }
}

/////////////////////////////////////////////////////////////////////////////////

////////////////////// Setup /////////////////////////////////////////////////////

// Setup function
void setup() 
{
    Serial.begin(115200);
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
    //BP32.setup(&onConnectedController, &onDisconnectedController); 
}

/////////////////////////////////////////////////////////////////////////

//////////////////////// Main Loop //////////////////////////////////////

// Main loop
void loop() 
{

      /*
        // Read left stick for directional movement
        int16_t lx = myControllers->axisX(); // Left stick X-axis
        int16_t ly = myControllers->axisY(); // Left stick Y-axis

        // Read left trigger for throttle control
        int16_t L2 = myControllers->throttle(); // Left trigger

        int button = myControllers->buttons();

        //float angle = calculateAngle(lx, ly);
        */

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
          moveCar(controlSignal);
        }
        else if (minIndex == 0)
        {
          controlSignal = 260;
          moveCar(controlSignal);
          delay(500);
        }
        else if (minIndex == 9)
        {
          controlSignal = 100;
          moveCar(controlSignal);
          delay(200);
        }
        else 
        {
          error = 0;
          controlSignal = 0;
          moveCar(controlSignal);
        }

        Serial.print(error);
        Serial.print("  ");

        

        Serial.println(controlSignal);

        // Move Car with throttle control
        //moveCar(controlSignal);

        delay(0);
    
    
}

///////////////////////////////////////////////////////////////////////