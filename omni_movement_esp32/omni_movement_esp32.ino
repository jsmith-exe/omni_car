/////////////////////////////////////////////////////////////////////////////////
////////////////   ESP32 VERSION   //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////   JAMIE SMITH   ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////////////   Queen's University Belfast   //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////// Librarys ////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Bluepad32.h>

/////////////////////////////////////////////////////////////////////////////////

//////////////// Motor pin definitions //////////////////////////////////////////

#define M1_1 4
#define M1_2 5
#define M2_1 6
#define M2_2 7
#define M3_1 15
#define M3_2 16
#define M4_1 17
#define M4_2 18 

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

/////////////////// Variables ////////////////////////////////////////////////

// LEDC configuration
#define PWM_FREQ 5000      // PWM frequency in Hz
#define PWM_RES 8   // 8-bit resolution (0-255)
#define PWM_CHANNELS 8

const String forward = "forward";
const String reverse = "reverse";

ControllerPtr myControllers; // Initialize to nullptr

// Motor Speed
int baseSpeed = 80;    // Default speed is 80/204 (~40%)

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

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

///////////// Main Movement Algoithm //////////////////////////////////

// Function to control motors based on joystick input
void moveCar(int16_t lx, int16_t ly, int motorSpeed, float angle, int button)
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
    // Clockwise Rotation (using button bitmask 0x0020)
    else if (button & 0x0020)
    {
        FRMotor(reverse, motorSpeed);
        FLMotor(forward, motorSpeed);
        BRMotor(reverse, motorSpeed);
        BLMotor(forward, motorSpeed);
    }
    // Anti-Clockwise Rotation (using button bitmask 0x0010)
    else if (button & 0x0010)
    {
        FRMotor(forward, motorSpeed);
        FLMotor(reverse, motorSpeed);
        BRMotor(forward, motorSpeed);
        BLMotor(reverse, motorSpeed);
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
    //Serial.begin(115200);
    //Serial.println("Starting Bluepad32...");

    // Motor Setup
    for (int i = 0; i < PWM_CHANNELS; i++)
    {
      ledcSetup(i, PWM_FREQ, PWM_RES); // Configure PWM
      ledcAttachPin(motorPins[i], motorChannels[i]); // Attach pin to channel
    }
    
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

/////////////////////////////////////////////////////////////////////////

//////////////////////// Main Loop //////////////////////////////////////

// Main loop
void loop() 
{
    BP32.update();
    if (myControllers && myControllers->isConnected()) 
    {
        // Read left stick for directional movement
        int16_t lx = myControllers->axisX(); // Left stick X-axis
        int16_t ly = myControllers->axisY(); // Left stick Y-axis

        // Read left trigger for throttle control
        int16_t L2 = myControllers->throttle(); // Left trigger

        int button = myControllers->buttons();

        float angle = calculateAngle(lx, ly);

        //Serial.println(angle);
        
        // Map the left trigger value (L2) to an additional speed (0 to 205)
        int additionalSpeed = map(L2, 0, 1023, 0, 124);

        // Combine base speed with additional speed
        int motorSpeed = baseSpeed + additionalSpeed;

        // Move Car with throttle control
        moveCar(lx, ly, motorSpeed, angle, button);
    }
    else
    {
        stopAllMotors();
    }
}

///////////////////////////////////////////////////////////////////////

