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

#define M1_1 7
#define M1_2 6
#define M2_1 5
#define M2_2 4
#define M3_1 18
#define M3_2 17
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

/////////////////// Encoder Pins /////////////////////////////////////////////

#define FR_enc1 21
#define FR_enc2 20

/////////////////// Variables ////////////////////////////////////////////////

// LEDC configuration
#define PWM_FREQ 5000      // PWM frequency in Hz
#define PWM_RES 8   // 8-bit resolution (0-255)
#define PWM_CHANNELS 8

const String forward = "forward";
const String reverse = "reverse";

ControllerPtr myControllers; // Initialize to nullptr

// Motor Speed
int baseSpeed = 120;    // Default speed is 80/204 (~40%)

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

////// Encoder Variables ////////

volatile int FR_enc_counter1 = 0;
volatile int FR_enc_counter2 = 0;

// PID Controls
float Kp = 7; //set Kp Value
float Ki = 0; //set Ki Value
float Kd = 4; //set Kd Value

float proportional = 0;
float differential = 0;
float integral = 0;

float currentTime;
float previousTime;

int error = 0;
int prevError = 0;

const int setPoint = 1260;
const int tolerance = 5;

/////////////////////////////

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

///////////// Encoder ISRs //////////////////////////////////////////////

void IRAM_ATTR FR_encoder1()
{
  FR_enc_counter1++;
}

void IRAM_ATTR FR_encoder2()
{
  FR_enc_counter2++;
}

//////////////////////////////////////////////////////////////////////////

/////////////// Encoder P Controller /////////////////////////////////////

float enc_PController() 
{
  // Calculate time step
  float currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; // in seconds
  if (dt < 0.001) dt = 0.001;  // Prevent division by zero
  
  // Get current encoder position (using counter 2)
  noInterrupts();
  int currentPosition = FR_enc_counter2;
  interrupts();
  
  // Compute error
  int error = setPoint - currentPosition;
  
  // If we are within tolerance or overshot the target, command zero output.
  if(error <= tolerance) 
  {
    return 0;
  }
  
  // Compute PID terms
  float Pout = Kp * error;
  integral += error * dt;
  float Iout = Ki * integral;
  float derivative = (error - prevError) / dt;
  float Dout = Kd * derivative;
  
  float output = Pout + Iout + Dout;
  
  // Save error and time for the next iteration
  prevError = error;
  previousTime = currentTime;
  
  // If the computed output is negative (would mean reversing), force it to zero.
  if (output < 0) 
  {
    output = 0;
  }
  // Clamp output to the PWM limits.
  if (output > 255) 
  {
    output = 255;
  }
  
  return output;
}


////////////////////////////////////////////////////////////////////////

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
    /*
    // Clockwise Rotation (using button bitmask 0x0020)
    else if (button & 0x0020)
    {
        FRMotor(reverse, motorSpeed);
        FLMotor(forward, motorSpeed);
        BRMotor(reverse, motorSpeed);
        BLMotor(forward, motorSpeed);
    }
    /*
    // Anti-Clockwise Rotation (using button bitmask 0x0010)
    else if (button & 0x0010)
    {
        FRMotor(forward, motorSpeed);
        FLMotor(reverse, motorSpeed);
        BRMotor(forward, motorSpeed);
        BLMotor(reverse, motorSpeed);
    } */
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

    // Encoder pins setup
    pinMode(FR_enc1, INPUT_PULLUP);
    pinMode(FR_enc2, INPUT_PULLUP);

    // Attach interrupts using the correct macro
    attachInterrupt(digitalPinToInterrupt(FR_enc1), FR_encoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_enc2), FR_encoder2, CHANGE);
    
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    previousTime = millis();
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

        float percent = (FR_enc_counter2 * 100) / 1260;

        Serial.print("Enc1: ");
        Serial.print(FR_enc_counter1);
        Serial.print("  ");
        Serial.print("Enc2: ");
        Serial.print(FR_enc_counter2);
        Serial.print("  Percent: ");
        Serial.println(percent);

        float motorSpeedEncoder = enc_PController();
        
        FRMotor(forward, motorSpeedEncoder);

    }
    else
    {
        stopAllMotors();
    }
}

///////////////////////////////////////////////////////////////////////

