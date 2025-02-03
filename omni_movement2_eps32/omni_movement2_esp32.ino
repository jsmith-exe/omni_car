#include <Bluepad32.h>

//////////// Motor pin definitions ////////////////

#define M1_1 11
#define M1_2 12
#define M2_1 10
#define M2_2 9
#define M3_1 6
#define M3_2 5
#define M4_1 3
#define M4_2 2

//////////////////////////////////////////////////

////////////// Variables /////////////////////////

GamepadPtr myGamepad;

// Motor Speed
int baseSpeed = 50; // Default speed is 50/255 (~20%)

// Joystick thresholds
int joystickThreshold = 480;

/////////////////////////////////////////////////

////////// Bluetooth Connecting Functions //////////

// Callback when a gamepad is connected
void onConnectedGamepad(GamepadPtr gp) 
{
    myGamepad = gp;
}

// Callback when a gamepad is disconnected
void onDisconnectedGamepad(GamepadPtr gp) 
{
    myGamepad = nullptr;
}

///////////////////////////////////////////////////

////////// Calculate the Input Angle from the Joystick //////////

float calculateAngle(int16_t lx, int16_t ly) 
{
  int resultMagnitude = sqrt((lx * lx) + (ly * ly));
  if (resultMagnitude > joystickThreshold)
  {
    float angle = atan2(ly, lx);
    angle = (angle * 180 / M_PI) + 90;
    if (angle <0) angle += 360;
    
    return angle;
  }
  else
  {
    return -1;
  }
}
//////////////////////////////////////////////////////////////

//////////// Individual Movement Functions with throttle control /////////////

void FRMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    analogWrite(M1_1, 0);
    analogWrite(M1_2, pwm);
  }
  else if (pwm >= 0)
  { // forward
    analogWrite(M1_2, 0);
    analogWrite(M1_1, pwm);
  }
  else 
  {
    analogWrite(M1_1, 0);
    analogWrite(M1_2, 0);
  }
}

void FLMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    analogWrite(M2_2, 0);
    analogWrite(M2_1, pwm);
  }
  else if (pwm >= 0)
  { // forward
    analogWrite(M2_1, 0);
    analogWrite(M2_2, pwm);
  }
  else 
  {
    analogWrite(M2_1, 0);
    analogWrite(M2_2, 0);
  }
}

void BRMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    analogWrite(M3_2, 0);
    analogWrite(M3_1, pwm);
  }
  else if (pwm >= 0)
  { // forward
    analogWrite(M3_1, 0);
    analogWrite(M3_2, pwm);
  }
  else 
  {
    analogWrite(M3_1, 0);
    analogWrite(M3_2, 0);
  }
}

void BLMotor(float pwm)
{
  if (pwm < 0)
  { // reverse
    pwm = pwm * -1;
    analogWrite(M4_1, 0);
    analogWrite(M4_2, pwm);
  }
  else if (pwm >= 0)
  { // forward
    analogWrite(M4_2, 0);
    analogWrite(M4_1, pwm);
  }
  else 
  {
    analogWrite(M4_1, 0);
    analogWrite(M4_2, 0);
  }
}

// Stop all motors

void stopAllMotors() 
{
    analogWrite(M1_1, 0);
    analogWrite(M1_2, 0);
    analogWrite(M2_1, 0);
    analogWrite(M2_2, 0);
    analogWrite(M3_1, 0);
    analogWrite(M3_2, 0);
    analogWrite(M4_1, 0);
    analogWrite(M4_2, 0);
}

/////////////////////////////////////////////////////////////////////////////


////////// Motor angle multipler fucntions //////////

float diagonalMotors_CW(float angle) 
{
    if (angle >= 0 && angle <= 90) 
    {
        // Linear equation from 1 to 0
        return (1 - (angle / 45));
    } 
    else if (angle > 90 && angle <= 180) 
    {
        // Constant signal -1
        return -1;
    } 
    else if (angle > 180 && angle <= 270) 
    {
        // Linear equation from -1 to 0
        return ((angle / 45) - 5);
    } 
    else if (angle > 270 && angle <= 360) 
    {
        // Constant signal 1
        return 1;
    } 
    else {
        // Default case
        return 0;
    }
}

float diagonalMotors_ACW(float angle) 
{
  return diagonalMotors_CW(360 - angle);  // Mirror CW about 360
}

//////////////////////////////////////////////////////

/////////// Main Movement Function /////////////////

void moveCar(int motorSpeed, float angle, int button)
{
  float diagonal_1_multiplier = diagonalMotors_CW(angle);
  float diagonal_2_multiplier = diagonalMotors_ACW(angle);

  float diagonal_1_motor_pwm = diagonal_1_multiplier * motorSpeed;
  float diagonal_2_motor_pwm = diagonal_2_multiplier * motorSpeed;
  
  // Move Car Omni with throttle control
  FRMotor(diagonal_1_motor_pwm);
  FLMotor(diagonal_2_motor_pwm);
  BRMotor(diagonal_2_motor_pwm);
  BLMotor(diagonal_1_motor_pwm);

  // Clockwise Rotation
  if (button & 0x0020)
  {
    FRMotor(motorSpeed * -1);
    FLMotor(motorSpeed);
    BRMotor(motorSpeed * -1);
    BLMotor(motorSpeed);
  }
  // Anti-Clockwise Rotation
  else if (button & 0x0010)
  {
    FRMotor(motorSpeed);
    FLMotor(motorSpeed * -1);
    BRMotor(motorSpeed);
    BLMotor(motorSpeed * -1);
   }
   // Stop Motors if Joystick is Centered
   else 
   {
     stopAllMotors();
   }
}

///////////////////////////////////////////////////

////////////// Setup function ////////////////
void setup() 
{
    // Initialize motor pins
    pinMode(M1_1, OUTPUT);
    pinMode(M1_2, OUTPUT);
    pinMode(M2_1, OUTPUT);
    pinMode(M2_2, OUTPUT);
    pinMode(M3_1, OUTPUT);
    pinMode(M3_2, OUTPUT);
    pinMode(M4_1, OUTPUT);
    pinMode(M4_2, OUTPUT);
    
    // Initialize Bluepad32 or other gamepad library
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
}

//////////////////////////////////////////////

/////////////// Main loop /////////////////////////

void loop() 
{
    BP32.update();
    if (myGamepad) 
    {
        // Read left stick for directional movement
        int16_t lx = myGamepad->axisX(); // Left stick X-axis
        int16_t ly = myGamepad->axisY(); // Left stick Y-axis

        // Read left trigger for throttle control
        int16_t L2 = myGamepad->throttle(); // Left trigger

        // Read button input
        int button = myGamepad->buttons();

        // Calculate joystick angle
        float angle = calculateAngle(lx, ly);
        
        // Map the left trigger value (L2) to an additional speed (0 to 154)
        int additionalSpeed = map(L2, 0, 1023, 0, 154);

        // Combine base speed with additional speed
        int motorSpeed = baseSpeed + additionalSpeed;

        // Move the car
        moveCar(motorSpeed, angle, button);
    }
}

///////////////////////////////////////////////////////
