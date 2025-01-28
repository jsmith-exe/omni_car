#include <Bluepad32.h>

// Motor pin definitions
#define M1_1 11
#define M1_2 12
#define M2_1 10
#define M2_2 9
#define M3_1 6
#define M3_2 5
#define M4_1 3
#define M4_2 2

const String forward = "forward";
const String reverse = "reverse";

GamepadPtr myGamepad;

// Motor Speed
int baseSpeed = 50; // Default speed is 50/255 (~20%)
int motorSpeed = 255; // Variable to hold the final speed

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

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

// Movement Functions with throttle control

void FRMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      analogWrite(M1_2, 0);
      analogWrite(M1_1, speed);
      break;

    case 'r':  
      analogWrite(M1_1, 0);
      analogWrite(M1_2, speed);
      break;

    default:
      analogWrite(M1_1, 0);
      analogWrite(M1_2, 0);
      break;
  }
}

void FLMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      analogWrite(M2_1, 0);
      analogWrite(M2_2, speed);
      break;

    case 'r':  
      analogWrite(M2_2, 0);
      analogWrite(M2_1, speed);
      break;

    default:
      analogWrite(M2_1, 0);
      analogWrite(M2_2, 0);
      break;
  }
}

void BRMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      analogWrite(M3_1, 0);
      analogWrite(M3_2, speed);
      break;

    case 'r':  
      analogWrite(M3_2, 0);
      analogWrite(M3_1, speed);
      break;

    default:
      analogWrite(M3_1, 0);
      analogWrite(M3_2, 0);
      break;
  }
}

void BLMotor(String movement, int speed)
{
  switch (movement[0])
  {
    case 'f':  
      analogWrite(M4_2, 0);
      analogWrite(M4_1, speed);
      break;

    case 'r':  
      analogWrite(M4_1, 0);
      analogWrite(M4_2, speed);
      break;

    default:
      analogWrite(M4_1, 0);
      analogWrite(M4_2, 0);
      break;
  }
}

float calculateAngle(int16_t lx, int16_t ly) 
{
  int resultMagnitude = sqrt((lx * lx) + (ly * ly));
  if (resultMagnitude > 480)
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

// Setup function
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
    // Clockwise Rotation
    else if (button & 0x0020)
    {
        FRMotor(reverse, motorSpeed);
        FLMotor(forward, motorSpeed);
        BRMotor(reverse, motorSpeed);
        BLMotor(forward, motorSpeed);
    }
    // Anti-Clockwise Rotation
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

// Main loop
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

        int button = myGamepad->buttons();

        float angle = calculateAngle(lx, ly);
        Serial.println(angle);
        
        // Map the left trigger value (l2) to an additional speed (0 to 205)
        int additionalSpeed = map(L2, 0, 1023, 0, 205);

        // Combine base speed with additional speed
        motorSpeed = baseSpeed + additionalSpeed;

        // Move Car with throttle control
        moveCar(lx, ly, motorSpeed, angle, button);

    }
    else
    {
        stopAllMotors();
    }
}
