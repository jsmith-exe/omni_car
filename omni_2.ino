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

GamepadPtr myGamepad;

// Motor Speed
int baseSpeed = 50; // Default speed is 50/255 (~20%)
int motorSpeed = 0; // Variable to hold the final speed

// Joystick thresholds
int thresholdLow = -512;
int thresholdHigh = 512;

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
        
        // Map the left trigger value (l2) to an additional speed (0 to 154)
        int additionalSpeed = map(L2, 0, 1023, 0, 154);

        // Combine base speed with additional speed
        motorSpeed = baseSpeed + additionalSpeed;

        // Move Car with throttle control
        diagonal_1(motorSpeed);
        diagonal_2(motorSpeed);
    }
    else
    {
        stopAllMotors();
    }
}

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

void diagonal_1 (float pwm)
{
  if (pwm < 0)
  {
    pwm = pwm * -1;
    analogWrite(M1_1, 0);
    analogWrite(M4_1, 0);
    analogWrite(M1_2, pwm);
    analogWrite(M4_2, pwm);
  }
  else if (pwm >= 0)
  {
    analogWrite(M1_2, 0);
    analogWrite(M4_2, 0);
    analogWrite(M1_1, pwm);
    analogWrite(M4_1, pwm);
  }
  else 
  {
    analogWrite(M1_1, 0);
    analogWrite(M1_2, 0);
    analogWrite(M4_1, 0);
    analogWrite(M4_2, 0);
  }
}

void diagonal_2 (float pwm)
{
  if (pwm < 0)
  {
    pwm = pwm * -1;
    analogWrite(M2_2, 0);
    analogWrite(M3_2, 0);
    analogWrite(M2_1, pwm);
    analogWrite(M3_1, pwm);
  }
  else if (pwm >= 0)
  {
    analogWrite(M2_1, 0);
    analogWrite(M3_1, 0);
    analogWrite(M2_2, pwm);
    analogWrite(M3_2, pwm);
  }
  else 
  {
    analogWrite(M2_1, 0);
    analogWrite(M2_2, 0);
    analogWrite(M3_1, 0);
    analogWrite(M3_2, 0);
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
