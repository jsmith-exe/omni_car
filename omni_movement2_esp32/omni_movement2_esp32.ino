/////////////////////////////////////////////////////////////////////////////////
////////////////   ESP32 VERSION   //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////   Movement Algorithm 2   //////////////////////////////////////////
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
#define M3_1 35
#define M3_2 36
#define M4_1 16
#define M4_2 15

#define SERVO_PIN 10  // Servo signal pin

int motorPins[] = {M1_1, M1_2, M2_1, M2_2, M3_1, M3_2, M4_1, M4_2, SERVO_PIN};

// Motor channel definitions
#define M1_1_CHANNEL 0
#define M1_2_CHANNEL 1
#define M2_1_CHANNEL 2
#define M2_2_CHANNEL 3
#define M3_1_CHANNEL 4
#define M3_2_CHANNEL 5
#define M4_1_CHANNEL 6
#define M4_2_CHANNEL 7
#define SERVO_CHANNEL 8 

int motorChannels[] = {M1_1_CHANNEL, M1_2_CHANNEL, M2_1_CHANNEL, M2_2_CHANNEL, M3_1_CHANNEL, M3_2_CHANNEL, M4_1_CHANNEL, M4_2_CHANNEL, SERVO_CHANNEL};

//////////////////////////////////////////////////////////////////////////////

/////////////////// Variables ////////////////////////////////////////////////

// LEDC configuration
#define PWM_FREQ 5000      // PWM frequency in Hz
#define PWM_RES 8   // 8-bit resolution (0-255)
#define PWM_CHANNELS 9

const String forward = "forward";
const String reverse = "reverse";

ControllerPtr myControllers; // Initialize to nullptr

// Motor Speed
int baseSpeed = 150;    // Default speed is 100/220 

// PWM Variables
float PWM1 = 0;
float PWM2 = 0;


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

void FRMotor(float pwm)
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

void FLMotor(float pwm)
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

void BRMotor(float pwm)
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

void BLMotor(float pwm)
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

///////////// Calculate Motor Angle from Controller /////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////

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

//////////////// Main Movement Function //////////////////////////////////////////////

void moveCar(float angle, int button, int motorSpeed)
{ 
  // Move Car Omni with throttle control
  if (angle >= 0)
  {
    FRMotor(PWM1);
    FLMotor(PWM2);
    BRMotor(PWM2);
    BLMotor(PWM1);
  }

  // Clockwise Rotation
  else if (button & 0x0020)
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

///////////////////////////////////////////////////////////////////////////////////

////////////// Setup function ////////////////////////////////////////////////////
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
    
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

}

///////////////////////////////////////////////////////////////////////////////////////

/////////////// Main loop /////////////////////////////////////////////////////////////

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

        // Map the left trigger value (L2) to an additional speed (0 to 120)
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
        moveCar(angle, button, maxSpeed);
    }
    else
    {
        stopAllMotors();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
