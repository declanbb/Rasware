#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/pwm.h>
#include <RASLib/inc/encoder.h>

// Blink the LED to show we're on
tBoolean blink_on = true;
static tMotor *leftMotor;
static tMotor *rightMotor;
static tADC *light[3];
static tADC *distance;
static tPWM *leftPWM;
static tPWM *rightPWM;
static tEncoder *leftEncoder;
static tEncoder *rightEncoder;

tPin leftPWMPin = PIN_B7;
tPin rightPWMPin = PIN_B6;
tPin leftDirPin = PIN_C6;
tPin rightDirPin = PIN_C7;

// PID control struct
struct PID {
	float goal;
	float total;
	signed long oldEncoderVal;
	tTime oldTime;
	float oldPErr;
	float integral;
	float derivative;
};

struct PID leftpid;
struct PID rightpid;

// estimates for PID control
static float pErrCoeff = 0.5;
static float iErrCoeff = 0.5;
static float dErrCoeff = 0.5;

static float maxSpeed = 0.000012; // Units of inches per microsecond


void flick(void) {
    SetPin(PIN_F3, blink_on);
    blink_on = !blink_on;
}

// accelerate function for non-PWM motor
float accelerate(float fast){
    while(fast <= 1){			//change 1 to max speed of new motor
		fast += .1;
		SetMotor(leftMotor,fast*-1);
		SetMotor(rightMotor,fast);
		Wait(.05);			//change later
    }
    return fast;
}

// decelerate function for non-PWM motor
float decelerate(float slow){
	if (slow > 0) { 
		while(slow >= 0) {			//change 1 to max speed of new motor
			slow -= .1;
			SetMotor(leftMotor,slow*-1);
			SetMotor(rightMotor,slow);
			Wait(.05);			//change later
        }
    } else {
    	while(slow <= 0){			//change 1 to max speed of new motor
			slow += .1;
			SetMotor(leftMotor,slow);
			SetMotor(rightMotor,slow*-1);
			Wait(.05);			//change later
    	}
    	return slow;
    }
}

void initializeMotor() {
	leftMotor = InitializeServoMotor(leftPWMPin, true);
  rightMotor = InitializeServoMotor(rightPWMPin, true);

void initializeSensors() {
	//light[0] = InitializeADC(PIN_D1); // light 0 sensor is broken
    light[1] = InitializeADC(PIN_D2);
    light[2] = InitializeADC(PIN_D3);
    distance = InitializeADC(PIN_D0);
}

void initializePWM() {
	int startSignalLeft = 2000; // 2000Hz PWM signal
	int startSignalRight = 2000; // 2000Hz PWM signal
	leftPWM = InitializePWM(leftPWMPin, startSignalLeft);
	rightPWM = InitializePWM(rightPWMPin, startSignalRight);
	leftEncoder = InitializeEncoder(PIN_F2, PIN_F3, false);
	rightEncoder = InitializeEncoder(PIN_C4, PIN_C5, false);
	SetPin(leftDirPin, true); // left forward
	SetPin(rightDirPin, false); // right backward
}

void initializePIDControl(struct PID pid, tEncoder *encoder) {
	pid.goal = maxSpeed; // placeholder
	pid.oldTotal = 0.2;
	pid.oldEncoderVal = GetEncoder(encoder);
	pid.oldTime = GetTimeUS();
	pid.oldPErr = 0.0;
	pid.integral = 0.0;
	pid.derivative = 0.0;
}

int* getLightReadings() {
	int lightReadings[3];
	lightReadings[0] = 0; // broken
	lightReadings[1] = 4069 * ADCRead(light[1]);
	lightReadings[2] = 4069 * ADCRead(light[2]);
	return lightReadings;
}

int getDistanceReading() {
	return 4096*ADCRead(distance);
}

bool checkBlack(int lightReadings[]) {
	int black = 1500;
	return (lightReadings[1] >= black && lightReadings[2] >= black);
}

float convertEncoderValToSpeed(signed long encoderVal, float deltaTime) {
	float distance = encoderVal * (float) (1/59.32);
	return distance / deltaTime; //This speed is in units of inches per microsecond
}

void calculateSpeedOutput(struct PID pid, tEncoder *encoder) {
	// time
	tTime newTime = GetTimeUS();
	tTime deltaTime = (float)(newTime - pid.oldTime);
	// encoder values
	signed long newEncoderVal = GetEncoder(encoder);
	signed long deltaEncoderVal = newEncoderVal - pid.oldEncoderVal;
	float curSpeed = convertEncoderValToSpeed(deltaEncoderVal, deltaTime); //speed in inches per microsecond
	float pErr = pid.goal - curSpeed;
	// integral and derivative
	pid.integral += pErr * deltaTime;
	pid.derivative = (pid.oldPErr + pErr) / (deltaTime);
	// calculate speed
	pid.total += pErrCoeff*pErr + iErrCoeff*pid.integral + dErrCoeff*pid.derivative;

	// reset things
	pid.oldEncoderVal = newEncoderVal;
	pid.oldTime = newTime;
	pid.oldPErr = pErr;
}

void setPIDGoal(struct PID pid, float goal) {
	pid.goal = goal;
}

void setPWMMotorSpeed() {
	calculateSpeedOutput(leftpid, leftEncoder);
	calculateSpeedOutput(rightpid, rightEncoder);
	setPIN(leftDirPin, (leftpid.total > 0));
	setPIN(rightDirPin, (rightpid.total > 0));
	setPWM(leftPWMPin, abs(leftpid.total), 1);
	setPWM(rightPWMPin, abs(rightpid.total), 1);
}

bool checkMotionless(tEncoder *encoder, struct PID pid) {
	signed long curEncoderVal = GetEncoder(encoder);
	return (curEncoderVal == pid.oldEncoderVal);
}

void halt() {
	setPIDGoal(leftpid, 0);
	setPIDGoal(rightpid, 0);
	// decrease speed until it's motionless
	while (!checkMotionless(leftEncoder, leftpid) && !checkMotionless(rightEncoder, rightpid)) {
		setPWMMotorSpeed();
	}
}

void backoff() {
	setPIDGoal(leftpid, (-1)*maxSpeed);
	setPIDGoal(rightpid, (-1)*maxSpeed);

	signed long oldEncoderLVal = leftpid.oldEncoderVal;
	signed long oldEncoderRVal = rightpid.oldEncoderVal;
	signed long deltaL;
	signed long deltaR;
	while (deltaL > -240 && deltaR > -240) {
		deltaL = GetEncoder(leftEncoder) - oldEncoderLVal;
		deltaR = GetEncoder(rightEncoder) - oldEncoderRVal;
		setPWMMotorSpeed();
	}
}

// The 'main' function is the entry point of the program
int main(void) {
	//Wait 1 second for field to clear
	Wait(1);

    // constants and variables
    int lightReadings[3];
    int distanceReading;
    int minDistance = 1000;

    // initialization
    InitializeSystemTime();
    InitializeGPIO();
    initializePWM();
    initializeSensors();
    // PID initialization
    initializePIDControl(leftpid, leftEncoder);
    initializePIDControl(rightpid, rightEncoder);

    while (1) {
		lightReadings = getLightReadings();
		distanceReading = getDistanceReading();
		// Printf("Distance: %d\n", distanceReading)

		if (checkBlack(lightReadings)) {
			setPIDGoal(leftpid, maxSpeed);
			setPIDGoal(rightpid, maxSpeed);
			setPWMMotorSpeed();
		} else { // we see white
			halt();
			// back off a bit
			backoff();
			// detect and face object
			setPIDGoal(leftpid, -1); // reverse in place
			setPIDGoal(rightpid, 1);
			while (distanceReading < minDistance) {
				setPWMMotorSpeed();
			}
		}
}
