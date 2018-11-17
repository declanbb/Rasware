#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/pwm.h>
#include <RASLib/inc/encoder.h>
#include <stdio.h>

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

void flick(void) {
    SetPin(PIN_F3, blink_on);
    blink_on = !blink_on;
}

float accelerate(float fast){
    while(fast <= 1){			//change 1 to max speed of new motor
	fast += .1;
	SetMotor(leftMotor,fast*-1);
	SetMotor(rightMotor,fast);
	Wait(.05);			//change later
    }
    return fast;
}

float decelerate(float slow){
   if (slow > 0){ 
	while(slow >= 0){			//change 1 to max speed of new motor
		slow -= .1;
		SetMotor(leftMotor,slow*-1);
		SetMotor(rightMotor,slow);
		Wait(.05);			//change later
        }
    }
    else{
    	while(slow <= 0){			//change 1 to max speed of new motor
		slow += .1;
		SetMotor(leftMotor,slow);
		SetMotor(rightMotor,slow*-1);
		Wait(.05);			//change later
    	}
    return slow;
    }
}


// The 'main' function is the entry point of the program
int main(void) {
    // Initialization code can go here
    float speed = 0;
    InitializeSystemTime();
    InitializeGPIO();
    /*
    leftPWM = InitializePWM(PIN_??, 1000); //1000 Hz PWM signal
    rightPWM = InitializePWM(PIN_??, 2000); //2000 Hz PWM signal, keep signals separate (12 internal PWM controllers differentiated by freq)
    leftEncoder = InitializeEncoder(PIN_XY, PIN_IJ, false); //true or false used to invert directionality
    rightEncoder = Initialize Encoder(PIN_UV, PIN_KL, false);
    bool leftDir = true;
    bool rightDir = true; // True=forward, false=reverse
    */  
    leftMotor = InitializeServoMotor(PIN_B7, true);
    rightMotor = InitializeServoMotor(PIN_B6, true);
    //light[0] = InitializeADC(PIN_D1);
    light[1] = InitializeADC(PIN_D2);
    light[2] = InitializeADC(PIN_D3);
    distance = InitializeADC(PIN_D0);
    int BLACK = 1500;
    
	

    speed = accelerate(speed);

    while(1){
	//int lightReading1 = 4096*ADCRead(light[0]);
	int lightReading2 = 4096*ADCRead(light[1]);
	int lightReading3 = 4096*ADCRead(light[2]);
	int distanceReading = 4096*ADCRead(distance);
	//go straight on black
	
	
	Printf("2: %d\n",distanceReading);

	if (lightReading2 >= BLACK && lightReading3 >= BLACK){
		speed = accelerate(speed);
	}
    	else if (lightReading2 < BLACK || lightReading3 < BLACK){
		speed = decelerate(speed);
		SetMotor(leftMotor,1);
		SetMotor(rightMotor,-1);		//determine halt and reverse time
		Wait(1);
		CallEvery(flick,0,.5);
		
		while (distanceReading < 1000){
		    SetMotor(leftMotor,-1);
		    distanceReading = 4096*ADCRead(distance);
		}
		    
		    Printf("2: %d\n",distanceReading);
	}


	//search for robot




	SetMotor(leftMotor,-1);
	SetMotor(rightMotor,1);


    }
}
