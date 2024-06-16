/*
 * MotorMgr.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#include "MotorMgr.h"
#include <math.h>

// Sets up each motor's GPIO pins and PWM

MotorMgr::MotorMgr(uint8_t dir1, uint8_t dir2, uint8_t gpA, uint8_t gpB, uint8_t pwm) {

	printf("MotorMgr::MotorMgr init: dir1 %d - dir2 - %d gpA - %d gpB - %d pwm - %d\n", dir1, dir2, gpA, gpB, pwm);
	printf("Setting up a motor ...\n");

	xDir1 = dir1;
	xDir2 = dir2;
	xGPA = gpA;
	xGPB = gpB;
	xPWM = pwm;

	// Set up the GPIO pins for the rotary encoder
	gpio_init(xPWM);
	gpio_set_dir(xPWM, GPIO_OUT);
	gpio_set_function(xPWM, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xPWM, 0);
	uint slice_num = pwm_gpio_to_slice_num(xPWM);
	pwm_set_wrap(slice_num, 255);
	pwm_set_enabled(slice_num, true);

	gpio_init(xDir1);
    gpio_set_dir(xDir1, GPIO_OUT);
	gpio_init(xDir2);
    gpio_set_dir(xDir2, GPIO_OUT);

	// set up in addObserver
	//gpio_init(xGPA);
    //gpio_init(xGPB);
    //gpio_set_dir(xGPA, GPIO_IN);
    //gpio_set_dir(xGPB, GPIO_IN);
	//gpio_pull_up(xGPA);
    //gpio_pull_up(xGPB);

	// Set up the GPIO pins for the rotary encoder
	GPIOInputMgr::getMgr()->addObserver(xGPA, this);
	GPIOInputMgr::getMgr()->addObserver(xGPB, this);

}

MotorMgr::~MotorMgr() {
	// TODO Auto-generated destructor stub
}


void MotorMgr::setThrottle(float percent, bool cw){
	xThrottle = percent;
	xCW = cw;

	if (xThrottle < 0 ){
		xThrottle == 0.0;
	}

	if (xThrottle == 0.0){
		xActRPM = 0.0;
		xLastTime = 0;
		pwm_set_gpio_level(xPWM, 0);
		//pwm_set_gpio_level(xPWM, 0);
		//printf("PWM STOP\n");
		return;
	}

	if (xThrottle > 1.0 ){
		xThrottle = 1.0;
	}

	int pwm = (int)((float)(0xffff) * xThrottle);
	if (cw){
		pwm_set_gpio_level(xPWM, 0);
		pwm_set_gpio_level(xPWM, pwm);
		gpio_put(xDir1, true);
        gpio_put(xDir2, false);
		printf("GP%d, pwm %d\n", xCW, pwm);
	} else {
		pwm_set_gpio_level(xPWM, 0);
		pwm_set_gpio_level(xPWM, pwm);
		gpio_put(xDir1, false);
        gpio_put(xDir2, true);
		printf("GP%d, pwm %d\n", xCW, pwm);
	}
	printf("PWM %d, Throttle %0.2f\n", pwm, xThrottle);
}


// Handle the GPIO interrupts for the rotary encoders A and B

void MotorMgr::handleGPIO(uint gpio, uint32_t events){
	
	//if (gpio == xGPSlot){
	//	handleCalibration(gpio, events);
	//	return;
	//}

	//printf("MotorMgr::handleGPIO %d %d\n", gpio, events);

	char wheels[2][10] = {"left", "right"};
	
	int index = 0;
	if (gpio == 12 || gpio == 13){
		index = 0;
	} else{
		index = 1;
	}

	uint8_t c;
	c = gpio_get(xGPA);
	c = c << 1;
	c = (gpio_get(xGPB)) | c;


	// test if moving forward
	if (xRotEncCW[xLast] == c){
		xCount++;
		if (xCount > 3){
			xPos++;
			xDeltaPos++;
			if (xPos == xNumTicks){
				xPos = 0;
			}
			//printf("%s forward (gpio %d) - %d %d\n", wheels[index], gpio, xPos, xCount);
			handleRotate(true);
			xCount = 0;
		}
		xLast = c;
	}

	// test if moving backward
	if (xRotEncCCW[xLast] == c){
		xCount-- ;
		if (xCount < -3){
			xPos--;
			xDeltaPos--;
			if (xPos == -1){
				xPos = xNumTicks - 1;
			}
			//printf("%s reverse (gpio %d) - %d %d\n", wheels[index], gpio, xPos, xCount);
			handleRotate(false);
			xCount = 0;
		}
		xLast = c;
	}
}

void MotorMgr::handleRotate(bool cw){

	#ifdef MOTOR_DEBUG
		if(cw){
			//printf("MotorMgr::handleRotate forward\n");
		}else{
			//printf("MotorMgr::handleRotate reverse\n");
		}
	#endif //MOTOR_DEBUG

	uint32_t now = to_ms_since_boot (get_absolute_time ());

	if (xLastTime != 0){
		uint32_t ms = now - xLastTime;
		float rpm = 60000.0 / (float)ms;
		rpm = rpm / (float)xNumTicks;
		xActRPM = rpm;
		xMvAvgRPM = (rpm * 1.0 + xMvAvgRPM * 3.0)/ 4.0;

		#ifdef MOTOR_DEBUG
			printf("MotorMgr::handleRotate RPM %0.2f\n", xActRPM);
		#endif //MOTOR_DEBUG

	}
	xLastTime = now;
}


float MotorMgr::getThrottle(){
	return xThrottle;
}


float MotorMgr::getRPM(){
	//Check we have recently updated RPM, otherwise we are stopped
	uint32_t now = to_ms_since_boot (get_absolute_time ());
	uint32_t ms = now - xLastTime;
	if (ms > 250){
		xActRPM = 0.0;
	}
	return xActRPM;

}


bool MotorMgr::isCW(){
	return xCW;
}


float MotorMgr::getMovingAvgRPM(){
	if (getRPM() == 0.0){
		xMvAvgRPM = 0.0;
	}
	return xMvAvgRPM;
}


/***
 * Radian possition of wheel
 * @return 0.0 >= r < 2* PI
 */
float MotorMgr::getRadians(){
	float rad = (float)xPos / (float)xNumTicks;
	rad = rad * (2.0 * M_PI);
	return rad;
}


float MotorMgr::getAvgRadPerSec(){
	float rpm = getMovingAvgRPM();
	float rps = (rpm / 60.0) * (2.0 * M_PI);

	return rps;
}


/***
 * Get the delta of ticks since last cleared
 * @param clear - if true will be reset to zero after call
 * @return number of ROTENC ticks
 */
int32_t MotorMgr::getDeltaPos(bool clear){
	int32_t res = xDeltaPos;
	if (clear){
		xDeltaPos = 0;
	}
	return res;
}

/***
 * Get the delta of radians since last cleared
 * @param clear - if true will be reset to zero after call
 * @return Radians turn since last call (>0 CW, <0 CCW).
 */
float MotorMgr::getDeltaRadians(bool clear){
	float res = (float)getDeltaPos(clear);
	res = res / (float)xNumTicks;
	res = res * (2.0 * M_PI);
	return res;
}


