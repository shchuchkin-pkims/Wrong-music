/*********
  Shchuchkin Evgeny
  
  First version of "Wrong Music" rocket stabilization system controller program.
  Rocket stabilizers can be turned from -45 to +45 degrees. In this program 0 degrees is equal to 90 servo degrees. First servo is X axis, second is Y axis.
  
*********/

#include "ServoSmooth.h"

ServoSmooth servos[2]			// for 2 servo stabilizers

uint32_t servoTimer;
uint32_t turnTimer;

int position[2] = {90, 90};		// servos positions in degrees

void setup() {
//Servos init section	
	//Serial.begin(9600);	//!!!!For debug
	servos[0].attach(2, 600, 2400);  // pin numer, 600 и 2400 - min/max pulse duration
	//servos[1].attach(3, 600, 2400);  // pin numer, 600 и 2400 - min/max pulse duration

	servos[0].setSpeed(180);	// default 100 deg/s
	servos[0].setAccel(0.1);	// default 0.2
	//servos[1].setSpeed(90);
	//servos[1].setAccel(0.5);
}

void loop() {
// Gyroscope code section
	
	
// Servos code section	
	// Every 20 ms
	if (millis() - servoTimer >= 20) {	// millis - time after program start
		servoTimer += 20;				// todo: with uint32_t variable program will turn off after 49.7 days after start. Need another algorithm for another tasks.
		
		servos[0].tickManual();			// turn 1 servo in cycle
		servos[1].tickManual();			// turn 2 servo in cycle
		}
	}
	// For debug: every 1 second turn on. Need to test real time speed.
	if (millis() - turnTimer >= 1000) {
		turnTimer = millis();
		
		servos[0].setTargetDeg(position[0]);
		servos[1].setTargetDeg(position[1]);
	}
}

// Servos original https://alexgyver.ru/servosmooth/
// Gyro https://arduino-diy.com/arduino-MPU6050-dlya-opredeleniya-ugla-naklona
//		https://alexgyver.ru/arduino-mpu6050/
//		https://robototehnika.ru/content/article/balansiruyushchiy-robot-na-arduino/
//		https://robotclass.ru/tutorials/arduino-accelerometer-mpu6050/