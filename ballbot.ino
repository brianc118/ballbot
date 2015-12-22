/* Program for my ballbot, found at http://onewaytobalance.blogspot.com
 * 
 * Program pretty much runs on an interrupt.
 * Configurations, settings and global variables/objects found in ballbot.h
 *
 * Only runs on Teensy 3.x and requires a decent clock (I use 96MHz)
 * This is because it is quite processing intensive since the single MCU does
 *    - on board Mahony sensor fusion at >1000Hz using 9dof data from the MPU9250
 *          --> converting quaternions to Euler angles unfortunately requires CPU intensive 
 *				inverse trigonometry
 *    - PID roll/pitch control in 2 dimensions (i.e. two PID control loops for balancing)
 *    - Polling encoders at a high sample rate (64cpr encoders and a 12400rpm motor - no load)
 *			--> Finding velocity of each of the 3 wheels
 *    - Using wheel velocities and trig to obtain absolate location/position of robot
 *          see http://onewaytobalance.blogspot.com.au/2015/12/omnidirectional-control.html
 *          The maths requires you to have a basic understanding of linear algebra (matrices etc.)
 *    - PID control for position of robot
 *    - Debugging via bluetooth and serial to computer (the RN42 is quite slow)
 *    
 *
 * (C) Brian Chen 2015
 */

#include <SPI.h>
#include <EEPROM.h>
#include <FrequencyTimer2.h>  // Built into teensy core

/* own libraries */
#include <EEPROMAnything.h>   // see https://github.com/brianchen118/Team-PI-Lib/tree/master/EEPROMAnything
#include <MPU9250.h>          // see https://github.com/brianchen118/MPU9250
#include <pwmMotor.h>         // see https://github.com/brianchen118/Team-PI-Lib/tree/master/pwmMotor
#include <fastTrig.h>		  // https://github.com/brianchen118/fastTrig
#include <omnidrive.h>		  // see https://github.com/brianchen118/omnidrive
#include <sensorfusion.h>     // see https://github.com/brianchen118/sensorfusion
#include "balanceController.h"
#include "locationCalculator.h"
#include "ballbot.h"


/* Main part of the program begins here.
 *
 */

void thingy(){
	if (BT.available()){
 		while(BT.available()){
	 		Serial.print((char)BT.read());
	 	}
 	}
 	else{
 		Serial.println("none");
 	}
}

extern "C" int main () {
	SPI.setSCK(SCK_PIN);    // change SCK pin from 13 to 14 since 13 also has a useful LED
	SPI.begin();
	Serial.begin(115200);   // doesn't really matter what we put here (Teensy 3.x serial isn't "real" serial)
	// WAITFORINPUT();
	// CLEARSERIAL();
	// BT.begin(230400);  // RN42 default baud
	// delay(200);
	// BT.print("$$$");

	// delay(200);
	// thingy();

	// BT.println("SU,23");
	// delay(200);
	// thingy();

	// BT.println("E");
	// delay(400);
	// thingy();

	// BT.println("D");
	// delay(400);
	// thingy();
	
	// BT.println("---");
	// BT.begin(230400);
	// delay(200);
	// thingy();

	// WAITFORINPUT();
	// CLEARSERIAL();
	BT.begin(230400);

	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);	digitalWriteFast(LED, HIGH);   // initial set LEDs on high
	pinMode(LED2, OUTPUT);	digitalWriteFast(LED2, HIGH);  

	// use a higher frequency of >12kHz to reduce audible sound and reduce switching losses
	motorA.begin(PWM_FREQ);  
	motorB.begin(PWM_FREQ);
	motorC.begin(PWM_FREQ);

	pinMode(ENCXA, INPUT);  pinMode(ENCYA, INPUT);  pinMode(ENCZA, INPUT);
	pinMode(ENCXB, INPUT);  pinMode(ENCYB, INPUT);  pinMode(ENCZB, INPUT);

	mpu.init(true, false);  // don't calib acc. as this screws up everything

	uint8_t wai = mpu.whoami();
	if (wai == 0x71)
		Serial.println("Successful connection to mpu");
	else
		Serial.print("Failed connection to mpu: ");
		Serial.println(wai, HEX);

	uint8_t wai_AK8963 = mpu.AK8963_whoami();

	if (wai_AK8963 == 0x48)
		Serial.println("Successful connection to mag");
	else
		Serial.print("Failed connection to mag: ");
		Serial.println(wai_AK8963, HEX);

	//mpu.calib_acc();
	mpu.calib_mag();

	readMagCalib();
	readBalanceTunings();
	readPosTunings();
	readState();
	readIMUOffset();

	bController.setOffset(0, 0, 0, 0, 0, 0);
	bController.setTipLimit(150);    // limit of velocity component before tip
	// bController.enablePosCorFlip();  // flip coordinates

	//bController.disableBalance();
	//bController.disablePosCorrection();

	// default priority of 127. The lower the number, the higher the priority.
	intService.priority(127);  
	intService.begin(intServiceRoute, INT_UPDATE_INTERVAL);

	while(true){
		unsigned long now = micros();
		elapsedMicros elapsed = 0;
		int loopCountMod4 = loopCount % 4;

		mpuRead(); // takes long time
		fusionUpdate();  // takes long time
		
		if (loopCountMod4 == 0){
			elapsed = 0;
			fusionGetEuler(); // inverse trig functions take a long time

			encoderGetVelocity();  // need to first get velocity of wheels before getting pos.
			lCalculator.update();
			pos_x = -lCalculator.y;
			pos_y = -lCalculator.x;
			theta = lCalculator.theta;

			if ((bController.balanceEnabled() || bController.posCorEnabled()) && !calibMotorMode){
				blinkInterval = BLINK_OK_INTERVAL;
				if (!bController.update()){
					if (btDebug){
						btCtrl.println("---");
						btCtrl.println("Fell");
					}
					if (serDebug){
						Serial.println("---");
						Serial.println("Fell");
					}

					bController.disable();  // disable balancing
					blinkInterval = BLINK_STOPPED_INTERVAL;
				}
				pidUpdateCount++;
			}

			if (calibMotorMode){
				calibMotorModeRoutine();
			}
			else{
				omni.moveCartesian((int)v_x, (int)v_y, 0);
			}

			if ((bController.balanceEnabled() || bController.posCorEnabled()) && !calibMotorMode){
				if (btDebug && loopCount % 12 == 0){
					btCtrl.print(now);  										btCtrl.print('\t');
					// btCtrl.print(pidUpdateCount);								btCtrl.print('\t');
					btCtrl.print(roll);											btCtrl.print('\t');
					btCtrl.print(pitch);										btCtrl.print('\t');
					btCtrl.print(bController.e_theta_x * bController.kp_theta); btCtrl.print('\t');
					btCtrl.print(bController.e_theta_y * bController.kp_theta); btCtrl.print('\t');	
					btCtrl.print(bController.int_theta_x);  					btCtrl.print('\t');
					btCtrl.print(bController.int_theta_y);  					btCtrl.print('\t');
					btCtrl.print(bController.d_theta_x * bController.kd_theta); btCtrl.print('\t');
					btCtrl.print(bController.d_theta_y * bController.kd_theta); btCtrl.print('\t');

					btCtrl.print(bController.e_dtheta_x * bController.kp_dtheta); btCtrl.print('\t');
					btCtrl.print(bController.e_dtheta_y * bController.kp_dtheta); btCtrl.print('\t');	
					btCtrl.print(bController.int_dtheta_x);  					btCtrl.print('\t');
					btCtrl.print(bController.int_dtheta_y);  					btCtrl.print('\t');
					btCtrl.print(bController.d_dtheta_x * bController.kd_dtheta); btCtrl.print('\t');
					btCtrl.print(bController.d_dtheta_y * bController.kd_dtheta); btCtrl.print('\t');

					btCtrl.print(v_x);  										btCtrl.print('\t');
					btCtrl.print(v_y);											btCtrl.print('\t');
					btCtrl.print(pA);											btCtrl.print('\t');
					btCtrl.print(pB);											btCtrl.print('\t');
					btCtrl.print(pC);											btCtrl.print('\t');
					btCtrl.print(pos_x);  										btCtrl.print('\t');
					btCtrl.println(pos_y);
				}
				if (serDebug){
					Serial.print(now);  										Serial.print('\t');
					Serial.print(roll);											Serial.print('\t');
					Serial.print(pitch);										Serial.print('\t');
					Serial.print(bController.e_theta_x * bController.kp_theta);	Serial.print('\t');
					Serial.print(bController.e_theta_y * bController.kp_theta);	Serial.print('\t');	
					Serial.print(bController.int_theta_x);  					Serial.print('\t');
					Serial.print(bController.int_theta_y);  					Serial.print('\t');
					Serial.print(bController.d_theta_x * bController.kd_theta);	Serial.print('\t');
					Serial.print(bController.d_theta_y * bController.kd_theta);	Serial.print('\t');

					Serial.print(bController.e_dtheta_x * bController.kp_dtheta); Serial.print('\t');
					Serial.print(bController.e_dtheta_y * bController.kp_dtheta); Serial.print('\t');	
					Serial.print(bController.int_dtheta_x);  					Serial.print('\t');
					Serial.print(bController.int_dtheta_y);  					Serial.print('\t');
					Serial.print(bController.d_dtheta_x * bController.kd_dtheta); Serial.print('\t');
					Serial.print(bController.d_dtheta_y * bController.kd_dtheta); Serial.print('\t');
					
					Serial.print(v_x);  										Serial.print('\t');
					Serial.print(v_y);											Serial.print('\t');
					Serial.print(pA);											Serial.print('\t');
					Serial.print(pB);											Serial.print('\t');
					Serial.print(pC);											Serial.print('\t');
					Serial.print(pos_x);  										Serial.print('\t');
					Serial.println(pos_y);
				}
			}

		}
		else if (loopCountMod4 == 1){
			// motor velocity pid control
			if (!calibMotorMode){
				if (abs(pA) > 0){
					pA = pA + SIGN(pA) * motorMinPwr;
				}
				if (abs(pB) > 0){
					pB = pB + SIGN(pB) * motorMinPwr;
				}
				if (abs(pC) > 0){
					pC = pC + SIGN(pC) * motorMinPwr;
				}
			}
			motorA.move(pA);
			motorB.move(pB);
			motorC.move(pC);
		}

		serCtrl.update();
		serCtrl.send();
		btCtrl.update();
		btCtrl.send();
		// delay(1);
		loopCount++;
	}
}


// this routine is called every INT_UPDATE_INTERVAL microseconds
void intServiceRoute(){
	encoderPoll();  // poll encoders

	int intMod10 = intUpdateCount % 10;
	unsigned long blinkMod = intUpdateCount % (int)(blinkInterval*1000/INT_UPDATE_INTERVAL);
	// unsigned long now = micros();

	if (blinkMod == 0){
		digitalWriteFast(LED, !digitalReadFast(LED));
	}

	if (intMod10 == 0){
		switch(channelCount){
			case 0:				
				break;
			case 1:				
				break;
			case 2:				
				break;
			case 3:
				channelCount = -1;
				break;
		}
		channelCount++;
	}

	intUpdateCount++;
}



