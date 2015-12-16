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
void setup() {
	SPI.setSCK(SCK_PIN);    // change SCK pin from 13 to 14 since 13 also has a useful LED
	SPI.begin();
	Serial.begin(115200);   // doesn't really matter what we put here (Teensy 3.x serial isn't "real" serial)
	//while(!Serial.available()){};
	BT.begin(115200);  // RN42 default baud
	// delay(200);
	// BT.print("$$$");

 // 	delay(200);
 // 	thingy();
 	
 // 	BT.println("ST,1");   // MUST MAKE SURE RN42 GOES INTO FAST MODE
 // 	delay(200);
 // 	thingy();
 // 	BT.println("---");

	

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
		Serial.println("Successful connection");
	else
		Serial.print("Failed connection: ");
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

	bController.setTunings(
		10,       // kp for balancing (yaw/pitch)
		0,        // ki for balancing (yaw/pitch)
		0,        // kd for balancing (yaw/pitch)
		1,        // kp for position control
		0,        // ki for position control
		0         // kd for position control
	);

	bController.setOffset(0, 0, 0, 0);
	bController.enablePosCorFlip();  // flip coordinates

	//bController.disableBalance();
	//bController.disablePosCorrection();

	// default priority of 127. The lower the number, the higher the priority.
	intService.priority(127);  
	intService.begin(intServiceRoute, INT_UPDATE_INTERVAL);
}


void loop() {
	serialCtrl.update();
	btCtrl.update();
}

// this routine is called every INT_UPDATE_INTERVAL microseconds
void intServiceRoute(){
	encoderPoll();  // poll encoders

	int intMod10 = intUpdateCount % 10;
	unsigned long intMod1000 = intUpdateCount % 1000;

	if (intMod1000 == 0){
		digitalWriteFast(LED, !digitalReadFast(LED));
	}
	if (intMod10 == 0){
		switch(channelCount){
			case 0:
				wA = (rtA - prA) * 1000000 / INT_UPDATE_INTERVAL / 40 / CPRAD;
				vA = wA * WHEEL_R;
				tA += wA * INT_UPDATE_INTERVAL * 40 / 1000000;
				prA = rtA;	   // cleanup
				break;
			case 1:
				vB = rtB - prB;	 // get elapsed counts. Should not overflow/underflow.
				vB = vB * 1000000 / CPRAD / INT_UPDATE_INTERVAL / 40 * WHEEL_R;
				prB = rtB;	   // cleanup
				break;
			case 2:
				vC = rtC - prC;	 // get elapsed counts. Should not overflow/underflow.
				vC = vC * 1000000 / CPRAD / INT_UPDATE_INTERVAL / 40 * WHEEL_R;
				prC = rtC;	   // cleanup

				lCalculator.update();
				pos_x = -lCalculator.y;
				pos_y = -lCalculator.x;
				theta = lCalculator.theta;


				if (bController.balanceEnabled() || bController.posCorEnabled()){
					if (!bController.update()){
						if (btDebug){
							BT.println();
							BT.println("Fell");
						}

						motorA.move(0);
						motorB.move(0);
						motorC.move(0);

						bController.disable();  // disable balancing
					}
				}
				
				omni.moveCartesian((int)v_x, (int)v_y, 0);							
				break;
			case 3:
				if (bController.balanceEnabled()){
					unsigned long now = micros();
					if (btDebug){
						BT.print(now);  										BT.print('\t');
						BT.print(roll);											BT.print('\t');
						BT.print(pitch);										BT.print('\t');
						BT.print(bController.e_theta_x * bController.kp_theta); BT.print('\t');
						BT.print(bController.e_theta_y * bController.kp_theta); BT.print('\t');	
						BT.print(bController.int_theta_x);  					BT.print('\t');
						BT.print(bController.int_theta_y);  					BT.print('\t');
						BT.print(bController.d_theta_x * bController.kd_theta); BT.print('\t');
						BT.print(bController.d_theta_y * bController.kd_theta); BT.print('\t');
						BT.print(v_x);  										BT.print('\t');
						BT.print(v_y);  										BT.print('\n');
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
						Serial.print(v_x);  										Serial.print('\t');
						Serial.print(v_y);  										Serial.print('\n');
					}
				}
				
				channelCount = -1;
				break;
		}
		channelCount++;
	}
	else if (intMod10 == 1){
		mpuRead(); // takes long time
	}
	else if (intMod10 == 2){
		
	}
	else if (intMod10 == 3){
		
	}
	else if (intMod10 == 4){
		fusionUpdate();  // takes long time

	}
	else if (intMod10 == 5){
		fusionGetEuler(); // inverse trig functions take a long time
	}
	else if (intMod10 == 6){
		
	}
	else if (intMod10 == 7){
		
	}
	else if (intMod10 == 8){
		
	}
	else{
		// motor velocity pid control
		motorA.move(pA);
		motorB.move(pB);
		motorC.move(pC);
	}

	intUpdateCount++;
}



