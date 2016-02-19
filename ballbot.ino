/* Program for my ballbot, found at http://onewaytobalance.blogspot.com
 * 
 * Configurations, settings and global variables/objects found in ballbot.h
 *
 * Only runs on Teensy 3.x and requires a decent clock (I use 96MHz)
 * This is because it is quite processing intensive since the single MCU does
 *    - on board Mahony sensor fusion at >1000Hz using 9dof data from the MPU9250
 *          --> converting quaternions to Euler angles unfortunately requires CPU intensive 
 *                inverse trigonometry
 *    - PID roll/pitch control in 2 dimensions (i.e. two PID control loops for balancing)
 *    - Polling encoders at a high sample rate (64cpr encoders and a 12400rpm motor - no load)
 *            --> Finding velocity of each of the 3 wheels
 *    - Using wheel velocities and trig to obtain absolate location/position of robot
 *          see http://onewaytobalance.blogspot.com.au/2015/12/omnidirectional-control.html
 *          The maths requires you to have a basic understanding of linear algebra (matrices etc.)
 *    - PID control for position of robot
 *    - Debugging via bluetooth and serial to computer (the RN42 is quite slow)
 *    
 * TODO
 *    - Improve magnetometer calibration routine
 *    - Implemented tilt compensated bearing calculations with magnetometer
 *    - Pass tilt compensated bearing to position calculator
 *    - Implement yaw correction
 *    - 
 *    
 * (C) Brian Chen 2015
 */

#include <SPI.h>               // requires SPI library with transactions
#include <EEPROM.h>
#include <FrequencyTimer2.h>   // Built into teensy core
#include <PID_v1.h>

/* own libraries */
#include <EEPROMAnything.h>    // see https://github.com/brianchen118/Team-PI-Lib/tree/master/EEPROMAnything
#include <MPU9250.h>           // see https://github.com/brianchen118/MPU9250
#include <pwmMotor.h>          // see https://github.com/brianchen118/Team-PI-Lib/tree/master/pwmMotor
#include <fastTrig.h>          // https://github.com/brianchen118/fastTrig
#include <omnidrive.h>         // see https://github.com/brianchen118/omnidrive
#include <sensorfusion.h>      // see https://github.com/brianchen118/sensorfusion
#include "balanceController.h"
#include "locationCalculator.h"
#include "macros.h"
#include "bt.h"
#include "ballbot.h"

#define POS_LPF_T 200.0f
elapsedMicros posLPF_dt;

IntervalTimer intService;

/* Main part of the program begins here.
 *
 */

extern "C" int main () {
    SPI.setSCK(SCK_PIN);    // change SCK pin from 13 to 14 since 13 also has a useful LED
    SPI.begin();

    Serial.begin(115200);   // doesn't really matter what we put here (Teensy 3.x serial isn't "real" serial)

    pinMode(INT_PIN, INPUT);
    pinMode(LED, OUTPUT);     digitalWriteFast(LED, HIGH);   // initial set LEDs on high
    pinMode(LED2, OUTPUT);    digitalWriteFast(LED2, HIGH);

    btSetup(HC06, 921600, BAUD_1382400);    
    // btSetup(HM11, 9600, BAUD_230400);    
    // while(bluetoothEcho()){};

    // use a higher frequency of >12kHz to reduce audible sound and reduce switching losses
    motorA.begin(PWM_FREQ);  
    motorB.begin(PWM_FREQ);
    motorC.begin(PWM_FREQ);

    pidMtrA.SetOutputLimits(-2550000, 2550000);
    pidMtrB.SetOutputLimits(-2550000, 2550000);
    pidMtrC.SetOutputLimits(-2550000, 2550000);

    pidMtrA.SetSampleTime(1);
    pidMtrB.SetSampleTime(1);
    pidMtrC.SetSampleTime(1);

    pidMtrA.SetMode(AUTOMATIC);
    pidMtrB.SetMode(AUTOMATIC);
    pidMtrC.SetMode(AUTOMATIC);

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
    readMotorMinPwr();

    mpuRead();       // takes long time
    fusionUpdate();  // takes long time
    fusionGetEuler();
    zeroImu();

    bController.setOffset(0, 0, 0, 0, 0, 0);
    bController.setTipLimit(255);    // limit of velocity component before tip
    bController.enablePosCorFlip();  // flip coordinates

    // default priority of 127. The lower the number, the higher the priority.
    intService.priority(127);  
    intService.begin(intServiceRoute, INT_UPDATE_INTERVAL);

    while(true){
        now = micros();
        int loopCountMod4 = loopCount % 4;

        mpuRead(); // takes long time
        fusionUpdate();  // takes long time
        
        if (loopCountMod4 == 0){
            fusionGetEuler(); // inverse trig functions take a long time

            encoderGetVelocity();  // need to first get velocity of wheels before getting pos.
            lCalculator.update();
            // pos_x = -lCalculator.y;
            // pos_y = -lCalculator.x;

            // low pass filter implementation on position (smooths out derivative term)
            unsigned long dt = posLPF_dt;
            posLPF_dt = 0;
            float aa = POS_LPF_T/(float)(POS_LPF_T + dt);
            pos_x = aa * (-lCalculator.y) + (1 - aa) * pos_x;
            pos_y = aa * (-lCalculator.x) + (1 - aa) * pos_y;

            theta = lCalculator.theta;

            if ((bController.balanceEnabled() || bController.posCorEnabled()) && !calibMotorMode){
                blinkInterval = BLINK_OK_INTERVAL;
                if (!bController.update() || abs(pA) > 255 || abs(pB) > 255 || abs(pC) > 255){
                    if (btDebug){
                        btCtrl.println("---");
                        btCtrl.println("Fell");
                        btCtrl.debug();
                        btCtrl.println("---");
                    }
                    if (serDebug){
                        serCtrl.println("---");
                        serCtrl.println("Fell");
                        serCtrl.debug();
                        serCtrl.println("---");
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

                if (motorPidControl){
                    vA_targ *= 10;
                    vB_targ *= 10;
                    vC_targ *= 10;  
                }                
            }

            if ((bController.balanceEnabled() || bController.posCorEnabled()) && !calibMotorMode){
                if (btDebug && loopCount % 1 == 0){
                    btCtrl.debug();
                }
                if (serDebug && loopCount % 1 == 0){
                    serCtrl.debug();
                }
            }

        }
        else if (loopCountMod4 == 1){
            // motor velocity pid control
            if (motorPidControl){              

                pidMtrA.Compute();
                pidMtrB.Compute();
                pidMtrC.Compute();

                unsigned long elapsed = pidMtrdt;
                pidMtrdt = 0;

                pA_f += pidOutA * elapsed / 1000000;
                pB_f += pidOutB * elapsed / 1000000;
                pC_f += pidOutC * elapsed / 1000000;

                pA = (int)pA_f;
                pB = (int)pB_f;
                pC = (int)pC_f;

                if (abs(pA) > 255) pA = SIGN(pA) * 255;
                if (abs(pB) > 255) pB = SIGN(pB) * 255;
                if (abs(pC) > 255) pC = SIGN(pC) * 255;
            }
            else{
                pA = (int)vA_targ;
                pB = (int)vB_targ;
                pC = (int)vC_targ;

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
            }
            
            pA *= motorPwrScalar;
            pB *= motorPwrScalar;
            pC *= motorPwrScalar;

            motorA.move(pA);
            motorB.move(pB);
            motorC.move(pC);
        }
        
        serCtrl.update();  // reads incoming data
        serCtrl.send();    // sends outgoing data
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

    // blinking
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

// #define BT Serial1

// #define BAUD_9600 0
// #define BAUD_115200 1
// #define BAUD_230400 2
// #define BAUD_460800 3
// #define BAUD_921600 4
// #define BAUD_1382400 5

// #define RN42 0
// #define HC06 1
// #define HM11 2

// void btSetup(uint8_t module, int initBaud, int finalBaud){
//     if (module == RN42){

//     }
//     else if (module == HC06){
//         BT.begin(initBaud);
//         delay(100);
//         Serial.print("AT");
//         BT.println("AT");

//         delay(1000);

//         switch (finalBaud){
//             case BAUD_9600:    BT.print("AT+BAUD4"); delay(500); BT.begin(9600);   break;
//             case BAUD_115200:  BT.print("AT+BAUD8"); delay(500); BT.begin(115200); break;
//             case BAUD_230400:  BT.print("AT+BAUD9"); delay(500); BT.begin(230400); break;
//             case BAUD_460800:  BT.print("AT+BAUDA"); delay(500); BT.begin(460800); break;
//             case BAUD_921600:  BT.print("AT+BAUDB"); delay(500); BT.begin(921600); break;
//             case BAUD_1382400: BT.print("AT+BAUDC"); delay(500); BT.begin(1382400); break;
//         }
//     }
//     else if (module == HM11){
//         BT.begin(initBaud);
//         delay(100);
//         Serial.print("AT");
//         BT.print("AT");

//         delay(1000);

//         switch (finalBaud){
//             case BAUD_9600:    BT.print("AT+BAUD0"); delay(500); BT.begin(9600);   break;
//             case BAUD_115200:  BT.print("AT+BAUD4"); delay(500); BT.begin(115200); break;
//             case BAUD_230400:  BT.print("AT+BAUD8"); delay(500); BT.begin(230400); break;
//             // case BAUD_460800:  BT.print("AT+BAUDA"); delay(500); BT.begin(460800); break;
//             // case BAUD_921600:  BT.print("AT+BAUDB"); delay(500); BT.begin(921600); break;
//             // case BAUD_1382400: BT.print("AT+BAUDC"); delay(500); BT.begin(1382400); break;
//             default: break;
//         }

//     }
// }

// void setup()  
// {

//   // Open serial communications and wait for port to open:
//   Serial.begin(9600);
//   BT.begin(9600);

//   delay(1000);

//   btSetup(HM11, 230400, BAUD_9600);  

//   delay(1000);

//   Serial.println("Goodnight moon!");

//   // set the data rate for the SoftwareSerial port
//   BT.begin(9600);
//   // set slave
//   //BT.print("AT+ROLE0"); 
//   delay(2000);

 
// }

// void loop() // run over and over
// {
//   char c;
//     // set the data rate for the SoftwareSerial port
//   // BT.print("test I am slave ");
//   // delay(2000);  
//   if (BT.available()){
//     c = BT.read();
//     Serial.write(c);
//   }
//   if (Serial.available()){
//     c = Serial.read();
//     BT.write(c);
//   }
    
//     //Serial.write(c);
// }