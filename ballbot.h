
#define BT Serial1

#define SPI_CLOCK 8000000

#define SCK_PIN  14
#define SS_PIN   10 
#define INT_PIN  3
#define LED      13
#define LED2     28

#define PWM_FREQ  16000

#define MT_A_DIR  4
#define MT_A_PWM  5
//#define MT_A_PWM  4
#define MT_A_BRK  16
#define MT_A_FLIP false
#define MT_A_CS   A10

#define MT_B_DIR  9
#define MT_B_PWM  23
#define MT_B_BRK  7
#define MT_B_FLIP false
#define MT_B_CS   A14

#define MT_C_DIR  8
#define MT_C_PWM  6
//#define MT_C_PWM  25
#define MT_C_BRK  22
#define MT_C_FLIP false
#define MT_C_CS   A11

#define INT_UPDATE_INTERVAL 75

// #define ENCXA	   21
// #define ENCXB	   17
// #define ENCYA	   3
// #define ENCYB	   2
// #define ENCZA	   20
// #define ENCZB	   15

#define ENCXA	   20
#define ENCXB	   15
#define ENCYA	   21
#define ENCYB	   17
#define ENCZA	   3
#define ENCZB	   2

#define WHEEL_R    24
#define CHASSIS_R  88
#define CPR        64  // though effectively 256
#define GEAR_RATIO 16
#define MAX_RPM    12400
#define CPRAD      651.898644   // 4x counts per radian

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  \

/* macros to shift array. Parameters are inclusive. For example:
   int array[] =                  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
   ARRAYSHIFTUP(array, 2, 6)   => { 1, 2, 3, 3, 4, 5, 6, 7, 9, 10 };
   ARRAYSHIFTDOWN(array, 2, 6) => { 1, 3, 4, 5, 6, 7, 7, 8, 9, 10 }; */

#define ARRAYSHIFTUP(a, lower, upper){            \
    if (lower == 0){                              \
        for (int q = lower; q < upper; q++){      \
            *(a + q) = *(a + q + 1); }            \
    } else{                                       \
        for (int q = lower - 1; q < upper; q++){  \
            *(a + q) = *(a + q + 1); }}}          \

#define ARRAYSHIFTDOWN(a, lower, upper){          \
    if (upper == (sizeof(a)/sizeof(a[0])) - 1){   \
        for (int q = upper - 1; q >= lower; q--){ \
            *(a + q + 1) = *(a + q); }            \
    } else{                                       \
        for (int q = upper; q >= lower; q--){     \
            *(a + q + 1) = *(a + q); }}}          \

#define ARRAYAVERAGE(a, out){                         \
    int average = 0;                                  \
    for (int i = 0; i < sizeof(a)/sizeof(a[0]); i++){ \
    	average += a[i];}                             \
    out = average/(sizeof(a)/sizeof(a[0]));}          \

#define CLEARARRAY(a){                                \
	for (int q = 0; q < sizeof(a)/sizeof(a[0]); q++){ \
		a[q] = 0; }}                                  \
		

#define CLEARSERIAL(){                  \
	while (Serial.available()){         \
		Serial.read();	}}              \

#define PRINTARRAY(a){                                \
	Serial.print('{');                                \
	for (int i = 0; i < sizeof(a)/sizeof(a[0]); i++){ \
		Serial.print(a[i]);                           \
		Serial.print('\t'); }                         \
	Serial.println('}'); }                            \


MPU9250 mpu(SPI_CLOCK, SS_PIN);

int16_t mag_max[3];
int16_t mag_min[3];

float roll, pitch, yaw;

float mag_scalar[3], mag_bias[3];

PMOTOR motorA(MT_A_PWM, MT_A_DIR, MT_A_BRK, MT_A_FLIP, MT_A_CS);
PMOTOR motorB(MT_B_PWM, MT_B_DIR, MT_B_BRK, MT_B_FLIP, MT_B_CS);
PMOTOR motorC(MT_C_PWM, MT_C_DIR, MT_C_BRK, MT_C_FLIP, MT_C_CS);

int32_t pA, pB, pC;

omnidrive omni(&pA, &pB, &pC);

IntervalTimer intService;
/* since there are 4 phases a quadrature encoder goes through, there are 16 combinations, many of which
   are invalid (and hence = 0) */
int32_t enc_TAB[]= { 0, 1, -1, 0,
 					-1, 0,  0, 1,
 					 1, 0,  0,-1,
 					 0,-1,  1, 0 };

volatile uint32_t encState = 0, lEncState = 0;
volatile uint32_t encIndex;

volatile int32_t channelCount = 0, intUpdateCount = 0;

volatile int32_t rtA = 0, rtB = 0, rtC = 0, rtD = 0;	// Realtime values
volatile int32_t prA = 0, prB = 0, prC = 0, prD = 0;	// Previous values

float vA = 0, vB = 0, vC = 0, vD = 0;	// wheel velocity (m/s)
float wA = 0, wB = 0, wC = 0, wD = 0;   // angular velocity (rad/s)
float tA = 0, tB = 0, tC = 0, tD = 0;   // wheel angle

volatile float vA_targ, vB_targ, vC_targ, vD_targ; // target angular velocity

elapsedMicros readIMUdt;
elapsedMicros readMagDt;

float v_x, v_y;
float pos_x, pos_y;
float theta;

BalanceController bController(&v_x, &v_y, &pos_x, &pos_y, &theta, &roll, &pitch);
LocationCalculator lCalculator(CHASSIS_R, 180, 60, 300, &vA, &vB, &vC);

void calibMagRoutine();
void calcMagCalib();
void storeMagCalib();
void readMagCalib();
void mpuRead();
void fusionUpdate();
void fusionGetEuler();
inline void encoderPoll();
bool bluetoothEcho();

bool btDebug = true;
bool serDebug = true;

// controller for serial interface
class SerialController{
public:
	SerialController(Stream& _port): port(_port) {}
	void update(){
		if (ready){
	        ready = false;

	        char command = buffer[0];
	        buffer[0] = ',';

	        switch(command){
	        	case 'c':
	        		{
	        		port.println("Starting magnetometer calibration");
	        		bController.disable();
	        		calibMagRoutine();
	        		port.println("Finished calibration");
	        		storeMagCalib();
	        		port.println("Stored calibration");
	        		bController.enable();
	        		}
	        		break;
			    case 's':	// set PID parameters
			    	{
			    	bController.disable();
			    	float my_array[7];
					int i = 0;
					char * tok = strtok(buffer, ",");
					while (tok != 0 && i < 7) {
					    my_array[i] = atof(tok);
					    tok = strtok(0, ",");
					    port.println(my_array[i]);
					    i++;
					}
					if (i != 6){
						port.println("Incorrect number of inputs:");
						port.println("\t6 comma separated values required");
					}
					else{
						bController.setTunings(my_array[0], my_array[1], my_array[2],
										   	   my_array[3], my_array[4], my_array[5]);
					}
					bController.enable();
					}
			    	break;
			    case 'e':
			    	{
			    	port.println("Enabled bController");
			    	bController.enable();
			    	}
					break;
				case 'd':
					{
					port.println("Disabled bController");
					bController.disable();
					}
					break;
				case 'p':
					btDebug = false;
					serDebug = false;
					break;
				case 'r':
					btDebug = true;
					serDebug = true;
					break;
			    default: break;
			}
	    } 
	    else{
	        while (port.available()){
	            char c = port.read();
	            buffer[cnt++] = c;
	            if ((c == '\n') || (cnt == sizeof(buffer)-1)){
	                buffer[cnt] = '\0';
	                cnt = 0;
	                ready = true;
	            }
	        }
	    }
	}
private:
	Stream &port;
	char buffer [32];
	int cnt = 0;
	bool ready = false;
};

SerialController btCtrl(BT);
SerialController serialCtrl(Serial);

// class SerialServo
// {
//   Stream & port_; // http://forum.arduino.cc/index.php?topic=190210.0#lastPost  Reply #14
  
//   public:
//   // constructor
//     SerialServo (Stream & port) : port_ (port) { }
  
//   // methods
//     void Number_of_Channels(uint8_t channel = 4);
//     void Move(uint8_t channel, uint8_t position);
//  void AdjustLeftRight(uint8_t L, uint8_t R);
//  void Calibrate(){
//  	port_.print("hi");
//  }

//   private:
//     uint8_t left, right;
//     unsigned int ServoNum[35];
// };

// SerialServo ss (Serial);


/* Magnetometer calibration stuff
 *   Need to calibrate the AK8963 for hard and soft iron distortion
 *   A simple method is implemented using the max/min readings of
 *   each axis
 */

void calibMagRoutine(){
	noInterrupts();

	uint16_t ii = 0, sample_count = 10000; // (100 seconds)

	mag_max[0] = 0xFFFF;    mag_min[0] = 0x7FFF;
	mag_max[1] = 0xFFFF;    mag_min[1] = 0x7FFF;
	mag_max[2] = 0xFFFF;    mag_min[2] = 0x7FFF;	

	for(ii = 0; ii < sample_count; ii++) {
		digitalWriteFast(LED, !digitalReadFast(LED));
		mpu.read_mag();

		for (int jj = 0; jj < 3; jj++) {
			if(mpu.mag_data_raw[jj] > mag_max[jj]) mag_max[jj] = mpu.mag_data_raw[jj];
			if(mpu.mag_data_raw[jj] < mag_min[jj]) mag_min[jj] = mpu.mag_data_raw[jj];
		}
		delay(10);  // AK8963 mode 2 => 100Hz
		if (Serial.available()) break;
	}
	CLEARSERIAL();

	calcMagCalib();

	interrupts();
}

void calcMagCalib(){
	float mag_scale[3] = {0, 0, 0};

	// Get hard iron correction
	mag_bias[0]  = mpu.Magnetometer_ASA[0] * (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = mpu.Magnetometer_ASA[1] * (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = mpu.Magnetometer_ASA[2] * (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	// Get soft iron correction estimate
	mag_scale[0]  = mpu.Magnetometer_ASA[0] * (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = mpu.Magnetometer_ASA[1] * (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = mpu.Magnetometer_ASA[2] * (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	mag_scalar[0] = avg_rad/((float)mag_scale[0]);
	mag_scalar[1] = avg_rad/((float)mag_scale[1]);
	mag_scalar[2] = avg_rad/((float)mag_scale[2]);
}

void storeMagCalib(){
	int i = 0;
	for (; i < 3; i++){
		EEPROM_writeAnything(100 + 4*i, mag_max[i]);
	}
	for (; i < 6; i++){
		EEPROM_writeAnything(100 + 4*i, mag_min[i-3]);
	}
}

void readMagCalib(){
	int i = 0;
	for (; i < 3; i++){
		EEPROM_readAnything(100 + 4*i, mag_max[i]);
	}
	for (; i < 6; i++){
		EEPROM_readAnything(100 + 4*i, mag_min[i-3]);
	}
	calcMagCalib();

	Serial.println("Read Mag Calib");
	Serial.print("mag_bias ");   PRINTARRAY(mag_bias);
	Serial.print("mag_scalar "); PRINTARRAY(mag_scalar);
}


/* Read IMU
 *
 */

void mpuRead(){
	// only read mag at 100Hz
	if (readMagDt > 10000){
		readMagDt = 0;
		mpu.read_all();
	}
	else{
		mpu.read_gyro();
		mpu.read_acc();
	}

	mpu.mag_data[0] = (mpu.mag_data[0] - mag_bias[0]) * mag_scalar[0];
	mpu.mag_data[1] = (mpu.mag_data[1] - mag_bias[1]) * mag_scalar[1];
	mpu.mag_data[2] = (mpu.mag_data[2] - mag_bias[2]) * mag_scalar[2];
}

/* Read Sensor fusion update
 *
 */

void fusionUpdate(){
	float readIMUdt_s = readIMUdt/1000000.0f;
	readIMUdt = 0;

	sensfusionUpdateQ(mpu.gyro_data[0],
					  mpu.gyro_data[1],
					  mpu.gyro_data[2],
					  mpu.accel_data[0],
					  mpu.accel_data[1],
					  mpu.accel_data[2],
					  // mpu.mag_data[1],
					  // mpu.mag_data[0],
					  // mpu.mag_data[2],
					  readIMUdt_s);
}

void fusionGetEuler(){
	sensfusionGetEulerRPY(&roll, &pitch, &yaw);
	roll += 0;
	pitch += 4.7;
}

/* Poll encoders
 *
 */

inline void encoderPoll(){
	encState = (digitalReadFast(ENCXB)<<1) | (digitalReadFast(ENCXA))	 
			 | (digitalReadFast(ENCYB)<<5) | (digitalReadFast(ENCYA)<<4) 
			 | (digitalReadFast(ENCZB)<<9) | (digitalReadFast(ENCZA)<<8);

	encIndex = encState | (lEncState << 2);

	/* byte   4	3	2	1
			  lB   lA   B	A */


	// if anything the following 4 lines are the best to optimise
	rtA += enc_TAB[ (encIndex >> 0)  & 0x0f]; // we are only using 4 bytes per encoder
	rtB += enc_TAB[ (encIndex >> 4)  & 0x0f];
	rtC += enc_TAB[ (encIndex >> 8)  & 0x0f];

	lEncState = encState;
}


bool bluetoothEcho(){
	char c;
	if(BT.available()){
		Serial.print((char)BT.read());  
	}
	if(Serial.available()){
		c = Serial.read();
		if(c == '~') return false;
		BT.print(c);
	}
	return true;
}