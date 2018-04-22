/*
  ESC_Ramp
    Based on the Sweep example in the Servo library, this allow a ramp up and down of the ESC speed based on the Min and Max values.
    The Min and Max speed are defined so you can change them at one location and it will be used in the ramp as well.
    
  27 April 2017
  by Eric Nantel
 */
#include "ESC.h"

#include <Wire.h>
#include <MPU9255.h>// include MPU9255 library
#include <Adafruit_Sensor.h>
//#include <utility/imumaths.h> 
#include <PID_v1.h>


#define WIFI_DISCONNECT_DO_TIMEOUT true             // setting to disable automatic timeout
#define WIFI_DISCONNECT_TIMEOUT 3000000             // If we haven't heard from wifi in 3 seconds, bail!


#define LED_PIN (LED_BUILTIN)                                      // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MIN_SPIN (1050)                                  // Set the Minimum Speed to spin the motors
#define MAX_DECELERATION 0.1
#define MAX_ACCELERATION 0.5

#define PID_THRESHOLD 1000                                //the limit at which to start the PID control loop
#define PID_SAMPLE_RATE 5

/* Must be less than Speed max so we have room to balance */
#define MAX_TOTAL_THRUST 1800



/* Joystick values from controller */
int JOYSTICK_LX = 512;
int JOYSTICK_LY = 512;
int JOYSTICK_RX = 512;
int JOYSTICK_RY = 512;


/* Target values for roll, pitch, and yaw */
double TARGET_ROLL = 0.0;
double TARGET_PITCH = 0.0;
double TARGET_YAW = 0.0;  // Unused for now


double roll;
double roll_feedback;


#define MAX_ROLL_DEGREES 50  // if it goes beyond 40 degrees, shutdown 

double ROLL_P = 2.0;
double ROLL_I = 3.0;
double ROLL_D = 0.35;
double ROLL_MAX = 400;

//PID rollPID(&roll, &roll_feedback, &TARGET_ROLL, ROLL_P, ROLL_I, ROLL_D, P_ON_M, DIRECT);
PID rollPID(&roll, &roll_feedback, &TARGET_ROLL, ROLL_P, ROLL_I, ROLL_D, DIRECT);

double pitch;
double pitch_feedback;

#define MAX_PITCH_DEGREES 40  // if it goes beyond 40 degrees, shutdown 

double PITCH_P = 2.0;
double PITCH_I = 3.0;
double PITCH_D = 0.35;
double PITCH_MAX = 400;

//PID pitchPID(&pitch, &pitch_feedback, &TARGET_PITCH, PITCH_P, PITCH_I, PITCH_D, P_ON_M, DIRECT);
PID pitchPID(&pitch, &pitch_feedback, &TARGET_PITCH, PITCH_P, PITCH_I, PITCH_D, DIRECT);


//Motors with silver tops rotate counter-clockwise

double yaw;
double yaw_feedback;

//#define MAX_YAW_DEGREES 40  // if it goes beyond 40 degrees, shutdown 

double YAW_P = 1.0;
double YAW_I = 0.2;
double YAW_D = 0.0;
double YAW_MAX = 400;
PID yawPID(&yaw, &yaw_feedback, &TARGET_YAW, YAW_P, YAW_I, YAW_D, DIRECT);



long LAST_READ_FROM_WIFI; /* Keeps track of the last time we got something from the Wifi chip */


/* The base thrust value for altitude hold */
double MOTOR_FL_BASE = SPEED_MIN_SPIN;
double MOTOR_FR_BASE = SPEED_MIN_SPIN;
double MOTOR_BL_BASE = SPEED_MIN_SPIN;
double MOTOR_BR_BASE = SPEED_MIN_SPIN;

/* The balance value */
double MOTOR_FL_BAL = 0;
double MOTOR_FR_BAL = 0;
double MOTOR_BL_BAL = 0;
double MOTOR_BR_BAL = 0;

/* The current motor values */
double FL_VAL;
double FR_VAL;
double BL_VAL;
double BR_VAL;

/* The Target motor values */
double TARGET_FL_VAL;
double TARGET_FR_VAL;
double TARGET_BL_VAL;
double TARGET_BR_VAL;


double ALTITUDE = 0.0;
double TARGET_ALTITUDE = 19;

volatile long ECHO_START = 0;                         // Records start of echo pulse 
volatile long ECHO_END = 0;                           // Records end of echo pulse
volatile long ECHO_DURATION = 0;                      // Duration - difference between end and start
volatile int TRIGGER_TIME_COUNT = 0;                  // Count down counter to trigger pulse time

bool WAITFORSTART = false;


ESC ESC_BL (9, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_BR (10, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_FR (11, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_FL (12, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)


void(* resetFunc) (void) = 0; //declare reset function @ address 0

/* Calibrate ESCs, not called regularly */
void calibrateAll(){
	ESC_FL.calib();
	ESC_FL.stop();

	ESC_FR.calib();
	ESC_FR.stop();

	ESC_BL.calib();
	ESC_BL.stop();

	ESC_BR.calib();  
	ESC_BR.stop(); 
  
}



/* initalize push button for start */
void StartButtonSetup(void){
	Serial.print("Setting up push button...");
	pinMode(2, INPUT);
	attachInterrupt(digitalPinToInterrupt(2), startButtonPushed, FALLING);
	Serial.println("Done!");
}

/* Register a push button press */
void startButtonPushed(void){
	Serial.println("Start Button Pressed!!!");
	WAITFORSTART = true;
}



/* Look for "START" from WiFi module */
void checkWifiForStart(){
  String input;
  if(Serial1.available()){
    input = Serial1.readStringUntil('\n');
    if (input.startsWith("START")){
      WAITFORSTART = true;
    }
  }
}

/* Read the joystick throttle values from controller */
String input;


/* Expects string in format 
 *  "base_thrust base_thrust"
 *  both must be floats, and must be the same value twice to prevent bit flips
 */
void readSpeedFromWifi(){
  if(Serial1.available()){
      input = Serial1.readStringUntil('\n');
      //Serial.println(input);
      //int base_thrust = Serial1.parseInt();
      //int chksum = Serial1.parseInt();
      int spaceIndex = input.indexOf(' ', 0);
      String substr = input.substring(0, spaceIndex);
      int base_thrust = substr.toInt();
      int chksum = input.substring(spaceIndex + 1, input.length()).toInt();
      
//      Serial.print("Base thrust:");
//      Serial.println(base_thrust);
      /* Chksum should be the same value and make sure valid esc value */
      if (base_thrust != chksum || base_thrust < SPEED_MIN || base_thrust > SPEED_MAX){
        LAST_READ_FROM_WIFI = micros();  
        return;
      }

      //Serial.println(thrust);
      MOTOR_FL_BASE = base_thrust;
      MOTOR_FR_BASE = base_thrust;
      MOTOR_BL_BASE = base_thrust;
      MOTOR_BR_BASE = base_thrust;

      LAST_READ_FROM_WIFI = micros();  

  }
 else{
    if (WIFI_DISCONNECT_DO_TIMEOUT){
      if (micros() - LAST_READ_FROM_WIFI > WIFI_DISCONNECT_TIMEOUT){
        Serial.println("Lost connection");
        slowMotorsToStop();
        MOTOR_FL_BASE = SPEED_MIN;
        MOTOR_FR_BASE = SPEED_MIN;
        MOTOR_BL_BASE = SPEED_MIN;
        MOTOR_BR_BASE = SPEED_MIN;
      }
    }
 }
//  else{
//    Serial.println("It wasn't there....\n");
//  }
}


void readSpeedFromWifiSlow(){
	if(Serial1.available()){
		input = Serial1.readStringUntil('\n');
//		Serial.print("-");
//		Serial.println(input);


		/* Parse the joystick values */
		/**
		*  Format is
		*  "LX=100 LY=200 RX=300 RY=400\n"
		*/
		int startIndex = 0;
		int spaceIndex = 0;
    int lx, ly, rx, ry, checksum;
		String substr;
		String value;
		if (input.startsWith("LX=")){

			spaceIndex = input.indexOf(' ', startIndex);
			substr = input.substring(startIndex, spaceIndex); 
      if (substr.startsWith("LX=")){
        value = substr.substring(3, substr.length()); 
        lx = value.toInt();
      }
			startIndex = spaceIndex + 1;

			spaceIndex = input.indexOf(' ', startIndex);
			substr = input.substring(startIndex, spaceIndex);
      if (substr.startsWith("LY")){
  			value = substr.substring(3, substr.length()); 
  			ly = value.toInt();
      }
			startIndex = spaceIndex + 1; 

			spaceIndex = input.indexOf(' ', startIndex); 
			substr = input.substring(startIndex, spaceIndex);
      if (substr.startsWith("RX")){
  			value = substr.substring(3, substr.length()); 
  			rx = value.toInt();
      }
			startIndex = spaceIndex + 1; 

      spaceIndex = input.indexOf(' ', startIndex);
			substr = input.substring(startIndex, spaceIndex);
      if (substr.startsWith("RY")){
  			value = substr.substring(3, substr.length()); 
  			ry = value.toInt();
      }
      startIndex = spaceIndex + 1; 

      
      spaceIndex = input.indexOf(' ', startIndex);
      substr = input.substring(startIndex, spaceIndex);
      /* Must include a checksum */
      if (!substr.startsWith("CK")){
        return;
      }
      value = substr.substring(3, substr.length()); 
      checksum = value.toInt();

      /* make sure our checksum matches */
      if (checksum == (lx + ly + rx + ry) % 256){
        JOYSTICK_LX = lx;
        JOYSTICK_LY = ly;
        JOYSTICK_RX = rx;
        JOYSTICK_RY = ry;
      }

      
//			Serial.print("LX=");
//			Serial.print(JOYSTICK_LX);
//			Serial.print(" LY=");
//			Serial.print(JOYSTICK_LY);
//			Serial.print(" RX=");
//			Serial.print(JOYSTICK_RX);
//			Serial.print(" RY=");
//			Serial.println(JOYSTICK_RY);
      LAST_READ_FROM_WIFI = micros();  
		}
	}
 else{
    if (WIFI_DISCONNECT_DO_TIMEOUT){
      if (micros() - LAST_READ_FROM_WIFI > WIFI_DISCONNECT_TIMEOUT){
        Serial.println("Lost connection");
        slowMotorsToStop();
        JOYSTICK_LX = 512;
        JOYSTICK_LY = 512;
        JOYSTICK_RX = 512;
        JOYSTICK_RY = 512;
      }
    }
 }
//	else{
//		Serial.println("It wasn't there....\n");
//	}
}


void ESCArmAll(){
	Serial.print("Arming ESCs...");
	ESC_FL.arm();                                            // Send the Arm value so the ESC will be ready to take commands
	ESC_FR.arm();                                            // Send the Arm value so the ESC will  be ready to take commands
	ESC_BL.arm();                                            // Send the Arm value so the ESC will be ready to take commands
	ESC_BR.arm();                                            // Send the Arm value so the ESC will be ready to take commands
	delay(3000);                                            // Wait for a while

	Serial.println("Done!");

}


void startMotors(void){
	Serial.println("Taking off!");
	ESC_FL.speed(SPEED_MIN);
	ESC_FR.speed(SPEED_MIN);
	ESC_BL.speed(SPEED_MIN);
	ESC_BR.speed(SPEED_MIN);
	delay(3000);

}

void slowMotorsToStop(void){
  while(FL_VAL > SPEED_MIN || FR_VAL > SPEED_MIN || BL_VAL > SPEED_MIN || BR_VAL > SPEED_MIN){
        ESC_FL.speed(FL_VAL);                                    // tell ESC to go to the oESC speed value
        ESC_FR.speed(FR_VAL);                                    // tell ESC to go to the oESC speed value
        ESC_BL.speed(BL_VAL);                                    // tell ESC to go to the oESC speed value
        ESC_BR.speed(BR_VAL);                                    // tell ESC to go to the oESC speed value
        delay(1);                                            // waits 10ms for the ESC to reach speed
        FL_VAL -= MAX_DECELERATION;
        FR_VAL -= MAX_DECELERATION;
        BL_VAL -= MAX_DECELERATION;
        BR_VAL -= MAX_DECELERATION;
        FL_VAL = constrain(FL_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
        FR_VAL = constrain(FR_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
        BL_VAL = constrain(BL_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
        BR_VAL = constrain(BR_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
  }
}

/* Checks for new set points over Serial0 */
void checkSerial0(void){
	if(Serial.available()){
		String input = Serial.readStringUntil('\n');
		if (input.startsWith("reset")){
			slowMotorsToStop();
			resetFunc();
		}
		String substr;
		substr = input.substring(3, input.length());
		//Serial.println(substr);

		if(input.startsWith("RP=")){
			ROLL_P = substr.toFloat();			
		}
		else if(input.startsWith("RI=")){
			ROLL_I = substr.toFloat();			
		}
		else if(input.startsWith("RD=")){
			ROLL_D = substr.toFloat();			
		}
		else if(input.startsWith("RT=")){
			TARGET_ROLL = substr.toInt();      
		}
		else if(input.startsWith("PP=")){
			PITCH_P = substr.toFloat();			
		}
		else if(input.startsWith("PI=")){
			PITCH_I = substr.toFloat();			
		}
		else if(input.startsWith("PD=")){
			PITCH_D = substr.toFloat();			
		}
		else if(input.startsWith("PT=")){
			TARGET_PITCH = substr.toInt();      
		}
		else if(input.startsWith("BT=")){
			int val = substr.toInt();     
			MOTOR_FL_BASE = val;
			MOTOR_FR_BASE = val;
			MOTOR_BL_BASE = val;
			MOTOR_BR_BASE = val;
		}
    rollPID.SetTunings(ROLL_P, ROLL_I, ROLL_D);
    pitchPID.SetTunings(PITCH_P, PITCH_I, PITCH_D);
	}
}

/* Set both PID controller to manual, or "off" */
void PID_off(void){
  roll_feedback = 0;          /* resets the internal sums */
  rollPID.SetMode(MANUAL);
  pitch_feedback = 0;         
  pitchPID.SetMode(MANUAL);
  yaw_feedback = 0;         
  yawPID.SetMode(MANUAL);
}
/* Set PID controllers to automatic, or "on" */
void PID_on(void){
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
}

void PID_setup(void){
  PID_off();
  rollPID.SetOutputLimits(-ROLL_MAX, ROLL_MAX);
  pitchPID.SetOutputLimits(-PITCH_MAX, PITCH_MAX);
  yawPID.SetOutputLimits(-YAW_MAX, YAW_MAX);

  rollPID.SetSampleTime(PID_SAMPLE_RATE);
  pitchPID.SetSampleTime(PID_SAMPLE_RATE);
  yawPID.SetSampleTime(PID_SAMPLE_RATE);

}


void setup() {
	Serial.begin(115200);

	/* Serial1 used to read from ESP8266 */
	Serial1.begin(115200); 
	pinMode(LED_PIN, OUTPUT);                               // LED Visual Output
  //ESCArmAll();

  IMU_setup();
  IMU_calibrate();

 	StartButtonSetup();

  PID_setup();
	//calibrateAll();  

//  ESCArmAll();
//  startMotors();
//  delay(10);
//  ESC_BR.speed(SPEED_MIN_SPIN);
//
//  ESC_FL.speed(SPEED_MIN_SPIN);
//  ESC_FR.speed(SPEED_MIN_SPIN);
//  ESC_BL.speed(SPEED_MIN_SPIN);
//  while(1);

	while(!WAITFORSTART){
		//Serial.print(".");
    checkWifiForStart();
		delay(10);
	}
	ESCArmAll();

	digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed

	startMotors();
	delay(10);

	MOTOR_FL_BASE = 1000;
	MOTOR_FR_BASE = 1000;
	MOTOR_BL_BASE = 1000;
	MOTOR_BR_BASE = 1000;

}

unsigned long last = 0;

void loop() {
  unsigned long now = micros();
  if (now - last > 5000) Serial.println(now - last);
  last = now;
  /* Check for any new commands over serial monitor */
	checkSerial0();
  
//	Serial.print("Altitude:");
//	Serial.print(ALTITUDE);
//	Serial.print("\t");

//  long now = micros();
  readSpeedFromWifi();
//  if (micros() - now > 1000){
//      Serial.println(micros() - now);
//  }
 // joystickToBaseThrust();
  readSensors();

	/* Only execute the balancing code if above the threshold */
	if (MOTOR_FL_BASE <= PID_THRESHOLD || MOTOR_FR_BASE <= PID_THRESHOLD || MOTOR_BR_BASE <= PID_THRESHOLD || MOTOR_BL_BASE <= PID_THRESHOLD){
    
		MOTOR_FL_BAL = 0;
		MOTOR_FR_BAL = 0;
		MOTOR_BL_BAL = 0;
		MOTOR_BR_BAL = 0;   
   
    PID_off();

	}
	else{
          /* Code to balance the quad */
          PID_on();
      		PIDLoop();
	}

	TARGET_FL_VAL = MOTOR_FL_BASE + MOTOR_FL_BAL;
	TARGET_FR_VAL = MOTOR_FR_BASE + MOTOR_FR_BAL;
	TARGET_BL_VAL = MOTOR_BL_BASE + MOTOR_BL_BAL;
	TARGET_BR_VAL = MOTOR_BR_BASE + MOTOR_BR_BAL;

  /* Keep it within certain region, prevent motors from going crazy */
//  if (TARGET_FL_VAL > MAX_TOTAL_THRUST || TARGET_FR_VAL > MAX_TOTAL_THRUST){
//    slowMotorsToStop();
//    Serial.println("WE HAVE A PROBLEM!!!!");
//    while(1){
//      digitalWrite(LED_BUILTIN, ~digitalRead(LED_BUILTIN));
//    }
//  }
//	TARGET_FL_VAL = constrain(TARGET_FL_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
//	TARGET_FR_VAL = constrain(TARGET_FR_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
//	TARGET_BL_VAL = constrain(TARGET_BL_VAL, SPEED_MIN, MAX_TOTAL_THRUST);
//	TARGET_BR_VAL = constrain(TARGET_BR_VAL, SPEED_MIN, MAX_TOTAL_THRUST);


  /* Make sure we have room to correct at max thrust */
  if (MOTOR_BL_BASE > MAX_TOTAL_THRUST) MOTOR_BL_BASE = MAX_TOTAL_THRUST; 
  if (MOTOR_FR_BASE > MAX_TOTAL_THRUST) MOTOR_FR_BASE = MAX_TOTAL_THRUST; 
  if (MOTOR_FL_BASE > MAX_TOTAL_THRUST) MOTOR_FL_BASE = MAX_TOTAL_THRUST; 
  if (MOTOR_BR_BASE > MAX_TOTAL_THRUST) MOTOR_BR_BASE = MAX_TOTAL_THRUST; 

  if (MOTOR_BL_BASE == 1000 || MOTOR_FR_BASE == 1000 || MOTOR_FL_BASE == 1000 || MOTOR_BR_BASE == 1000){
    TARGET_FL_VAL = constrain(TARGET_FL_VAL, SPEED_MIN, SPEED_MAX);
    TARGET_FR_VAL = constrain(TARGET_FR_VAL, SPEED_MIN, SPEED_MAX);
    TARGET_BL_VAL = constrain(TARGET_BL_VAL, SPEED_MIN, SPEED_MAX);
    TARGET_BR_VAL = constrain(TARGET_BR_VAL, SPEED_MIN, SPEED_MAX);
  }
  else{
//    if (TARGET_FL_VAL < SPEED_MIN_SPIN){
//      Serial.print("Target Val:");
//      Serial.print(TARGET_FL_VAL);
//      Serial.print("\tBase Thrust:");
//      Serial.print(MOTOR_FL_BASE);
//      Serial.print("\tFeedback");
//      Serial.print(roll_feedback);
//      Serial.print(" ");
//      Serial.print(pitch_feedback);
//      Serial.print(" ");
//
//      Serial.print(roll);
//      Serial.print(" ");
//
//      Serial.print(pitch);
//      Serial.print("\tinput:");
//      Serial.println(input);
//    }
    TARGET_FL_VAL = constrain(TARGET_FL_VAL, SPEED_MIN_SPIN, SPEED_MAX);
    TARGET_FR_VAL = constrain(TARGET_FR_VAL, SPEED_MIN_SPIN, SPEED_MAX);
    TARGET_BL_VAL = constrain(TARGET_BL_VAL, SPEED_MIN_SPIN, SPEED_MAX);
    TARGET_BR_VAL = constrain(TARGET_BR_VAL, SPEED_MIN_SPIN, SPEED_MAX);
  }
  
	/* This is to avoid too quick of deceleration or acceleration, which removes props */
//	if (TARGET_FL_VAL < FL_VAL - MAX_DECELERATION){
//		FL_VAL = FL_VAL - MAX_DECELERATION;
//	}
//  else if(TARGET_FL_VAL > FL_VAL + MAX_ACCELERATION){
//    FL_VAL = FL_VAL + MAX_ACCELERATION;
//  }
//	else{
//		FL_VAL = TARGET_FL_VAL;
//	}
//
//	if (TARGET_FR_VAL < FR_VAL - MAX_DECELERATION){
//		FR_VAL = FR_VAL - MAX_DECELERATION;
//	}
//  else if (TARGET_FR_VAL > FR_VAL + MAX_ACCELERATION){
//    FR_VAL = FR_VAL + MAX_ACCELERATION;
//  }
//	else{
//		FR_VAL = TARGET_FR_VAL;
//	}
//
//	if (TARGET_BL_VAL < BL_VAL - MAX_DECELERATION){
//		BL_VAL = BL_VAL - MAX_DECELERATION;
//	}
//  else if (TARGET_BL_VAL > BL_VAL + MAX_ACCELERATION){
//    BL_VAL = BL_VAL + MAX_ACCELERATION;
//  }
//	else{
//		BL_VAL = TARGET_BL_VAL;
//	}
//
//	if (TARGET_BR_VAL < BR_VAL - MAX_DECELERATION){
//		BR_VAL = BR_VAL - MAX_DECELERATION;
//	}
//  else if (TARGET_BR_VAL > BR_VAL + MAX_ACCELERATION){
//    BR_VAL = BR_VAL + MAX_ACCELERATION;
//  }
//	else{
//		BR_VAL = TARGET_BR_VAL;
//	}
  FL_VAL = TARGET_FL_VAL;
  FR_VAL = TARGET_FR_VAL;
  BL_VAL = TARGET_BL_VAL;
  BR_VAL = TARGET_BR_VAL;


  /* Send values to ESCs */
  ESC_FL.speed(FL_VAL);                                    
  ESC_FR.speed(FR_VAL);                                    
  ESC_BL.speed(BL_VAL);                                    
  ESC_BR.speed(BR_VAL);  


  /* protect against over rolling or pitching */
  if (roll > MAX_ROLL_DEGREES || pitch > MAX_PITCH_DEGREES){
          resetFunc();    // Kill it!
  }


	//delay(10);                                            // waits 10ms for the ESC to reach speed
	//Serial.print("FL: ");
	//Serial.print(MOTOR_FL_BASE);
	//Serial.print("+");
	//Serial.print(MOTOR_FL_BAL);
	//Serial.print("  FR: ");
	//Serial.print(MOTOR_FR_BASE);
	//Serial.print("+");
	//Serial.print(MOTOR_FR_BAL);
	//Serial.print("  BL: ");
	//Serial.print(MOTOR_BL_BASE);
	//Serial.print("+");
	//Serial.print(MOTOR_BL_BAL);
	//Serial.print("  BR: ");
	//Serial.print(MOTOR_BR_BASE);
	//Serial.print("+");
	//Serial.println(MOTOR_BR_BAL);
//
//	Serial.print(" FL: ");
//	Serial.print(FL_VAL);
//	Serial.print("  FR: ");
//	Serial.print(FR_VAL);
//	Serial.print("  BL: ");
//	Serial.print(BL_VAL);
//	Serial.print("  BR: ");
//	Serial.println(BR_VAL);


	//oESC = SPEED_MIN + 50;
	//ESC_FL.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//ESC_FR.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//ESC_BL.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//ESC_BR.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//delay(10);                                            // waits 10ms for the ESC to reach speed


	//oESC = SPEED_MIN + 50;
	//ESC_FL.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//delay(2000);
	//ESC_FR.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//delay(2000);
	//ESC_BL.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//delay(2000);
	//ESC_BR.speed(oESC);                                    // tell ESC to go to the oESC speed value
	//delay(2000);
	//delay(10);                                            // waits 10ms for the ESC to reach speed



//  startMotors();
//  //calibrateAll();
//  delay(1000);            
//  for (oESC = SPEED_MIN; oESC <= SPEED_MIN + 500; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
//    ESC_FL.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_FR.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_BL.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_BR.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    delay(10);                                            // waits 10ms for the ESC to reach speed
//  }
//
//  //digitalWrite(LED_PIN, LOW);                            // LED High Once Armed
//  for (oESC = SPEED_MIN + 500; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
//    ESC_FL.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_FR.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_BL.speed(oESC);                                    // tell ESC to go to the oESC speed value
//    ESC_BR.speed(oESC);                                    // tell ESC to go to the oESC speed value    delay(10);                                            // waits 10ms for the ESC to reach speed  
//    delay(10);
//  }
//  delay(5000);                                            // Wait for a while befor restart  
//  while(1);
}
