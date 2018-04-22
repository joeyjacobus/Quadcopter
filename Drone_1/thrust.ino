


void PIDLoop(void){
	long deltaT;
	double roll;
	double roll_feedback;
	double pitch;
	double pitch_feedback;
	
	sensors_event_t event; 
	bno.getEvent(&event);

	if(TIME_LAST == 0){
		TIME_LAST = micros();
		return;
	}

	TIME_NOW = micros();
	deltaT = TIME_NOW - TIME_LAST;

	/*
	 * Roll is left to right, z axis given our sensor orientation 
	 * right is negative, left is positive 
	 */
	roll = event.orientation.z;
	ROLL_ERROR = TARGET_ROLL - roll;
	ROLL_ERROR_SUM += ROLL_I * ROLL_ERROR * deltaT / 1000000.0;
	ROLL_ERROR_DIFF = ROLL_D * (ROLL_ERROR - ROLL_ERROR_LAST) * 1000000.0 / deltaT;
	ROLL_ERROR_LAST = ROLL_ERROR;
//	Serial.print("Roll:\t");
//	Serial.print(roll);
//	Serial.print("\tFeedback = ");
//	Serial.print(ROLL_ERROR * ROLL_P);
//	Serial.print(" + ");
//	Serial.print(ROLL_ERROR_SUM * ROLL_I);
//	Serial.print(" + ");
//	Serial.print(ROLL_ERROR_DIFF * ROLL_D);
//	Serial.print(" = ");
//	Serial.print(ROLL_ERROR * ROLL_P + ROLL_ERROR_SUM * ROLL_I + ROLL_ERROR_DIFF * ROLL_D);
//	Serial.print("\t");
	roll_feedback = ROLL_ERROR * ROLL_P + ROLL_ERROR_SUM + ROLL_ERROR_DIFF;	
	roll_feedback = constrain(roll_feedback, -ROLL_MAX, ROLL_MAX);
	//roll_feedback = 0;

  /* Pitch is front to back, forward is negative, backwards is positive */
	pitch = event.orientation.y;
	PITCH_ERROR = TARGET_PITCH - pitch;
	PITCH_ERROR_SUM += PITCH_I * PITCH_ERROR * deltaT / 1000000.0;
	PITCH_ERROR_DIFF = PITCH_D * (PITCH_ERROR - PITCH_ERROR_LAST) * 1000000.0 / deltaT;
	PITCH_ERROR_LAST = PITCH_ERROR;
//	Serial.print("\tPitch:\t");
//	Serial.print(pitch);
//  Serial.println();

//	Serial.print("\tFeedback = ");
//	Serial.print(PITCH_ERROR * PITCH_P);
//	Serial.print(" + ");
//	Serial.print(PITCH_ERROR_SUM * PITCH_I);
//	Serial.print(" + ");
//	Serial.print(PITCH_ERROR_DIFF * PITCH_D);
//	Serial.print(" = ");
//	Serial.print(PITCH_ERROR * PITCH_P + PITCH_ERROR_SUM * PITCH_I + PITCH_ERROR_DIFF * PITCH_D);
//	Serial.print("\t");

	pitch_feedback = PITCH_ERROR * PITCH_P + PITCH_ERROR_SUM + PITCH_ERROR_DIFF;
	pitch_feedback = constrain(pitch_feedback, -PITCH_MAX, PITCH_MAX);
	pitch_feedback = 0;
	
	/* When roll_feedback is negative, this will increase right side motors, and decrease left */
	/* when positive it will do the opposite */
	/* front motors +pitch feedback, back motors -pitch_feedback */
	/* right motors +roll feedback, left motors -roll_feedback */
	MOTOR_FR_BAL = roll_feedback + pitch_feedback;
	MOTOR_BR_BAL = roll_feedback - pitch_feedback;
	MOTOR_FL_BAL = -roll_feedback + pitch_feedback;
	MOTOR_BL_BAL = -roll_feedback - pitch_feedback;

//	int bound = SPEED_MAX - SPEED_MIN_SPIN;
//	MOTOR_FR_BAL = constrain(MOTOR_FR_BAL, -bound, bound);
//	MOTOR_BR_BAL = constrain(MOTOR_BR_BAL, -bound, bound);
//	MOTOR_FL_BAL = constrain(MOTOR_FL_BAL, -bound, bound);
//	MOTOR_BL_BAL = constrain(MOTOR_BL_BAL, -bound, bound);

	//Serial.print("FL: ");
	//Serial.print(MOTOR_FL_BAL);
	//Serial.print("  BL: ");
	//Serial.print(MOTOR_BL_BAL);
	//Serial.print("  FR: ");
	//Serial.print(MOTOR_FR_BAL);
	//Serial.print("  BR: ");
	//Serial.println(MOTOR_BR_BAL);

	//delay(100);

	/* TODO Calculate thrust based on controller input */


	TIME_LAST = TIME_NOW;
}

/**  --------------------------
 * echo_interrupt() External interrupt from HC-SR04 echo signal. 
 * Called every time the echo signal changes state.
 * 
 * Calculates the altitude adjustment using basic PID feedback loop
 */
void echoInterrupt()
{
	switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
	{
		case HIGH:                                      // High so must be the start of the echo pulse
			ECHO_END = 0;                                 // Clear the end time
			ECHO_START = micros();                        // Save the start time
			return;

		case LOW:                                       // Low so must be the end of the echo pulse
			ECHO_END = micros();                          // Save the end time
			ECHO_DURATION = ECHO_END - ECHO_START;        // Calculate the pulse duration
			double tmp_altitude = ECHO_DURATION / 148 - SENSOR_DISTANCE_TO_GROUND; 
			/* Ignore case where value is larger than max detectable by sensor */
			if (tmp_altitude > 158){
				break;
				}
			ALTITUDE = tmp_altitude;
			break;
	}

	/* Calculate base thrust based on feedback to hold altitude */
	long deltaT;
	if(ALTITUDE_TIME_LAST == 0){
		ALTITUDE_TIME_LAST = micros();
	}
	ALTITUDE_TIME_NOW = micros();
	deltaT = ALTITUDE_TIME_NOW - ALTITUDE_TIME_LAST;

	/* final thrust = altitude pid + balance pid */
	/* Altitude PID */
	ALTITUDE_ERROR = TARGET_ALTITUDE - ALTITUDE; 
	ALTITUDE_ERROR_SUM += ALTITUDE_ERROR * deltaT / 100000.0; 
	ALTITUDE_ERROR_DIFF = (ALTITUDE_ERROR - ALTITUDE_ERROR_LAST); /// ((double)deltaT / 100000);
	ALTITUDE_ERROR_LAST = ALTITUDE_ERROR;


	//Serial.print("Feedback = ");
	//Serial.print(ALTITUDE_ERROR * ALTITUDE_P);
	//Serial.print(" + ");
	//Serial.print(ALTITUDE_ERROR_SUM * ALTITUDE_I);
	//Serial.print(" + ");
	//Serial.print(ALTITUDE_ERROR_DIFF * ALTITUDE_D);
	//Serial.print(" = ");
	//Serial.println(ALTITUDE_ERROR * ALTITUDE_P + ALTITUDE_ERROR_SUM * ALTITUDE_I + ALTITUDE_ERROR_DIFF * ALTITUDE_D);

	double feedback = ALTITUDE_ERROR * ALTITUDE_P + ALTITUDE_ERROR_SUM * ALTITUDE_I + ALTITUDE_ERROR_DIFF * ALTITUDE_D;
	MOTOR_FL_BASE += feedback;
	MOTOR_FR_BASE += feedback;
	MOTOR_BL_BASE += feedback;
	MOTOR_BR_BASE += feedback;

	MOTOR_FL_BASE = constrain( MOTOR_FL_BASE, SPEED_MIN, SPEED_MAX);
	MOTOR_FR_BASE = constrain( MOTOR_FR_BASE, SPEED_MIN, SPEED_MAX);
	MOTOR_BL_BASE = constrain( MOTOR_BL_BASE, SPEED_MIN, SPEED_MAX);
	MOTOR_BR_BASE = constrain( MOTOR_BR_BASE, SPEED_MIN, SPEED_MAX);     

	ALTITUDE_TIME_LAST = ALTITUDE_TIME_NOW;
}
