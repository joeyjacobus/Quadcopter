
#define ALPHA 0.999  

#define IMU_SAMPLE_RATE_HZ 10000;     //Go as fast as possible... 
#define CALIB_LED_PIN 23
MPU9255 mpu;




/* IMU global vars */
const double IMU_SAMPLE_RATE_MICROS = 1000000 / IMU_SAMPLE_RATE_HZ;
static double gyro_roll_cal = 0;
static double gyro_pitch_cal = 0;
static double gyro_yaw_cal = 0;

double Axyz[3];
double Gxyz[3];
double Mxyz[3];

double dt, last_time;
double RollDt, PitchDt, YawDt;
double RollAccel, PitchAccel;

/* initalize IMU*/
void IMU_setup(void){
  Serial.print("Setting up IMU...");
  mpu.init();// Initialize MPU9255 chip 
  Serial.println("Done!");
  pinMode(CALIB_LED_PIN, OUTPUT);
  digitalWrite(CALIB_LED_PIN, HIGH);


}

void IMU_calibrate(void){
  /* calibrate */
  int i;

  Serial.print("Calibrating IMU. Hold steady...");
  for (i = 0; i < 2000; i++){
    mpu.read_acc();
    mpu.read_gyro();
    mpu.read_mag();    
    gyro_roll_cal += mpu.gx / 131.0;
    gyro_pitch_cal += mpu.gy / 131.0;
    gyro_yaw_cal += mpu.gz / 131.0;
//    Serial.print("i:");
//    Serial.println(i);
    if (i % 100 == 0){
      digitalWrite(CALIB_LED_PIN, !digitalRead(CALIB_LED_PIN));
    }
    delay(1);
  }
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
  digitalWrite(CALIB_LED_PIN, LOW);

  /* Account for any initial offset, we want to use accelerometer more at the 
    beginning, then make it less impactful once we have our initial orientation */
  double rollSum = 0;
  double pitchSum = 0;
  for (i = 0; i < 20; i++){
    readAcc();
    /* Calculate the roll and pitch from the accelerometer and convert to degrees */
    rollSum += (float) atan2 (Axyz[1], Axyz[2]) * ( 57.296);
    pitchSum += (float) (atan2 (-Axyz[0], sqrt((Axyz[1] * Axyz[1]) + (Axyz[2] * Axyz[2]))) * (57.296));
  }
  roll = rollSum / 20.0;
  pitch = pitchSum / 20.0;
//
//  Serial.print("Roll: ");
//  Serial.print(roll);
//  Serial.print("\tPitch: ");
//  Serial.println(pitch); 
  
  Serial.println("Done!");
}


void PIDLoop(void){
	long deltaT;
	/*
	 * Roll is left to right, z axis given our sensor orientation 
	 * right is negative, left is positive 
	 */
  //Serial.println(roll_feedback);
  rollPID.Compute();
//  roll_feedback = 0;
//
//  Serial.print("Roll feedback:");
//  Serial.println(roll_feedback);

  /* Pitch is front to back, forward is negative, backwards is positive */
  pitchPID.Compute();
	//pitch_feedback = 0;
//	Serial.print("\tPitch feedback:");
//  Serial.println(pitch_feedback);


  /**
   * Increasing counterclockwise motors will cause the craft to rotate clockwise
   * Increasing clockwise motors will cause craft to rotate counterclockwise
   * FL and BR are counter-clockwise
   * FR and BL are clockwise
   */
  yawPID.Compute();
  // yaw-feedback is negative, we want to rotate clockwise
  
//  Serial.print("\tYaw feedback:");
//  Serial.println(yaw_feedback);
  
	/* When roll_feedback is negative, this will increase right side motors, and decrease left */
	/* when positive it will do the opposite */
	/* front motors +pitch feedback, back motors -pitch_feedback */
	/* right motors +roll feedback, left motors -roll_feedback */
	MOTOR_FR_BAL = roll_feedback + pitch_feedback + yaw_feedback;
	MOTOR_BR_BAL = roll_feedback - pitch_feedback - yaw_feedback;
	MOTOR_FL_BAL = -roll_feedback + pitch_feedback - yaw_feedback;
	MOTOR_BL_BAL = -roll_feedback - pitch_feedback + yaw_feedback;
}



void readSensors(void){
  
  readGyro();
  readAcc();
  readMagnet();
  
  dt = micros() - last_time;
  //Serial.println(dt);
  if (dt < IMU_SAMPLE_RATE_MICROS){
    return;
  }
  last_time = micros();

  dt /= 1000000;
  RollDt = Gxyz[0] * dt;
  PitchDt = Gxyz[1] * dt;
  YawDt = Gxyz[2] * dt;

  /* Calculate the roll and pitch from the accelerometer and convert to degrees */
  RollAccel = (float) atan2 (Axyz[1], Axyz[2]) * ( 57.296);
  PitchAccel = (float) (atan2 (-Axyz[0], sqrt((Axyz[1] * Axyz[1]) + (Axyz[2] * Axyz[2]))) * (57.296));

  /* Complementary filter to reduce drift over time */
  roll = ALPHA * (roll + RollDt) + (1 - ALPHA) * RollAccel;
  pitch = ALPHA * (pitch + PitchDt) + (1 - ALPHA) * PitchAccel;
  yaw = yaw + YawDt; 

//  double yawangle = atan2(Mxyz[0], Mxyz[1]) * 180 / PI;
//  Serial.print( " Yaw:");
//  Serial.println(yaw);



//  Serial.print("Roll: ");
//  Serial.print(roll);
//  Serial.print("\tPitch: ");
//  Serial.println(pitch);  
}

void readGyro(void){
  mpu.read_gyro();

  float gx=mpu.gx;
  float gy=mpu.gy;
  float gz=mpu.gz;


  //---- Gyroscope data ----
  /*
  To get rotation in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value) 
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale 
  
  */
  Gxyz[0]=gx/131 - gyro_roll_cal;
  Gxyz[1]=gy/131 - gyro_pitch_cal;
  Gxyz[2]=gz/131 - gyro_yaw_cal;
  
}


void readAcc(void){
  mpu.read_acc();
  float ax=mpu.ax;
  float ay=mpu.ay;
  float az=mpu.az;
    //---- Acceleration ---- 
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default value) 
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale 
  */
  Axyz[0]=ax/16384;
  Axyz[1]=ay/16384;
  Axyz[2]=az/16384;
}

void readMagnet(void){
  mpu.read_mag();

  float mx=mpu.mx;
  float my=mpu.my;
  float mz=mpu.mz;

    //---- Magnetic flux ----
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by some number (calibration) and then divided by 0.6.
  (Faced North each axis should output arround 31 µT) (without any metal / walls around) 
  Note : This manetometer has really low initial calibration tolerance : +- 500 unit ( 833,3 μT ) !!!  
  Scale of the magnetometer is fixed -> +- 4800 μT. 
  */
  const float cal=0.06;
  Mxyz[0]=mx*cal;
  Mxyz[1]=my*cal;
  Mxyz[2]=mz*cal;
  Mxyz[0]=Mxyz[0]/0.6;
  Mxyz[1]=Mxyz[1]/0.6;
  Mxyz[2]=Mxyz[2]/0.6;

}

