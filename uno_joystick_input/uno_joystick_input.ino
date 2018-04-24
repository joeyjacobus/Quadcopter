/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13. 
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead(). 
 
 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +3.3V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/AnalogInput
 
 */
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to -1!
#define TFT_DC     7
// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
 
#define ESP_ENABLE 8
#define ESP_RST 7
#define LS_CLICK_PIN 2


#define SPEED_MIN 1000
#define MIN_THRUST 1200
#define TAKEOFF_THRUST 1300
#define MAX_CONTROLLER_THRUST 1300                        // The maximum value the controller can set the base thrust

#define MAX_ROLL 10
#define MAX_PITCH 10


#define JOYSTICK_HYST 20

int sensorPinLX = A5;
int sensorPinLY = A4;    // select the input pin for the potentiometer
 
int sensorPinRX = A1;
int sensorPinRY = A0;    // select the input pin for the potentiometer

int sensorPinSlidePot = A2;

int ledPin = LED_BUILTIN;      // select the pin for the LED

int sensorValueLX = 0;  // variable to store the value coming from the sensor
int sensorValueLY = 0;  // variable to store the value coming from the sensor
int sensorValueRX = 0;  // variable to store the value coming from the sensor
int sensorValueRY = 0;  // variable to store the value coming from the sensor
int sensorValueSlidePot = 0;

bool WAITFORSTART = false;

void leftStickClicked(void){
  Serial.println("START");
  WAITFORSTART= true;
}


void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void tftWriteVals( int lx, int ly, int rx, int ry) {
  tft.setTextWrap(true);
  //tft.fillScreen(ST7735_BLACK);
  
  /* clear the screen */
  
  //tft.fillRect(0, 10, tft.width(), 60,ST7735_BLACK);
  
  tft.setCursor(0, 10);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setTextSize(1);

  tft.print("lx:");
  tft.print(lx);
  tft.println(" ");
  tft.print("ly:");
  tft.print(ly); 
  tft.println(" ");
  tft.print("rx:");
  tft.print(rx);
  tft.println(" ");
  tft.print("ry:");
  tft.print(ry);
  tft.println(" ");
}



void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  pinMode(ESP_ENABLE, OUTPUT);
  pinMode(ESP_RST, OUTPUT);
  digitalWrite(ESP_ENABLE, HIGH);
  digitalWrite(ESP_RST, LOW);
  Serial.begin(115200);
  pinMode(LS_CLICK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LS_CLICK_PIN), leftStickClicked, FALLING);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  testdrawtext("Click the left stick to continue!", ST7735_WHITE);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setTextSize(1);
  while(!WAITFORSTART){
    delay(100);
  }
  tft.fillScreen(ST7735_BLACK);

}

/*
 * Sends the longer format string
 * "LX=512 LY=234 RX=232 RY=232 CK=xxx" 
 */
void sendJoystickVals(void){
  int checksum = (sensorValueLX + sensorValueLY + sensorValueRX + sensorValueRY) % 256;
  // stop the program for <sensorValue> milliseconds:
  Serial.print("LX=");
  Serial.print(sensorValueLX); 
  Serial.print(" ");          
  Serial.print("LY=");
  Serial.print(sensorValueLY); 
  Serial.print(" ");
  Serial.print("RX=");
  Serial.print(sensorValueRX); 
  Serial.print(" ");  
  Serial.print("RY=");        
  Serial.print(sensorValueRY); 
  Serial.print(" ");  
  Serial.print("CK=");        
  Serial.print(checksum);  
  Serial.print("\n");
}


void sendBaseThrustOnly(void){
  int thrust;
  int base1 = sensorValueLY - 512;
  int base2 = sensorValueRY - 512;
  if (base1 < JOYSTICK_HYST && base1 > -JOYSTICK_HYST){
    base1 = 0;
  }    
  if (base2 < JOYSTICK_HYST && base2 > -JOYSTICK_HYST){
    base2 = 0;
  }    
  
  if (base1 == 0 && base2 == 0){
    thrust = SPEED_MIN;
  }
  else {
    double ratio1 = base1 / 512.0;
    double ratio2 = base2 / 512.0;
    thrust = MIN_THRUST + (TAKEOFF_THRUST - MIN_THRUST) * ratio1;
    thrust +=  (MAX_CONTROLLER_THRUST - TAKEOFF_THRUST) * ratio2;
  }
  Serial.print(thrust);
  Serial.print(" ");

  /* Send the checksum */
  Serial.print(thrust);
  Serial.print("\n");
  tft.setCursor(10, 70);
  tft.print("Base:");
  tft.print(thrust);

}

void sendPitchRollBaseThrust(void) {
  int thrust;
  int pitch; 
  int roll;
  int checksum;
  int slide = sensorValueSlidePot;
  int ry = sensorValueRY - 512;
  int rx = sensorValueRX - 512;

  if (rx < JOYSTICK_HYST && rx > -JOYSTICK_HYST){
    rx = 0;
  }      
  if (ry < JOYSTICK_HYST && ry > -JOYSTICK_HYST){
    ry = 0;
  }    
  
  if (slide == 0){
    thrust = SPEED_MIN;
  }
  else{
    thrust = map(slide, 0, 1023, MIN_THRUST, MAX_CONTROLLER_THRUST);
  }
  
  if (rx == 0){
    roll = 0.0;
  }
  else{
    roll = map(rx, -512, 512, -MAX_ROLL, MAX_ROLL);
  }
  if (ry == 0){
     pitch = 0.0;
  }
  else{
     pitch = map(ry, -512, 512, -MAX_PITCH, MAX_PITCH);
  }
  
  checksum = ( thrust + roll + pitch ) % 256;
  Serial.print(thrust);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(checksum);

  tft.setCursor(0, 70);
  tft.println(thrust);
  tft.print(roll);
  tft.println("    ");
  tft.print(pitch);
  tft.println("    ");
  
}

void loop() {
  // read the value from the sensor:
  sensorValueLX = analogRead(sensorPinLX); 
  sensorValueLY = analogRead(sensorPinLY); 
  sensorValueRX = analogRead(sensorPinRX); 
  sensorValueRY = analogRead(sensorPinRY);   
  sensorValueSlidePot = analogRead(sensorPinSlidePot);

  // turn the ledPin on
  digitalWrite(ledPin, HIGH);  

  
  //sendBaseThrustOnly();
  sendPitchRollBaseThrust();
  //sendJoystickVals();
 
  
  //tftWriteVals(sensorValueLX, sensorValueLY, sensorValueRX, sensorValueRY);
        
  // turn the ledPin off:        
  digitalWrite(ledPin, LOW);   
  // stop the program for for <sensorValue> milliseconds:
  delay(10);                  
}
