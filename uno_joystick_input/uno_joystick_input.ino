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
#define MIN_THRUST 1150
#define TAKEOFF_THRUST 1350
#define MAX_CONTROLLER_THRUST 1425                        // The maximum value the controller can set the base thrust



int sensorPinLX = A5;
int sensorPinLY = A4;    // select the input pin for the potentiometer
 
int sensorPinRX = A1;
int sensorPinRY = A0;    // select the input pin for the potentiometer

int ledPin = LED_BUILTIN;      // select the pin for the LED

int sensorValueLX = 0;  // variable to store the value coming from the sensor
int sensorValueLY = 0;  // variable to store the value coming from the sensor
int sensorValueRX = 0;  // variable to store the value coming from the sensor
int sensorValueRY = 0;  // variable to store the value coming from the sensor

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


void sendBaseThrust(void){
  int thrust;
  int base1 = sensorValueLY - 512;
  int base2 = sensorValueRY - 512;
  if (base1 < 30 && base1 > -30){
    base1 = 0;
  }    
  if (base2 < 30 && base2 > -30){
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
  Serial.print(thrust);
  Serial.print("\n");
  tft.setCursor(10, 70);
  tft.print("Base:");
  tft.print(thrust);

}

void loop() {
  // read the value from the sensor:
  sensorValueLX = analogRead(sensorPinLX); 
  sensorValueLY = analogRead(sensorPinLY); 
  sensorValueRX = analogRead(sensorPinRX); 
  sensorValueRY = analogRead(sensorPinRY);   
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);  

  
  sendBaseThrust();
  //sendJoystickVals();
 
  
  //tftWriteVals(sensorValueLX, sensorValueLY, sensorValueRX, sensorValueRY);
        
  // turn the ledPin off:        
  digitalWrite(ledPin, LOW);   
  // stop the program for for <sensorValue> milliseconds:
  delay(10);                  
}
