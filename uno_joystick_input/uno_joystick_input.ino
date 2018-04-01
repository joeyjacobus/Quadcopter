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
#define ESP_ENABLE 8
#define ESP_RST 7
 
int sensorPinLX = A5;
int sensorPinLY = A4;    // select the input pin for the potentiometer
 
int sensorPinRX = A1;
int sensorPinRY = A0;    // select the input pin for the potentiometer

int ledPin = LED_BUILTIN;      // select the pin for the LED

int sensorValueLX = 0;  // variable to store the value coming from the sensor
int sensorValueLY = 0;  // variable to store the value coming from the sensor
int sensorValueRX = 0;  // variable to store the value coming from the sensor
int sensorValueRY = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  pinMode(ESP_ENABLE, OUTPUT);
  pinMode(ESP_RST, OUTPUT);
  digitalWrite(ESP_ENABLE, HIGH);
  digitalWrite(ESP_RST, LOW);
  Serial.begin(115200);
  //Serial2.begin(115200);
}

void loop() {
  // read the value from the sensor:
  sensorValueLX = analogRead(sensorPinLX); 
  sensorValueLY = analogRead(sensorPinLY); 
  sensorValueRX = analogRead(sensorPinRX); 
  sensorValueRY = analogRead(sensorPinRY);   
    
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);  
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
  Serial.print("\n");
//  Serial2.print("LX=");
//  Serial2.print(sensorValueLX); 
//  Serial2.print(" ");          
//  Serial2.print("LY=");
//  Serial2.print(sensorValueLY); 
//  Serial2.print(" ");
//  Serial2.print("RX=");
//  Serial2.print(sensorValueRX); 
//  Serial2.print(" ");  
//  Serial2.print("RY=");        
//  Serial2.print(sensorValueRY); 
//  Serial2.print("\n");
        
  // turn the ledPin off:        
  digitalWrite(ledPin, LOW);   
  // stop the program for for <sensorValue> milliseconds:
  delay(100);                  
}
