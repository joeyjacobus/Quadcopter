/*  Client
*   This client will turn the LED on or off every 5 seconds
*   https://arduino.stackexchange.com/questions/18176/peer-to-peer-communication
*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 

const char* ssid = "MyDronesAP";   // ssid of access point (Server)
const char* password = "123456789";    // password of access point (Server)
int port = 80;                // port number

byte ip[]= {192,168,4,1};

int LEDstatus = 0;

WiFiClient client;  // Declare client

void setup() 
{
    Serial.begin(115200);
    delay(10);

    WiFi.mode(WIFI_STA);  // set mode to station (client)

    // Connect to wiFi
    //Serial.println(); Serial.println();
    //Serial.println("CONNECTING TO WiFi");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");  // print dots until connection
    }
    Serial.println();
    Serial.println("WiFi CONNECTED");
    client.println("testing!");

}

String data;
int LX = 0;
int LY = 100;
int RX = 200;
int RY = 300;

void readDataStream() {
  data = "";
  if(Serial.available()){
    data = Serial.readStringUntil('\n');    
  }
  

//  data = "LX=";
//  LX += 1;
//  data += String(LX);
//  
//  data += " LY=";
//  LY += 1;
//  data += String(LY);
//
//  data += " RX=";
//  RX += 1;
//  data += String(RX);
//  data += " RY=";
//  RY += 1;
//  data += String(RY);
  //Serial.println("- data stream: "+data);
}

void loop() 
{
    // Connect client to server
    //Serial.println("CONNECTING CLIENT TO SERVER");
    readDataStream();
    if (data.equals("")){
      return;   //Don't do anything if no new data to send
    }
    
    if (client.connect(ip, port)) 
    {
        //Serial.println("CONNECTED");
    }
    else 
    {
        //Serial.println("CONNECTION FAILED");
    }
    while(client.connected()){
      //Serial.println("*");
      readDataStream();
      if (data.equals("")){
        continue;   //Don't do anything if no new data to send
      }
      client.println(data);
    }


    //ßclient.println();

    client.flush();

    delay(1);
    //Serial.println("CLIENT DISCONNECTED");
    //Serial.println();

}
