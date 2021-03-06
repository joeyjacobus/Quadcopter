

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Hardcode WiFi parameters as this isn't going to be moving around.
const char* ssid = "MyDronesAP";       // ssid of server (Access Point (AP))
const char* password = "123456789";        // password of server (Access Point (AP))
WiFiServer server(80);            //
WiFiClient client;
void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);   // set GPIO 2 as an output
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  WiFi.mode(WIFI_AP_STA);  // Set WiFi to AP and station mode
  WiFi.softAP(ssid, password) ;

  
  // Start the TCP server
  server.begin();
  //Serial.println("Starting the server");

}

void loop() {
  // listen for incoming clients
 client = server.available();
 //Serial.println("checking.");
  if (client){
    client.setTimeout(1000);
    //Serial.println("Client connected");
    while (client.connected()){
        // Read the incoming TCP command
        //Serial.println("reading line");
        digitalWrite(LED_BUILTIN, ~digitalRead(LED_BUILTIN));   // Turn the LED on (Note that LOW is the voltage level

        String command = client.readStringUntil('\n');
        // Debugging display command
        
        Serial.println(command);
        client.flush();

    }
  }
 }
