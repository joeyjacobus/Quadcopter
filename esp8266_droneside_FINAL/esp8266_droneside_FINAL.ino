/* Server
*  We write the server first because we need its IP address
*  https://arduino.stackexchange.com/questions/18176/peer-to-peer-communication
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

const char* ssid = "MyDronesAP";       // ssid of server (Access Point (AP))
const char* password = "123456789";        // password of server (Access Point (AP))
WiFiServer server(80);            //Service Port

int ledPin = 2; // GPIO2 of Server ESP8266

void setup() 
{
    delay(1000);
    Serial.begin(115200);    // to use tools->serial monitor

    pinMode(ledPin, OUTPUT);   // set GPIO 2 as an output

    WiFi.mode(WIFI_AP_STA);  // Set WiFi to AP and station mode

    // Connect to the WiFi network
//    Serial.println();
//    Serial.print("Connecting to: "); Serial.println(ssid);
    WiFi.softAP(ssid, password) ;

    // Display the server address
//    Serial.print("Connected, My address: ");
//    Serial.print("http://");
//    Serial.print(WiFi.softAPIP());
//    Serial.println("/");

    // Tell the server to begin listening for incoming connections
    server.begin();
    Serial.println("Server listening for incoming connections");
}  

void loop() 
{
    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) // if not available, return
    {
        return;
    }

    // Wait until the client sends some data
    //Serial.println();
    //Serial.println("Client connected");
    while(!client.available())
    {
        delay(1);
    }

    // Read the request
    String request = client.readStringUntil('\n');  
    client.flush();

    //Serial.print("Request = "); 
    Serial.println(request);

    
    //Serial.println("Client disconnected");

}
