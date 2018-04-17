

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Hardcode WiFi parameters as this isn't going to be moving around.
const char* ssid = "JacHome";       // ssid of server (Access Point (AP))
const char* password = "WIRDATRD856793511710";        // password of server (Access Point (AP))
WiFiServer server(80);            //
WiFiClient client;
void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);   // set GPIO 2 as an output

  WiFi.mode(WIFI_STA);  // Set WiFi to  station mode
  WiFi.begin(ssid, password) ;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
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
        String command = client.readStringUntil('\n');
        // Debugging display command
        
        Serial.println(command);
        client.flush();
    }
  }
 }
