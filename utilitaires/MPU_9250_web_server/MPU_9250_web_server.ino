#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char* ssid = "ssid";
const char* password = "pswd";

ESP8266WebServer server(80);

String statusDroneValue = "Not Ready";

void handleRoot() {
  server.send(200, "text/plain", "Drone Control");
}

void statusDrone() {
  if (statusDroneValue == "Ready"){
    server.send(200, "text/plain", "Drone Ready");
  } else {
    server.send(200, "text/plain", "Drone Not Ready");
  }
}

void startDrone() {
  server.send(200, "text/plain", "start");
  Serial.print("start");
}

void stopDrone() {
  server.send(200, "text/plain", "stop");
  Serial.print("stop");
}


void handleNotFound(){
   String message = "File Not Found\n\n";
   message += "URI: ";
   message += server.uri();
   message += "\nMethod: ";
   message += (server.method() == HTTP_GET)?"GET":"POST";
   message += "\nArguments: ";
   message += server.args();
   message += "\n";
   for (uint8_t i=0; i<server.args(); i++){
     message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
   }
   server.send(404, "text/plain", message);
}

void setup(void){
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  statusDroneValue = "Ready";
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  if (MDNS.begin("esp8266")) {
    // Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/start", startDrone);
  server.on("/stop", stopDrone);
  server.on("/status", statusDrone);

  server.onNotFound(handleNotFound);

  server.begin();
  
}

void loop(void){
  server.handleClient();
}
