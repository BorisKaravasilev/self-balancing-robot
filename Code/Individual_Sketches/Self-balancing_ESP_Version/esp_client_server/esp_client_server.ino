#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

ESP8266WebServer server(80);

String serialData = "";

// read html file as string

// read file as string
String readFile(String filename) {
  String data = "";
  if (SPIFFS.exists(filename)) {
    File f = SPIFFS.open(filename, "r");
    data = f.readString();
    f.close();
  }
  return data;
}

const String htmlPage = readFile("index.html");

const IPAddress serverIPAddress(10, 0, 0, 7);

void setup() {
    Serial.begin(9600);

    //here we set up as a hot spot, called XC4411 dual board
	WiFi.softAPConfig(serverIPAddress, serverIPAddress, IPAddress(255, 255, 255, 0));
	WiFi.softAP("XC4411 Dual Board example code");

	//here we set server paramters, the main page is the html_page from above
	server.on("/", []() { //send html code, from above
		server.send(200, "text/html", htmlPage);
	});

  server.on("/delay", []() { //send raw serial data
		Serial.println(serialData);
		server.send(200, "text/plain", serialData);
	});

  server.onNotFound(handleNotFound);
	server.begin();
}


void loop() {
  while (Serial.available())
    {
      char c = Serial.read();
      if (c == '\r')
        continue;
      serialData += c;
    }

    server.handleClient();
}

void handleNotFound() {
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for (uint8_t i = 0; i < server.args(); i++) 	{
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
}
