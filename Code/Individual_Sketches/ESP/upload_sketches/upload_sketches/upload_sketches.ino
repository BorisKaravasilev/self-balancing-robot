// https://blog.flynnmetrics.com/applications/nodemcu-esp8266-upload-sketches-wirelessly/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>

const char* ssid = "WiFimodem-ADB7";
const char* password = "hdn3utnlft";
const char* host = "ESP-OTA";

#define led_pin BUILTIN_LED
#define blu_pin D4
#define beat 450
IPAddress ip(192, 168, 1, 196); //Node static IP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
void setup() {
  Serial.begin(9600);

  delay(25);

  pinMode(led_pin, OUTPUT);

  Serial.println();
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  WiFi.config(ip, gateway, subnet);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("Retrying connection…");
    delay(5);
  }
  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() { // switch off all the PWMs during upgrade
  });

  ArduinoOTA.onEnd([]() { // do a fancy thing with our board led at end
    ESP.restart();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    ESP.restart();
  });

  /* setup the OTA server */
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {

}
