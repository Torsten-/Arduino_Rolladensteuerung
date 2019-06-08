// TODO: encapsulate stepper with "struct"
// TODO: MQTT/WLAN/Stepper functions in extra files
// TODO: stop is not working any more with blocking stepper movement

#define VERSION "19.06"

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <PubSubClient.h> // MQTT - if you get rc=-4 you should maybe change MQTT_VERSION in library
#include <AccelStepper.h>

const int connect_wait = 20; // WLAN

// Pin-Zuordnung
#define STEPPER_COUNT 2
const uint8_t pin_stepper_dir[]   = { 2,14};
const uint8_t pin_stepper_step[]  = { 4,12};
const uint8_t pin_stepper_reed[]  = {16,13};
const uint8_t pin_stepper_sleep   = 5;

uint16_t mqttport = 1883;
const char* mqtttopic_state[] = {"rollo/rollo0/state","rollo/rollo1/state"};
const char* mqtttopic_cmd[] = {"rollo/rollo0/cmd","rollo/rollo1/cmd"};
const char* mqtttopic_abs[] = {"rollo/rollo0/abs","rollo/rollo1/abs"};
const char* mqtttopic_all_cmd = "rollo/rollo/cmd";
//const char* mqtttopic_speed_set = "fhem/torsten/rollo/speed/set";
//const char* mqtttopic_speed_get = "fhem/torsten/rollo/speed/get";

ESP8266WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String scanned_wifis;
String html;
bool wifi_connected = false;
String eeprom_ssid;
String eeprom_pass;
String eeprom_mqttServer;
byte eeprom_configWritten; // 42 = yes

boolean leave_reed_position[] = {false,false};
unsigned int time_to_leave_reed = 1000;
unsigned long time_leaving[2];

AccelStepper stepper[STEPPER_COUNT];
boolean driver_active;
boolean send_state_update[] = {false,false};

int max_steps[] = {
  (1048*16),
  (-1048*18)
};


//////////////////////////
// Setup
//////////////////////////
void setup() {
  delay(1000);
  
  EEPROM.begin(512);
  delay(10);
  
  // EEPROM: Config Written?
  eeprom_configWritten = EEPROM.read(500);
  
  if(eeprom_configWritten == 42){ // Yes
    // EEPROM: SSID
    for(int i = 0; i < 32; i++){
      int val = EEPROM.read(i);
      if(val == 0) break;
      eeprom_ssid += char(val);
    }

    // EEPROM: Password
    for(int i = 32; i < 96; i++) {
      int val = EEPROM.read(i);
      if(val == 0) break;
      eeprom_pass += char(val);
    }

    // EEPROM: mqttServer
    for(int i = 96; i < 128; i++) {
      int val = EEPROM.read(i);
      if(val == 0) break;
      eeprom_mqttServer += char(val);
    }
  }else{
    eeprom_ssid = "";
    eeprom_pass = "";
    eeprom_mqttServer = "192.168.1.1";
  }
 
  // Connect to WiFi
  WiFi.hostname("rollo");
  if(eeprom_ssid.length() > 1){
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(eeprom_ssid.c_str(), eeprom_pass.c_str());

    for(int i = 0; i < 20; i++){
      if(WiFi.status() == WL_CONNECTED){
        wifi_connected = true;
        break;
      }else{
        delay(500);
      }
    }
  }
  
  if(wifi_connected){
    if(eeprom_mqttServer != ""){
      mqttClient.setServer(eeprom_mqttServer.c_str(), mqttport);
      mqttClient.setCallback(mqtt_callback);

      // Init Steppers
      pinMode(pin_stepper_sleep, OUTPUT);
      digitalWrite(pin_stepper_sleep, LOW); // LOW = sleep, HIGH = active
      driver_active = false;
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        send_state_update[i] = true;
        stepper[i] = AccelStepper (AccelStepper::DRIVER, pin_stepper_step[i], pin_stepper_dir[i]);
        unsigned int new_speed = 1600;
        stepper[i].setMaxSpeed(new_speed);
//        mqttClient.publish(mqtttopic_speed_get,String(new_speed).c_str());
        stepper[i].setAcceleration(1000);
        pinMode(pin_stepper_reed[i], INPUT);
        findPositionZero(i);
      }
    }
  }else{
    setupAP();
  }

  startWebServer();
}


//////////////////////////
// Loop
//////////////////////////
void loop() {
  if(wifi_connected){
    if(!driver_active){
      // Do MQTT only if steppers are not running
      if(!mqttClient.connected()) {
        mqtt_reconnect();
      }else {    
        mqttClient.loop(); // Do MQTT
      }
      // Deaktivate Stepper drivers
      digitalWrite(pin_stepper_sleep, LOW); // LOW = sleep, HIGH = active
    }
    
    driver_active = false;
    for(byte i=0; i<STEPPER_COUNT; i++){
      if(stepper[i].distanceToGo() != 0){
        // Activate the stepper driver just for movements
        driver_active = true;
        send_state_update[i] = true;
        digitalWrite(pin_stepper_sleep, HIGH); // LOW = sleep, HIGH = active
      }else if(!driver_active && send_state_update[i]){
        // Send Update of current position
        send_state_update[i] = false;
        const char* cur_pos = String(getPosition(i)).c_str();
        mqttClient.publish(mqtttopic_state[i],cur_pos);
      }

      // Stop at reed-Position
      if(!leave_reed_position[i] && stepper[i].currentPosition() != 0 && digitalRead(pin_stepper_reed[i]) == LOW){
        stepper[i].stop();
        stepper[i].setSpeed(0);
        stepper[i].setCurrentPosition(0);
      }

      // Reset "leave_reed_position" if "time_to_leave" is reached
      if(leave_reed_position[i] && digitalRead(pin_stepper_reed[i]) == HIGH && millis() > (time_leaving[i]+time_to_leave_reed)) leave_reed_position[i] = false;

      // Do pending jobs
      stepper[i].run();
    }
  }
  server.handleClient(); // Webserver
}

//////////////////////////
// MQTT
//////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char msg[length];
  for (int i = 0; i < length; i++) {
    msg[i] = payload[i];
  }

  if(String(topic) == String(mqtttopic_all_cmd)){
    String cmd = String(msg);
    if(cmd.startsWith("on")){
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        goToPosition(i,100);
      }
    }else if(cmd.startsWith("off")){
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        goToPosition(i,0);
        //findPositionZero(i);
      }
    }else if(cmd.startsWith("stop")){
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        stepper[i].stop();
        stepper[i].setSpeed(0);
        stepper[i].moveTo(stepper[i].currentPosition());
        send_state_update[i] = true;
      }
    }
/*  }else if(String(topic) == String(mqtttopic_speed_set)){
    unsigned int new_speed = String(msg).toInt();
    for(uint8_t s=0; s<STEPPER_COUNT; s++){
      stepper[s].setMaxSpeed(new_speed);
    }
    mqttClient.publish(mqtttopic_speed_get,String(new_speed).c_str());
*/
  }else{
    for(uint8_t i=0; i<STEPPER_COUNT; i++){
      if(String(topic) == String(mqtttopic_abs[i])){
        float new_pos = String(msg).toFloat();
        if(new_pos > 0 && new_pos < 100){
          goToPosition(i,new_pos);
        }
      }else if(String(topic) == String(mqtttopic_cmd[i])){
        String cmd = String(msg);
        if(cmd.startsWith("on")){
          goToPosition(i,100);
        }else if(cmd.startsWith("off")){
          findPositionZero(i);
        }else if(cmd.startsWith("stop")){
          stepper[i].stop();
          stepper[i].setSpeed(0);
          stepper[i].moveTo(stepper[i].currentPosition());
          send_state_update[i] = true;
        }
      }
    }
  }
}

void mqtt_reconnect() {
//  while (!mqttClient.connected()) {

    if (mqttClient.connect("Rollo")) {
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        mqttClient.subscribe(mqtttopic_cmd[i]);
        mqttClient.subscribe(mqtttopic_abs[i]);
      }
//      mqttClient.subscribe(mqtttopic_speed_set);
      mqttClient.subscribe(mqtttopic_all_cmd);
    } else {
      delay(5000); // Wait 5 seconds before retrying
    }
//  }
}

//////////////////////////
// Stepper
//////////////////////////
void goToPosition(uint8_t stepper_nr, float pos_in_percent){
  float pos_to_go = max_steps[stepper_nr] * pos_in_percent / 100;
  stepper[stepper_nr].moveTo(pos_to_go);
  send_state_update[stepper_nr] = true;
  if(pos_in_percent > 5) leave_reed_position[stepper_nr] = true;
  time_leaving[stepper_nr] = millis();
}

int getPosition(uint8_t stepper_nr){
  float pos = String(stepper[stepper_nr].currentPosition()).toFloat()/max_steps[stepper_nr]*100;
  if(pos < 0) pos = pos*-1;
  pos += 0.01; // Fix to get the correct integer value from casting
  return (int)pos;
}

void findPositionZero(uint8_t stepper_nr){
  if(digitalRead(pin_stepper_reed[stepper_nr]) == LOW){
    stepper[stepper_nr].setCurrentPosition(0);
  }else{
    int pos_to_go;
    if(max_steps[stepper_nr] < 0){
      pos_to_go = (max_steps[stepper_nr]*-1)+1000;
    }else{
      pos_to_go = (max_steps[stepper_nr]*-1)-1000;
    }
    stepper[stepper_nr].moveTo(pos_to_go);
  }
  send_state_update[stepper_nr] = true;
}

///////////////
// WiFi Code //
///////////////
void setupAP(void){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  delay(100);
  
  int n = WiFi.scanNetworks();

  scanned_wifis = "<ol>";
  for (int i = 0; i < n; i++){
    scanned_wifis += "<li><a href='#' onClick=\"document.getElementById('ssid').value = '";
    scanned_wifis += WiFi.SSID(i);
    scanned_wifis += "'\">";
    scanned_wifis += WiFi.SSID(i);
    scanned_wifis += "</a></li>";
  }
  scanned_wifis += "</ol>";
  
  delay(100);
  
  WiFi.softAP("Rollo","");
}

void startWebServer(){
  server.on("/", []() {
    IPAddress ip = WiFi.softAPIP();
    html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Rollo</title></head><body>";
    html += "<h3>Rollo v";
    html += VERSION;
    html += "</h3>";
    html += "<p>";
    html += scanned_wifis;
    html += "</p><form method='post' action='save'><table>";
    html += "<tr><td>WLAN Name:</td><td><input name='ssid' id='ssid' value='";
    html += eeprom_ssid;
    html += "'></td></tr>";
    html += "<tr><td>WLAN Passwort:</td><td><input name='pass' type='password' value='";
    if(eeprom_pass.length() > 0) html += "***";
    html += "'></td></tr>";
    html += "<tr><td>MQTT Server:</td><td><input name='mqttServer' value='";
    html += eeprom_mqttServer;
    html += "'></td></tr>";
    html += "<tr><td><input type='submit' value='speichern'></form></td>";
    html += "<td><form method='post' action='clear' style='display:inline'><input type='submit' value='zur&uuml;cksetzen'></form></td></tr>";
    html += "</table></body></html>";
    server.send(200, "text/html", html);
  });
  server.on("/save", []() {
    String new_ssid = server.arg("ssid");
    String new_pass = server.arg("pass");
    String new_mqttServer = server.arg("mqttServer");
    
    for (int i = 0; i < 501; ++i) { EEPROM.write(i, 0); } // Clear EEPROM

    // EEPROM: SSID
    for (int i = 0; i < new_ssid.length(); i++){
      EEPROM.write(i, new_ssid[i]);
    }

    // EEPROM: Password
    if(new_pass == "***") new_pass = eeprom_pass;   
    for (int i = 0; i < new_pass.length(); i++){
      EEPROM.write(32+i, new_pass[i]);
    }    

    // EEPROM: mqttServer
    for (int i = 0; i < new_mqttServer.length(); i++){
      EEPROM.write(96+i, new_mqttServer[i]);
    }

    EEPROM.write(500,42); // Config Written = yes
    EEPROM.commit();

    html = "<html><head><title>Rollo</title><meta http-equiv='refresh' content='10; url=/'></head><body>";
    html += "<h3>Rollo</h3>";
    html += "Einstellungen gespeichert - starte neu..<br>";
    html += "(wenn es nicht funktioniert, bitte hart restarten (Stecker ziehen))";
    html += "</body></html>";
    server.send(200, "text/html", html);
    delay(100);
    ESP.restart();
  });

  server.on("/clear", []() {
    for (int i = 0; i < 501; i++) { 
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
    
    html = "<html><head><title>Rollo</title><meta http-equiv='refresh' content='10; url=/'></head><body>";
    html += "<h3>Rollo</h3>";
    html += "Einstellungen zur&uuml;ckgesetzt - starte neu..<br>";
    html += "(wenn es nicht funktioniert, bitte hart restarten (Stecker ziehen))";
    html += "</body></html>";
    server.send(200, "text/html", html);
    delay(100);
    ESP.restart();
  });

  server.begin();
}
