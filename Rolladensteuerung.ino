// TODO: encapsulate stepper with "struct"
// TODO: MQTT/WLAN/Stepper functions in extra files

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <PubSubClient.h> // MQTT - if you get rc=-4 you should maybe change MQTT_VERSION in library
#include <AccelStepper.h>

const int connect_wait = 20;

// Pin-Zuordnung
#define STEPPER_COUNT 2
const uint8_t pin_stepper_dir[]   = { 2,14};
const uint8_t pin_stepper_step[]  = { 4,12};
const uint8_t pin_stepper_reed[]  = {16,13};
const uint8_t pin_stepper_sleep   = 5;


const char* ssid;
const char* password;
const char* mqttserver = "192.168.42.4";
uint16_t mqttport = 1883;
const char* mqtttopic_state[] = {"fhem/torsten/rollo0/state","fhem/torsten/rollo1/state"};
const char* mqtttopic_cmd[] = {"fhem/torsten/rollo0/cmd","fhem/torsten/rollo1/cmd"};
const char* mqtttopic_abs[] = {"fhem/torsten/rollo0/abs","fhem/torsten/rollo1/abs"};
ESP8266WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String network_list = "";
boolean ap_mode = true;
boolean mqtt_configured = false;
boolean leave_reed_position = false;
unsigned int time_to_leave_reed = 1000;
unsigned long time_leaving;

AccelStepper stepper[STEPPER_COUNT];
boolean driver_active;
boolean send_state_update = false;

int max_steps[] = {
  (1048*16),
  (-1048*18)
};

//////////////////////////
// CONFIG File
//////////////////////////
bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  ssid = json["ssid"];
  password = json["password"];

  Serial.print("Loaded SSID: ");
  Serial.print("|");
  Serial.print(ssid);
  Serial.println("|");
  Serial.print("Loaded Password: ");
  Serial.print("|");
  Serial.print(password);
  Serial.println("|");
  return true;
}

bool saveConfig(JsonObject& json) {
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  return true;
}


//////////////////////////
// Configuration AP / Station
//////////////////////////
void scan_networks(bool save_list){
  Serial.println("Scanning for Networks ..");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.println("scan start");
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0){
    Serial.println("no networks found");
  }else{
    Serial.print(n);
    Serial.println(" networks found");
    if(save_list) network_list = "<ul>";
    for (int i = 0; i < n; ++i){
      // Print SSID and RSSI for each network found
      if(save_list) network_list += "<li><a href='/?ssid="+WiFi.SSID(i)+"'>";
      if(save_list) network_list += WiFi.SSID(i);
      if(save_list) network_list += " (";
      if(save_list) network_list += WiFi.RSSI(i);
      if(save_list) network_list += ")";
      if(save_list) network_list += (WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*";
      if(save_list) network_list += "</a></li>";
      
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      delay(10);
    }
    if(save_list) network_list += "</ul>";
  }
  Serial.println("");
}

void setup_ap(){
  scan_networks(true);
  
  String ssid_string = "ESP8266_"+WiFi.macAddress().substring(9);
  char config_ssid[25];
  ssid_string.toCharArray(config_ssid,25);
  
  Serial.println("Configuring access point...");
  Serial.print("SSID: ");
  Serial.println(config_ssid);
  WiFi.softAP(config_ssid);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  ssid = "";
  password = "";
}

bool setup_station(){
  //scan_networks(false);

  Serial.print("Connect to WiFi ");
  Serial.print(ssid);
  Serial.print(":");
  Serial.print(password);
  Serial.println(" ...");
  WiFi.begin(ssid, password);

  // Wait for connection
  int retry_count = 0;
  while(retry_count < connect_wait && WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    retry_count++;
  }

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Connection failed ..");
    return false;
  }else{
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  }
}


//////////////////////////
// WebServer
//////////////////////////
void handleRoot(){
  String message = "<html><head><title>ESP8266 "+WiFi.macAddress()+"</title></head><body>";
  message += "<h3>ESP8266 "+WiFi.macAddress()+"</h3>";

  // List of available APs
  message += "<h4>Available Networks</h4>";
  message += network_list;
  message += "NOTE: List is only updated on boot of ESP8266<br><br>";

  // Form for settings
  message += "<form action='/save' method='POST'><table>";
  message += "<tr><td><b>SSID:</b></td><td><input type='text' name='ssid' value='"+server.arg("ssid")+"'></td></tr>";
  message += "<tr><td><b>Password:</b></td><td><input type='text' name='password' value=''></td></tr>";
  message += "<tr><td colspan='2' align='center'><input type='submit' value='save'></td></tr>";
  message += "</table></form>";
  
  message += "</body></html>";
  server.send(200, "text/html", message);
}

void handleSave(){
  String message = "<html><head><title>ESP8266 "+WiFi.macAddress()+"</title></head><body>";
  message += "<h3>ESP8266 "+WiFi.macAddress()+"</h3>";
  
  if(server.method() == HTTP_POST && server.arg("ssid") != ""){
    Serial.println("--- New Settings ---");
    String new_ssid = server.arg("ssid");
    String new_password = server.arg("password");

    Serial.print("New SSID: ");
    Serial.println(new_ssid);
    Serial.print("New Password: ");
    Serial.println(new_password);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["ssid"] = new_ssid;
    json["password"] = new_password;
          
    if(!saveConfig(json)){
      Serial.println("Failed to save config");
      message += "Settings could not be saved";
    }else{
      Serial.println("Config saved");
      message += "Settings saved successfully - please reset me";
    }
  }else{
    message += "You told me something I don't understand ..";
  }

  message += "</body></html>";
  server.send(200, "text/html", message);
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
  
  Serial.println("--- [404] ---");
  Serial.println(message);
  Serial.println("------");
}

//////////////////////////
// MQTT
//////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg[i] = payload[i];
  }
  Serial.print("/");
  Serial.print(msg);
  Serial.print(" (");
  Serial.print(length);
  Serial.println(")");

  for(uint8_t i=0; i<STEPPER_COUNT; i++){
    if(String(topic) == String(mqtttopic_abs[i])){
      float new_pos = String(msg).toFloat();
      Serial.print("Stepper ");
      Serial.print(i);
      Serial.print(" - abs: ");
      if(new_pos < 0 || new_pos > 100) Serial.println("value out of range");
      else{
        Serial.println("move");
        goToPosition(i,new_pos);
      }
    }else if(String(topic) == String(mqtttopic_cmd[i])){
      String cmd = String(msg);
      Serial.print("Stepper ");
      Serial.print(i);
      Serial.print(" - cmd: ");
      if(cmd.startsWith("on")){
        Serial.println("move to 100");
        goToPosition(i,100);
      }else if(cmd.startsWith("off")){
        Serial.println("move and find pos zero");
        findPositionZero(i);
      }else if(cmd.startsWith("stop")){
        Serial.println("stop");
        stepper[i].stop();
        stepper[i].setSpeed(0);
        stepper[i].moveTo(stepper[i].currentPosition());
        send_state_update = true;
      }else Serial.println("command not known");
    }
  }
}

void mqtt_reconnect() {
  String client_name = "ESP8266Client_"+WiFi.macAddress();
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    if (mqttClient.connect(client_name.c_str())) {
      Serial.println("connected");
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        mqttClient.subscribe(mqtttopic_cmd[i]);
        mqttClient.subscribe(mqtttopic_abs[i]);
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

//////////////////////////
// Stepper
//////////////////////////
void goToPosition(uint8_t stepper_nr, float pos_in_percent){
  float pos_to_go = max_steps[stepper_nr] * pos_in_percent / 100;
  Serial.print("Move Stepper ");
  Serial.print(stepper_nr);
  Serial.print(" to ");
  Serial.println(pos_to_go);
  stepper[stepper_nr].moveTo(pos_to_go);
  send_state_update = true;
  if(pos_in_percent > 5) leave_reed_position = true;
  time_leaving = millis();
}

int getPosition(uint8_t stepper_nr){
  float pos = String(stepper[stepper_nr].currentPosition()).toFloat()/max_steps[stepper_nr]*100;
  if(pos < 0) pos = pos*-1;
  pos += 0.01; // Fix to get the correct integer value from casting
  return (int)pos;
}

void findPositionZero(uint8_t stepper_nr){
  if(digitalRead(pin_stepper_reed[stepper_nr]) == LOW){
    Serial.print("Stepper ");
    Serial.print(stepper_nr);
    Serial.println(" in position ..");
    stepper[stepper_nr].setCurrentPosition(0);
  }else{
    Serial.print("Stepper ");
    Serial.print(stepper_nr);
    Serial.println(" searching position ..");
    int pos_to_go;
    if(max_steps[stepper_nr] < 0){
      pos_to_go = (max_steps[stepper_nr]*-1)+1000;
    }else{
      pos_to_go = (max_steps[stepper_nr]*-1)-1000;
    }
    stepper[stepper_nr].moveTo(pos_to_go);
  }
  send_state_update = true;
}


//////////////////////////
// Setup
//////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println("");
  delay(1000);
  
  // Load Config and Connect to WiFi / setup AP
  Serial.println("Mounting FS...");
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }
  if (!loadConfig()) {
    Serial.println("Failed to load config");
    setup_ap();
  } else {
    Serial.println("Config loaded");
    if(!setup_station()){
      setup_ap();
    }else ap_mode = false;
  }

  // Setup Webserver
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  // Setup MQTT
  if(!ap_mode){
    if(mqttserver == "" || mqttport == 0){
      Serial.println("MQTT not configured ..");
    }else{
      Serial.print("Configure MQTT Server: ");
      Serial.print(mqttserver);
      Serial.print(":");
      Serial.println(mqttport);
      mqttClient.setServer(mqttserver, mqttport);
      mqttClient.setCallback(mqtt_callback);
      mqtt_configured = true;

      Serial.println("Initiate Steppers ..");
      pinMode(pin_stepper_sleep, OUTPUT);
      digitalWrite(pin_stepper_sleep, LOW); // LOW = sleep, HIGH = active
      driver_active = false;
      send_state_update = true;
      for(uint8_t i=0; i<STEPPER_COUNT; i++){
        stepper[i] = AccelStepper (AccelStepper::DRIVER, pin_stepper_step[i], pin_stepper_dir[i]);
        stepper[i].setMaxSpeed(3000);
        stepper[i].setAcceleration(2000);
        pinMode(pin_stepper_reed[i], INPUT);
        findPositionZero(i);
      }
    }
  }
}


//////////////////////////
// Loop
//////////////////////////
void loop() {
  if(!ap_mode && mqtt_configured){
    if(!mqttClient.connected()) {
      mqtt_reconnect();
    }
    
    mqttClient.loop();

    driver_active = false;
    for(byte i=0; i<STEPPER_COUNT; i++){
      // Activate the stepper driver just for movements
      if(stepper[i].distanceToGo() != 0){
        driver_active = true;
        send_state_update = true;
        digitalWrite(pin_stepper_sleep, HIGH); // LOW = sleep, HIGH = active
      }

      if(!leave_reed_position && stepper[i].currentPosition() != 0 && digitalRead(pin_stepper_reed[i]) == LOW){
        stepper[i].stop();
        stepper[i].setSpeed(0);
        stepper[i].setCurrentPosition(0);
      }

      if(leave_reed_position && digitalRead(pin_stepper_reed[i]) == HIGH && millis() > (time_leaving+time_to_leave_reed)) leave_reed_position = false;

      // Do pending jobs
      stepper[i].run();
    }
    if(!driver_active && send_state_update){
      digitalWrite(pin_stepper_sleep, LOW); // LOW = sleep, HIGH = active
      send_state_update = false;
      
      for(byte i=0; i<STEPPER_COUNT; i++){
        const char* cur_pos = String(getPosition(i)).c_str();
        Serial.print("Stepper ");
        Serial.print(i);
        Serial.print(" now at Pos: ");
        Serial.println(cur_pos);
        mqttClient.publish(mqtttopic_state[i],cur_pos);
      }
    }
  }else{
    server.handleClient();
  }
}
