//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// ACKNOWLDEGMENTS

/*
 * JAMIE CHURCHOUSE
 * HYGEIA MK1 FIRMWARE V4.8 2022-10-25
 * 
 * This program is the firmware for our third year mechatronics project at Massey University Palmerston North.
 * The project is a robotic vaccum cleaner system with suction and wireless comunication.
 * 
 * This program is communicated with by an app via websocket on the local network.
 * 
 * The controller app can be found here:
 * https://play.google.com/store/apps/details?id=appinventor.ai_jamespeterchurchouse.MythicalManufacturingControllerV2_4
 * 
 */


/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/


//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// CHANGELOG

/* V4.8
 *  New "UV" lighting scheme
 *  New "Massey" lighting scheme
 *  New "Massey" sound file
 *  
 *  
 *  V4.7
 *  Removed crash sensor functions (the sensors weren't installed in time)
 *  Updated LED settings
 *  
 *  V4.6
 * Added comments
 * Implemented crash sensor functions
 * 
 * V4.5
 * Full working program
 */

//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// INCLUDE LIBRARIES

// Load WiFi library
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Load MP3 libraries
#include "HardwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//RGB LED strip
#include <Adafruit_NeoPixel.h>


//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// PIN DEFINITIONS

// Drive Motor pins
const uint8_t pinMotorLeftFor     = 26;
const uint8_t pinMotorLeftRev     = 27;
const uint8_t pinMotorRightRev    = 33;
const uint8_t pinMotorRightFor    = 25;

// Drive Motor PWM channels
const uint8_t chnlLeftFor         = 0; 
const uint8_t chnlLeftRev         = 1; 
const uint8_t chnlRightFor        = 2; 
const uint8_t chnlRightRev        = 3; 

// Sensor pins
//const uint8_t pinEncoderLeft      = 34;
//const uint8_t pinEncoderRight     = 35;
const uint8_t pinCrashSensorLeft  = 36;
const uint8_t pinCrashSensorRight = 39;
const uint8_t pinCliffSensor      = 32;

// Suction system pins
const uint8_t pinSuctionEnable    = 21;
const uint8_t pinSweepEnable      = 18;

// LED pins
const uint8_t pinLeds             = 04;

//Sound
const uint8_t pinSoundEnable      = 05;


//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Global Variables

// Left and Right motor duty cycles
// speed% = input - 100 (for inputs 0-200), 100 is disabled
volatile unsigned uLeftMotorSpeed   = 100;
volatile unsigned uRightMotorSpeed  = 100;
volatile unsigned ubSuctionEnabled  = 0;

// Flag for new data to be processed
volatile bool bNewDataFlag = false;

// Variables to log the time a sensor is tripped
volatile unsigned uCrashTimeLeft    = 0;
volatile unsigned uCrashTimeRight   = 0;
volatile unsigned uCliffTime        = 0;

// Strings for websocket comms
String message = "";
String sitrep = "";
void vSitrep(){// Situation report generator function
  Serial.println("sitrep updated to: ");
  sitrep = "";
  sitrep += "L: ";
  sitrep += ((int)uLeftMotorSpeed - 100);
  sitrep += " R: ";
  sitrep += ((int)uRightMotorSpeed - 100);
  sitrep += " S: ";
  sitrep += ubSuctionEnabled;
    Serial.println(sitrep);
}
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// DRIVE VARS AND FUNCS

// speed% = input - 100 (0-200)
// 200in -> 100%
// 100in -> 0%
// 0in -> -100%
// therefore: 101-200in is foreward 1-100%, 0-99in is backwards 100-1%
// Function to update the motor duty cycles
void vUpdateDrive(){
  Serial.println("Begin UpdateDrive");

  // Set left motor duty
  if(uLeftMotorSpeed > 100 && uLeftMotorSpeed <= 200){      //Forward
    ledcWrite(chnlLeftFor, (uLeftMotorSpeed - 100) * 2.55);
    ledcWrite(chnlLeftRev, 0);
    Serial.print("LeftMotFor:");
    Serial.println((uLeftMotorSpeed - 100));
  }else if(uLeftMotorSpeed >= 0 && uLeftMotorSpeed < 100){  //Backward
    ledcWrite(chnlLeftRev, (100 - uLeftMotorSpeed) * 2.55 );
    ledcWrite(chnlLeftFor, 0);
    Serial.print("LeftMotRev:");
    Serial.println((100 - uLeftMotorSpeed));
  }else{                                                    //Stop
    ledcWrite(chnlLeftFor, 0);
    ledcWrite(chnlLeftRev, 0);
    Serial.println("LeftMotStop");
  }
  // Set right motor duty
  if(uRightMotorSpeed > 100 && uRightMotorSpeed <= 200){      //Forward
    ledcWrite(chnlRightFor, (uRightMotorSpeed - 100) * 2.55);
    ledcWrite(chnlRightRev, 0);
    Serial.print("RightMotFor:");
    Serial.println((uRightMotorSpeed - 100));
  }else if(uRightMotorSpeed >= 0 && uRightMotorSpeed < 100){  //Backward
    ledcWrite(chnlRightRev, (100 - uRightMotorSpeed) * 2.55);
    ledcWrite(chnlRightFor, 0);
    Serial.print("RightMotRev:"); 
    Serial.println((100 - uRightMotorSpeed));
  }else{                                                      //Stop
    ledcWrite(chnlRightFor, 0);
    ledcWrite(chnlRightRev, 0);
    Serial.println("RightMotStop");
  }

  // Set suction motor duty
  digitalWrite(pinSuctionEnable, ubSuctionEnabled);
  digitalWrite(pinSweepEnable, ubSuctionEnabled);
  Serial.print("Suction:");
  Serial.println((ubSuctionEnabled));

  Serial.println("End UpdateDrive");
}

//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// RGB LEDs

// LED strip variables
volatile int iLedType     = 0;  //LED colour mode
unsigned uLastLedUpdate   = 0;  //Variable for the time the LEDs were last updated
const uint8_t uNumLeds    = 77; //Number of LEDs in the strip

// Create LED strip object
Adafruit_NeoPixel pixels(uNumLeds, pinLeds, NEO_GRB + NEO_KHZ800);

// String to send to app that states what modes are available
const char* pcAvailableLights = "Rainbow,Ambo,Hazard,Police,UV,Massey";

// Function to update the LED strip
void vRunLeds(unsigned phase){
  if(iLedType == 0){//Off
    pixels.clear();
    for(int i = 0; i < uNumLeds; i++){
      pixels.setPixelColor(i, pixels.Color(0,0,0));
    }
    pixels.show();
    return;
    
  }else if(iLedType == 1){//Rainbow
    pixels.clear();

    //For each LED...
    for(int i = 0; i < uNumLeds; i++){
      //...generate a number between 0 and 767 based on time and LED index...
      unsigned p = ((20*i)+phase)%768;//This line covers all the settings
      //...and set the colour of the LED to the corresponding value in the cycle.
      if(p < 256){
        pixels.setPixelColor(i, pixels.Color(p,0,255-p));
      }else if(p < 512){
        pixels.setPixelColor(i, pixels.Color(511-p,p-256,0));
      }else{
        pixels.setPixelColor(i, pixels.Color(0,767-p,p-512));
      }
    }
    pixels.show();
    return;

  }else if(iLedType == 2){//Ambo   
    if(phase%5){//Only runs every 80 ms (20 Hz)
      return;
    }
    unsigned p = (phase/5) % 16;//16 states per cycle, 4 red, 4 white, 8 off
    pixels.clear();
    
    if(!(p%2)){
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0));
      }
    }else if(p < 8){
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(255,0,0));
      }
    }else{
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(85, 85, 85));
      }
    }
    pixels.show();
    return;
    
  }else if(iLedType == 3){//Hazard     
    if(phase%50){//Only runs every 300 ms (2 Hz)
      return;
    }
    pixels.clear();
    unsigned p = (phase / 50) % 2;//two states per cycle, on and off

    if(p){
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(255,50,0));
      }
    }else{
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0));
      }
    }
    pixels.show();
    return;
  }else if(iLedType == 4){//Police    
    if(phase%5){//Only runs every 80 ms (20 Hz)
      return;
    }
    unsigned p = (phase/5) % 16;//16 states per cycle, 4 red, 4 blue, 8 off
    pixels.clear();
    
    if(!(p%2)){
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0));
      }
    }else if(p < 8){
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(255,0,0));
      }
    }else{
      for(int i = 0; i < uNumLeds; i++){
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
      }
    }
    pixels.show();
    return;
    
  }else if(iLedType == 5){//UV
    pixels.clear();

    //For each LED...
    for(int i = 0; i < uNumLeds; i++){
      pixels.setPixelColor(i, pixels.Color(158,0,255));
    }
    pixels.show();
    return;

  }else if(iLedType == 6){//Massey
    pixels.clear();

    //For each LED...
    for(int i = 0; i < uNumLeds; i++){
      if( (i > 8 && i < 20) || (i > 40 && i < 52)){
        pixels.setPixelColor(i, pixels.Color(226,154,12));
      }else{
        pixels.setPixelColor(i, pixels.Color(17,68,140));
      }
    }
    pixels.show();
    return;

  }
}

// Function to set a specific LED a specifc RGB
void vSetLed(int pos, int R, int G, int B){
  pixels.clear();
  pixels.setPixelColor(pos, pixels.Color(R,G,B));
  pixels.show();
  return;
}
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// MP3 PLAYER

// String to send to app regarding what sounds are available
const char* pcAvailableSounds = "Alarm,AxelF,Dream,MoveIt,JamesBond,Mission,NunCat,Imperial,TwoTone,Wail,Yelp,Horn,*Fire*,Thrones,Massey,Travis";

// Hardware serial object "mp3Player" using "mp3Serial"
HardwareSerial mp3Serial(1);
DFRobotDFPlayerMini mp3Player;
// Function to print details from the DFplayer module (from example code)
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

// Variable to store the currently playing sound
volatile int iNowPlaying = 0;

// Function to set the play sound
void vPlaySound(int value){
  //If not 0...
  if(value){
    //...enable applifier and play the sound...
    digitalWrite(pinSoundEnable, HIGH);
    mp3Player.play(value);
    mp3Player.enableLoop();
  }else{
    //...otherwise disable the amplifier and stop the player.
    digitalWrite(pinSoundEnable, LOW);
    mp3Player.stop();
  }
  return;
}

        
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// WIFI VARS AND FUNCS

// POC WIFI credentials
// Enable soft AP mode (for use without the router)
bool SOFTAPMODE = false;

// WIFI AP credentials (hard coded for the POC)
const char* ssid     = "WAP";
const char* password = "Password";
/*
const char* ssid     = "Lodge Wireless Internet";
const char* password = "JulietCharlieHotelQuebec";
*/

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Send SitRep string to all clients
void notifyClients() {
  ws.textAll(sitrep);
}


// MAJOR FUNCTION
// Function to handle Websocket messages (adapted from example)
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  Serial.println("Handle websocket message reached");
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    Serial.println(message);

    //Find keywords
    int posL = message.indexOf("L=");
    int posR = message.indexOf("R=");
    int posS = message.indexOf("S=");
    int posV = message.indexOf("V=");
    int posA = message.indexOf("A=");
    int posX = message.indexOf("STOP");

     //If there is a left drive command...
    if (posL >= 0) {
      //...interperate the value and set the duty...
      Serial.print("LEFT ");
      int pos_ = message.indexOf("&",posL);
      if(posL+2 < pos_)
        uLeftMotorSpeed = message.substring(posL+2,pos_).toInt();
      Serial.println(uLeftMotorSpeed);
      //...and set the new data flag.
      bNewDataFlag = true;
    }
    if (posR >= 0) {
      Serial.print("RIGHT ");
      int pos_ = message.indexOf("&",posR);
      if(posR+2 < pos_)
        uRightMotorSpeed = message.substring(posR+2,pos_).toInt();
      Serial.println(uRightMotorSpeed);
      bNewDataFlag = true;
    }
    if (posS >= 0) {
      Serial.println("SUCK");
      int pos_ = message.indexOf("&",posS);
      if(posS+2 < pos_)
        ubSuctionEnabled = message.substring(posS+2,pos_).toInt();

      bNewDataFlag = true;
    }//iLedType
    if (posV >= 0) {
      Serial.print("VISUAL ");
      int pos_ = message.indexOf("&",posV);
      int index = 0;
      if(posV+2 < pos_)
        index = message.substring(posV+2,pos_).toInt();
        switch (index){
          case 0: iLedType = 0; Serial.println("off");break;
          case 1: iLedType = 1; Serial.println("rain");break;
          case 2: iLedType = 2; Serial.println("ambo");break;
          case 3: iLedType = 3; Serial.println("haz");break;
          case 4: iLedType = 4; Serial.println("popo");break;
          case 5: iLedType = 5; Serial.println("uv");break;
          case 6: iLedType = 6; Serial.println("mass");break;
          default: iLedType = 0;Serial.println("off");break;
        }
        Serial.println("LED type updated");
    }
    if (posA >= 0) {
      Serial.print("AUDIO ");
      int pos_ = message.indexOf("&",posA);
      int index = 0;
      if(posA+2 < pos_)
        index = message.substring(posA+2,pos_).toInt();
      iNowPlaying = index;
      vPlaySound(iNowPlaying);
      Serial.println("Audio play");
    }
    //If the stop command is received...
    if (posX >= 0) {
      // Stop all periferals
      vAllStop();
      Serial.println("STOPPING");
      bNewDataFlag = false;
    }
    vSitrep();
    Serial.println("Sitrep: " + sitrep);
    notifyClients();
  }
}

// On Websocket Event funciton (adapted from example)
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  Serial.println("On Event reached");
              
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      vAllStop();
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Function to initalise the websocket (adapted from example)
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  Serial.println("init websocket");
}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// MAIN FUNCTIONS

// Function to stop all periferals
void vAllStop(){
  uLeftMotorSpeed = 100;
  uRightMotorSpeed = 100;
  ubSuctionEnabled = 0;
  bNewDataFlag = true;
  vPlaySound(0);
  iLedType = 0;
}

// Function to check sensors
void vCheckSensors(){
  //pinCrashSensorLeft    uCrashTimeLeft
  //pinCrashSensorRight   uCrashTimeRight
  //pinCliffSensor        uCliffTime
  unsigned uTimeNow = millis();
  
  uCrashTimeLeft  = digitalRead(pinCrashSensorLeft)   ? uTimeNow : 0 ;
  uCrashTimeRight = digitalRead(pinCrashSensorRight)  ? uTimeNow : 0 ;
  uCliffTime      = !digitalRead(pinCliffSensor)      ? uTimeNow : 0 ;

  if(uCrashTimeLeft || uCrashTimeRight || uCliffTime){
    Serial.println("CRASH DETECTED");
    uLeftMotorSpeed   = 100;
    uRightMotorSpeed  = 100;
    ubSuctionEnabled  = 0;
    bNewDataFlag = true;
  }
  return;
}
// Setup function
void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("Beginning program");
  
  //LED strip
  Serial.print("LEDs: ");
  pixels.begin();  
  Serial.println("Initalised");
  vSetLed(0, 0, 0, 255);
  
  // Setup Drive Motor PWM
  Serial.print("Motors: ");
  ledcAttachPin(pinMotorLeftFor, chnlLeftFor);
  ledcAttachPin(pinMotorLeftRev, chnlLeftRev);
  ledcAttachPin(pinMotorRightFor, chnlRightFor);
  ledcAttachPin(pinMotorRightRev, chnlRightRev);

  ledcSetup(chnlLeftFor, 1000, 8);
  ledcSetup(chnlLeftRev, 1000, 8);
  ledcSetup(chnlRightFor, 1000, 8);
  ledcSetup(chnlRightRev, 1000, 8);

  // Suction motor setup
  pinMode(pinSuctionEnable, OUTPUT);
  pinMode(pinSweepEnable, OUTPUT);
  pinMode(pinSoundEnable, OUTPUT);
  
  digitalWrite(pinSuctionEnable, LOW);
  digitalWrite(pinSweepEnable, LOW);
  digitalWrite(pinSoundEnable, LOW);

  Serial.println("Initalised");
  vSetLed(0, 255, 255, 0);
  
  //MP3
  Serial.print("Player: ");
  mp3Serial.begin(9600, SERIAL_8N1, 16, 17);
  if (!mp3Player.begin(mp3Serial)) {
    Serial.println("ERROR");
    vSetLed(0, 255, 0, 0);
    while(true);
  }
  mp3Player.volume(15);  //Set volume value. From 0 to 30
  //mp3Player.disableLoop();
  //mp3Player.disableLoopAll();
  mp3Player.enableLoop();
  mp3Player.stop();
  Serial.println("Initalised");
  vSetLed(0, 0, 255, 0);

  // Sensors
  //pinMode(pinEncoderLeft, INPUT);
  //pinMode(pinEncoderRight, INPUT);
  pinMode(pinCrashSensorLeft, INPUT);
  pinMode(pinCrashSensorRight, INPUT);
  pinMode(pinCliffSensor, INPUT);

  // Setup wifi and websocket
  Serial.println("WIFI: "); 

  if(SOFTAPMODE){
    WiFi.softAP(ssid, password);
    Serial.println(WiFi.softAPIP());
  }else{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi..");
    }
    // Print ESP Local IP Address
    Serial.println(WiFi.localIP());
  }
  
  initWebSocket();

  // On "/Searching", send the bot type back
  server.on("/Searching", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/Searching");
    request->send(200, "text/plain", "HygeiaMk1");
  });

  // On "/Lights", send the light options back
  server.on("/Lights", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/Lights");
    request->send(200, "text/plain", pcAvailableLights);
  });

  // On "/Sounds", send the sound options back
  server.on("/Sounds", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/Sounds");
    request->send(200, "text/plain", pcAvailableSounds);
  });


  // Start server
  server.begin();
  vSetLed(0, 255, 255, 255);
  Serial.println("\nSetup complete!");
}

// Main loop
void loop() {

  //While the WIFI is disconnecred
  while (WiFi.status() != WL_CONNECTED) {
    vSetLed(0, 255, 0, 0);
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  //Websocket function
  ws.cleanupClients();

  //Check sensors
  //vCheckSensors();

  // If new drive data is available
  if(bNewDataFlag){
    vUpdateDrive();
    bNewDataFlag = false;
  }

  // Update lights if time is right
  unsigned uTimeNow = millis();
  if(uTimeNow > uLastLedUpdate + 2){//Update LEDs after every 2 ms
    vRunLeds(uTimeNow/10);//Pass the LED updater the number of 10 ms that have passed
    uLastLedUpdate = uTimeNow;//Update the last LastLedUpdate variable to the value just used
  }

  // If the mp3 player has something to say
  if (mp3Player.available()) {
    Serial.println("Player has something to say: ");
    printDetail(mp3Player.readType(), mp3Player.read()); //Print the detail message from DFPlayer to handle different errors and states.
    Serial.println("");
  }
}

testing stuff is fun