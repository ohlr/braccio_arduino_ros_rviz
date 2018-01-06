/*

 braccioOfUnoWifi.ino
 
 Based on Arduino Uno WiFi Rest Server example

 This example for the Arduino Uno WiFi shows how to 
 control a TinkerKit Braccio through REST calls. 
 You can create your how mobile app or your 
 browser app to control the Braccio in wireless mode
 
 Note that with the Braccio shield version less than V4
 you need to disconnect the pin A4 from the shield to the board

 Possible commands created in this shetch:

 * "/arduino/custom/base/value:80"	-> Moves the base of the Braccio at 80 degrees
 * "/arduino/custom/shoulder/value:150"	-> Moves the shoulder of the Braccio at 150 degrees
 * "/arduino/custom/elbow/value:45"	-> Moves the elbow of the Braccio at 45 degrees
 * "/arduino/custom/wristv/value:10"	-> Moves the wristv of the Braccio at 10 degrees
 * "/arduino/custom/wristr/value:120"	-> Moves the wristr of the Braccio at 120 degrees
 * "/arduino/custom/gripper/value:73"	-> Close the gripper
 * "/arduino/custom/ledon"		-> Turn ON the LED 13
 * "/arduino/custom/ledoff"		-> Turn OFF the LED 13
 * "/arduino/custom/servo:3/value:73"	-> Moves the servo to the pin 3 at 73 degrees
 * "/arduino/custom/sayciao"		-> Run the function sayciao(). The Braccio say "Ciao" with the gripper
 * "/arduino/custom/takesponge"		-> Run the function takesponge(). The Braccio take the big sponge you can find in the its box
 * "/arduino/custom/showsponge"		-> Run the function showsponge(). The Braccio show the sponge to the user
 * "/arduino/custom/throwsponge"	-> Run the function throwsponge(). The Braccio throw away the sponge

 This example code is part of the public domain

 http://labs.arduino.org/RestServer+and+RestClient
 http://www.arduino.org/products/tinkerkit/arduino-tinkerkit-braccio

*/

#include <Wire.h>
#include <ArduinoWiFi.h>
#include <Servo.h>
#include <Braccio.h>

//Initial Value for each Motor
int m1 = 0;
int m2 = 45;
int m3 = 180;
int m4 = 180;
int m5 = 90;
int m6 = 0;

boolean moveBraccio = false;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


void setup() {
  //Intitialization of Braccio
  Braccio.begin();
  //Intitialization of the Uno WiFi
  Wifi.begin();
  Wifi.println("REST Server is up");
}

void loop() {
  //Wait until the board receives HTTP commands
  while (Wifi.available()) {
    process(Wifi);
  }
  delay(50);
}

/**
Parse Command from REST
It parse a command like: /arduino/custom/base/value:45
@param command: The message to parse
@param type: the key for parsing
@return the value for the key 
*/
int parseCommand(String command, String type) {
  int typeIndex = command.indexOf(type);
  int dotsIndex = command.indexOf(':', typeIndex + type.length());

  int idxtmp = dotsIndex + 4;
  if ((dotsIndex + 4) > command.length()) idxtmp = command.length();
  String tmp = command.substring(dotsIndex + 1, idxtmp);

  return tmp.toInt();
}

/**
It process data from the HTTP protocol
*/
void process(WifiData client) {
  // read the command
  String command = client.readString();
  command.toUpperCase();
  
  if(command.indexOf("CUSTOM")==-1){
    client.println("Invalid command: " + command + "");
    return;
  }
  
  //The message from sender
  String message = command.substring(16);
  //client.println(message); //Debug 
  
  /*
  For each message perform the proper command
  */
  if (message == "LEDON") {
    //Turn ON Led 13
    digitalWrite(13, HIGH);
    //Return message to the sender (Eg: the browser)
    client.println("alert('Led D13 ON');");
  }
  else if (message == "LEDOFF") {
    digitalWrite(13, LOW);
    client.println("alert('Led D13 OFF');");
  }
  //This command allow you to move a desired servo motor giving the
  //PWM pin where is connected
  //eg: http://192.168.240.1/arduino/custom/servo:3/value:45 or http://192.168.240.1/arduino/custom/base/value:45
  else if (message.startsWith("SERVO")) {
    //Parse the message to retrive what is the servo to move
    int servo = parseCommand(message, "SERVO");
    //Parse the message to retrive what is the value for the servo
    int value = parseCommand(message, "VALUE");

    client.println("Message:" + String(message) + "SERVO: " + String(servo) + " " + String(value));
    
    moveBraccio=true;
  }
  //http://192.168.240.1/arduino/custom/base:45 or http://192.168.240.1/arduino/custom/base/value:45
  //Command for the base of the braccio (M1)
  else if (message.startsWith("BASE")) {
    m1 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("BASE: " + String(m1));
  }
  //Command for the shoulder of the braccio (M2)
  else if (message.startsWith("SHOULDER")) {
    m2 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("SHOULDER: " + String(m2));
  }
  //Command for the elbow of the braccio (M3)
  else if (message.startsWith("ELBOW")) {
    m3 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("ELBOW: " + String(m3));
  }
  //Command for the wrist of the braccio to move it up and down (M4)
  else if (message.startsWith("WRISTV")) {
    m4 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("WRISTV: " + String(m4));
  }
  //Command for the wrist of the braccio to rotate it  (M5)
  else if (message.startsWith("WRISTR")) {
    m5 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("WRISTR: " + String(m5));
  }
  //Command for the GRIPPER of the braccio to open and close it (M6)
  else if (message.startsWith("GRIPPER")) {
    m6 = parseCommand(message, "VALUE");
    moveBraccio = true;
    client.println("GRIPPER: " + String(m6));
  }
  //Command to say "Ciao"
  else if (message.startsWith("SAYCIAO")) {
    sayCiao();
    client.println("SAYCIAO: " + String(m6));
  }
  //Command for take the sponge
  else if (message.startsWith("TAKESPONGE")) {
    takesponge();
    client.println("TAKESPONGE: " + String(m6));
  }
  //Command for show the sponge
  else if (message.startsWith("SHOWSPONGE")) {
    showsponge();
    client.println("SHOWSPONGE: " + String(m6));
  }
  //Command for throw away the sponge
  else if (message.startsWith("THROWSPONGE")) {
    throwsponge();
    client.println("THROWSPONGE: " + String(m6));
  }
  else
    client.println("command error: " + message);

  //if flag moveBraccio is true fire the movement
  if (moveBraccio) {
    //client.println("moveBraccio");
    Braccio.ServoMovement(20, m1, m2, m3, m4, m5, m6);
    moveBraccio = false;
  }
  
  client.flush();
}

/**
The braccio Say 'Ciao' with the GRIPPER
*/
void sayCiao() {
  Braccio.ServoMovement(20,           90,  0, 180, 160,  0,  15);

  for (int i = 0; i < 5; i++) {
    Braccio.ServoMovement(10,           90,  0, 180, 160,  0,  15);
    delay(500);

    Braccio.ServoMovement(10,     90,  0,   180,   160,  0,   73);
    delay(500);
  }
}

/**
Braccio take the Sponge
*/
void takesponge() {
  //starting position
  //(step delay  M1 , M2 , M3 , M4 , M5 , M6);
  Braccio.ServoMovement(20,           0,  45, 180, 180,  90,  0);

  //I move arm towards the sponge
  Braccio.ServoMovement(20,           0,  90, 180, 180,  90,   0);

  //the GRIPPER takes the sponge
  Braccio.ServoMovement(20,           0,  90, 180, 180,  90,  60 );

  //up the sponge
  Braccio.ServoMovement(20,         0,   45, 180,  45,  0, 60);
}


/**
Braccio show the sponge to the user
*/
void showsponge() {
  for (int i = 0; i < 2; i++) {

    //(step delay  M1 , M2 , M3 , M4 , M5 , M6 );
    Braccio.ServoMovement(10,         0,   45, 180,  45,  180, 60);

    Braccio.ServoMovement(10,         0,   45, 180,  45,  0, 60);
  }
}

/**
Braccio throw away the sponge
*/
void throwsponge() {
  //(step delay  M1 , M2 , M3 , M4 , M5 , M6 );
  Braccio.ServoMovement(20,         0,   45, 90,  45,  90, 60);

  Braccio.ServoMovement(5,         0,   45, 135,  90,  90, 60);

  Braccio.ServoMovement(5,         0,   90, 150,  90,  90, 0);
}
