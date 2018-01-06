/*

 This sketch uses the rest connector to receive command for the MCU from a rest client.
 Each command received will fire an action for the Braccio.
 Run the sketch with Arduino Braccio - Web Examples to move your Braccio via web page.
 Visit the Demo section of the Braccio's page.
 http://www.arduino.org/products/tinkerkit/17-arduino-tinkerkit/arduino-tinkerkit-braccio
 


REST command example:

 * "ledon"             -> turn on led 13
 * "ledoff"            -> turn off led 13

 example: http://arduino.local/arduino/ledon

 NOTE: be sure to activate and configure rest connector on Linino OS
       http://labs.arduino.org/Ciao

 created March 2016
 by andrea[at]arduino[dot]org and a.ferrante[at]arduino[dot]org

 */

#include <Ciao.h>
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
  //Initialization function for Ciao
  Ciao.begin();
  //Initialization function for Braccio
  //You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
  Braccio.begin();
}

/**
Parse Command from REST
It parse a command like: /arduino/base/value:45
Giving "base" it return the value
@param command: The message to parse
@param type: the key for parsing
*/
int parseCommand(String command, String type) {
  int typeIndex = command.indexOf(type);
  int dotsIndex = command.indexOf(':', typeIndex + type.length());

  int idxtmp = dotsIndex + 4;
  if ((dotsIndex + 4) > command.length()) idxtmp = command.length();
  String tmp = command.substring(dotsIndex + 1, idxtmp);

  return tmp.toInt();
}

void loop() {

  //Select REST connector
  CiaoData data = Ciao.read("restserver");
  //If data is not empry
  if (!data.isEmpty()) {
    //ID of the command
    String id = data.get(0);
    //Sender ID
    String sender = data.get(1);
    //The message from sender
    String message = data.get(2);

    message.toUpperCase();

    /*
    For each message do the proper command
    */
    if (message == "LEDON") {
      //Turn OFF Led 13
      digitalWrite(13, HIGH);
      //Return message to the sender (Eg: the browser)
      Ciao.writeResponse("restserver", id, "Led D13 ON");
    }
    else if (message == "LEDOFF") {
      digitalWrite(13, LOW);
      Ciao.writeResponse("restserver", id, "Led D13 OFF");
    }
    //This command allow you to move a desired servo motor giving the
    //PWM pin where is connected
    else if (message.startsWith("SERVO")) {
      //Parse the message to retrive what is the servo to move
      int servo = parseCommand(message, "SERVO");
      //Parse the message to retrive what is the value for the servo
      int value = parseCommand(message, "VALUE");

      Ciao.writeResponse("restserver", id, "Message:" + String(message) + "SERVO: " + String(servo) + " " + String(value));
    }
    //Command for the base of the braccio (M1)
    else if (message.startsWith("BASE")) {
      m1 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "BASE: " + String(m1));
    }
    //Command for the shoulder of the braccio (M2)
    else if (message.startsWith("SHOULDER")) {
      m2 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "SHOULDER: " + String(m2));
    }
    //Command for the elbow of the braccio (M3)
    else if (message.startsWith("ELBOW")) {
      m3 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "ELBOW: " + String(m3));
    }
    //Command for the wrist of the braccio to move it up and down (M4)
    else if (message.startsWith("WRISTV")) {
      m4 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "WRISTV: " + String(m4));
    }
    //Command for the wrist of the braccio to rotate it  (M5)
    else if (message.startsWith("WRISTR")) {
      m5 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "WRISTR: " + String(m5));
    }
    //Command for the gripper of the braccio to open and close it (M6)
    else if (message.startsWith("GRIPPER")) {
      m6 = parseCommand(message, "VALUE");
      moveBraccio = true;
      Ciao.writeResponse("restserver", id, "GRIPPER: " + String(m6));
    }
    //Command to say "Ciao"
    else if (message.startsWith("SAYCIAO")) {
      sayCiao();
      Ciao.writeResponse("restserver", id, "SAYCIAO: " + String(m6));
    }
    //Command for take the sponge
    else if (message.startsWith("TAKESPONGE")) {
      takesponge();
      Ciao.writeResponse("restserver", id, "TAKESPONGE: " + String(m6));
    }
   //Command for show the sponge
    else if (message.startsWith("SHOWSPONGE")) {
      showsponge();
      Ciao.writeResponse("restserver", id, "SHOWSPONGE: " + String(m6));
    }
    //Command for throw away the sponge
    else if (message.startsWith("THROWSPONGE")) {
      throwsponge();
      Ciao.writeResponse("restserver", id, "THROWSPONGE: " + String(m6));
    }

    else
      Ciao.writeResponse("restserver", id, "command error");

    //if flag moveBraccio is true fire the movement
    if (moveBraccio) {
      Braccio.ServoMovement(20, m1, m2, m3, m4, m5, m6);
      moveBraccio = false;
    }
  }
}

/**
The braccio Say 'Ciao' with the Tongue
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

  //the gripper takes the sponge
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
