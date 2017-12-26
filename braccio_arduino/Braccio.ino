#include <BraccioLib.h>

#include <Arduino.h>

#include <Servo.h>



Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

int Grip;
int Tast;
void setup() {
  // put your setup code here, to run once:
   Braccio.begin();

   Serial.begin(115200);

//   Braccio.ServoMovement(30,90,90,90,90,90,10);
//
//   delay(1000);

//
  Braccio.ServoMovement(30,90,90,90,90,90,10);
  delay(1000);
  Braccio.ServoMovement(30,90,90,90,180,90,10);
  delay(1000);
  Braccio.ServoMovement(30,90,90,180,180,90,10);
  delay(1000);
  Braccio.ServoMovement(30,90,90,180,180,90,60);
  delay(1000);
//  Braccio.Grab(30);
//  Grip=Braccio.getGripper();
//  Tast=Braccio.getTaster();
//  Serial.println(Grip);
//  Serial.print(Tast);
//
//   Braccio.ServoMovement(30,0,45,90,90,90,73);
//
//  Braccio.ServoMovement(30,0,45,180,90,90,10);
//
//  Braccio.ServoMovement(30,0,45,180,0,90,10);
//
//   Braccio.ServoMovement(30,0,45,180,45,0,10);

  // Braccio.ServoMovement(30,0,45,180,45,180,73);
//    Braccio.ServoMovement(30,0,90,90,0,90,10);
//    delay(1000);
//  Braccio.ServoMovement(30,0,90,90,0,90,73);
}

void loop() {
  // put your main code here, to run repeatedly
	 Braccio.ServoMovement(30,90,90,90,90,90,60);
 // Braccio.ServoMovement(30,0,90,90,0,90,73);
  delay(1000);
  Braccio.ServoMovement(30,180,90,90,90,90,60);
 // Braccio.ServoMovement(30,90,90,90,0,0,73);
  delay(1000);
  Braccio.ServoMovement(30,0,90,90,90,90,60);
 // Braccio.ServoMovement(30,180,90,90,0,180,73);

}

