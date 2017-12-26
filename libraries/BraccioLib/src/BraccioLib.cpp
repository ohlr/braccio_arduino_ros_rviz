/*
 Braccio.cpp - board library Version 2.0
 Written by Andrea Martino and Angelo Ferrante

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "BraccioLib.h"

extern Servo base;
extern Servo shoulder;
extern Servo elbow;
extern Servo wrist_rot;
extern Servo wrist_ver;
extern Servo gripper;

extern int step_base = 0;
extern int step_shoulder = 45;
extern int step_elbow = 180;
extern int step_wrist_rot = 180;
extern int step_wrist_ver = 90;
extern int step_gripper = 10;

int step_taster=0;

_Braccio Braccio;

//Initialize Braccio object
_Braccio::_Braccio() {
}

/**
 * Braccio initialization and set intial position
 * Modifing this function you can set up the initial position of all the
 * servo motors of Braccio
 * @param soft_start_level: default value is 0 (SOFT_START_DEFAULT)
 * You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
 * SOFT_START_DISABLED disable the Braccio movements
 */
unsigned int _Braccio::begin(int soft_start_level) {
	//Calling Braccio.begin(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
	if(soft_start_level!=SOFT_START_DISABLED){
		pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
		digitalWrite(SOFT_START_CONTROL_PIN,LOW);
	}

	// initialization pin Servo motors
	base.attach(11); //M1
	shoulder.attach(10); //M2
	elbow.attach(9); //M3
	wrist_ver.attach(6); //M4
	wrist_rot.attach(5); //M5
	gripper.attach(3); //M6

	//tastsensor
	pinMode(8,INPUT);

	//For each step motor this set up the initial degree
	base.write(0);
	shoulder.write(40);
	elbow.write(180);
	wrist_ver.write(0);
	wrist_rot.write(170);
	gripper.write(73);
	//Previous step motor position
	step_base = 0;
	step_shoulder = 40;
	step_elbow = 180;
	step_wrist_ver = 0;
	step_wrist_rot = 170;
	step_gripper = 73;

	if(soft_start_level!=SOFT_START_DISABLED)
    		_softStart(soft_start_level);
	return 1;
}

/*
Software implementation of the PWM for the SOFT_START_CONTROL_PIN,HIGH
@param high_time: the time in the logic level high
@param low_time: the time in the logic level low
*/
void _Braccio::_softwarePWM(int high_time, int low_time){
	digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
	delayMicroseconds(high_time);
	digitalWrite(SOFT_START_CONTROL_PIN,LOW);
	delayMicroseconds(low_time);
}

/*
* This function, used only with the Braccio Shield V4 and greater,
* turn ON the Braccio softly and save it from brokes.
* The SOFT_START_CONTROL_PIN is used as a software PWM
* @param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
*/
void _Braccio::_softStart(int soft_start_level){
	long int tmp=millis();
	while(millis()-tmp < LOW_LIMIT_TIMEOUT)
		_softwarePWM(80+soft_start_level, 450 - soft_start_level);   //the sum should be 530usec

	while(millis()-tmp < HIGH_LIMIT_TIMEOUT)
		_softwarePWM(75 + soft_start_level, 430 - soft_start_level); //the sum should be 505usec

	digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
}

/**
 * This functions allow you to control all the servo motors
 *
 * @param stepDelay The delay between each servo movement
 * @param vBase next base servo motor degree
 * @param vShoulder next shoulder servo motor degree
 * @param vElbow next elbow servo motor degree
 * @param vWrist_ver next wrist rotation servo motor degree
 * @param vWrist_rot next wrist vertical servo motor degree
 * @param vGripper next gripper servo motor degree
 */
void _Braccio::ServoMovement(int stepDelay, int vBase, int vShoulder, int vElbow,int vWrist_ver, int vWrist_rot, int vGripper) {
    vBase=180-vBase;
	// Check values, to avoid dangerous positions for the Braccio
    	if (stepDelay > 30) stepDelay = 30;
	if (stepDelay < 10) stepDelay = 10;
	if (vBase < 0) vBase=0;
	if (vBase > 180) vBase=180;
	if (vShoulder < 15) vShoulder=15;
	if (vShoulder > 165) vShoulder=165;
	if (vElbow < 0) vElbow=0;
	if (vElbow > 180) vElbow=180;
	if (vWrist_ver < 0) vWrist_ver=0;
	if (vWrist_ver > 180) vWrist_ver=180;
	if (vWrist_rot > 180) vWrist_rot=180;
	if (vWrist_rot < 0) vWrist_rot=0;
    if (vGripper < 10) vGripper = 10;
	if (vGripper > 73) vGripper = 73;

	int exit = 1;

	//Until the all motors are in the desired position
	while (exit)
	{
		//For each servo motor if next degree is not the same of the previuos than do the movement
		if (vBase != step_base)
		{
			base.write(step_base);
			//One step ahead
			if (vBase > step_base) {
				step_base++;
			}
			//One step beyond
			if (vBase < step_base) {
				step_base--;
			}
		}

		if (vShoulder != step_shoulder)
		{
			shoulder.write(step_shoulder);
			//One step ahead
			if (vShoulder > step_shoulder) {
				step_shoulder++;
			}
			//One step beyond
			if (vShoulder < step_shoulder) {
				step_shoulder--;
			}

		}

		if (vElbow != step_elbow)
		{
			elbow.write(step_elbow);
			//One step ahead
			if (vElbow > step_elbow) {
				step_elbow++;
			}
			//One step beyond
			if (vElbow < step_elbow) {
				step_elbow--;
			}

		}

		if (vWrist_ver != step_wrist_ver)
		{
			wrist_ver.write(step_wrist_ver);
			//One step ahead
			if (vWrist_ver > step_wrist_ver) {
				step_wrist_ver++;
			}
			//One step beyond
			if (vWrist_ver < step_wrist_ver) {
				step_wrist_ver--;
			}

		}

		if (vWrist_rot != step_wrist_rot)
		{
			wrist_rot.write(step_wrist_rot);
			//One step ahead
			if (vWrist_rot > step_wrist_rot) {
				step_wrist_rot++;
			}
			//One step beyond
			if (vWrist_rot < step_wrist_rot) {
				step_wrist_rot--;
			}
		}

		if (vGripper != step_gripper)
		{
			gripper.write(step_gripper);
			if (vGripper > step_gripper) {
				step_gripper++;
			}
			//One step beyond
			if (vGripper < step_gripper) {
				step_gripper--;
			}
		}

		//read tasterstatus
		//taster=1 if gripper is closed
		step_taster=digitalRead(8);

		ServoStatus(vBase, vShoulder, vElbow, vWrist_ver, vWrist_rot, vGripper, step_taster);

		//delay between each movement
		delay(stepDelay);

		//It checks if all the servo motors are in the desired position
		if ((vBase == step_base) && (vShoulder == step_shoulder)
				&& (vElbow == step_elbow) && (vWrist_ver == step_wrist_ver)
			    && (vWrist_rot == step_wrist_rot) && (vGripper == step_gripper)) {
			step_base = vBase;
			step_shoulder = vShoulder;
			step_elbow = vElbow;
			step_wrist_ver = vWrist_ver;
			step_wrist_rot = vWrist_rot;
			step_gripper = vGripper;
			exit = 0;
		} else {
			exit = 1;
		}
	}
}

void _Braccio::Grab(int stepDelay){

	int exit = 1;

	while (exit == 1){
		step_taster=digitalRead(8);

		if(step_taster == 1){
			exit = 0;
		}
		else if(step_gripper <= 73){
			step_gripper++;
			gripper.write(step_gripper);
			delay(stepDelay);
		}
		else{
			exit=0;
		}

	}
}

int _Braccio::getTaster(){
	return step_taster;
}

int _Braccio::getGripper(){
	return step_gripper;
}

void _Braccio::ServoStatus(int vBase, int vShoulder, int vElbow,int vWrist_ver, int vWrist_rot, int vGripper, int step_taster){

	Serial.print("Base: ");
    Serial.print(180-step_base);
	Serial.print(" (");
    Serial.print(180-vBase);
    Serial.print(") ");

	Serial.print(" Shoulder: ");
	Serial.print(step_shoulder);
	Serial.print(" (");
	Serial.print(vShoulder);
	Serial.print(") ");

	Serial.print(" Elbow: ");
    Serial.print(step_elbow);
    Serial.print(" (");
    Serial.print(vElbow);
    Serial.print(") ");

	Serial.print(" Wrist Vertical: ");
    Serial.print(step_wrist_ver);
    Serial.print(" (");
    Serial.print(vWrist_ver);
    Serial.print(") ");

	Serial.print(" Wrist Rotation: ");
    Serial.print(step_wrist_rot);
    Serial.print(" (");
    Serial.print(vWrist_rot);
    Serial.print(") ");

	Serial.print(" Gripper: ");
    Serial.print(step_gripper);
    Serial.print(" (");
    Serial.print(vGripper);
    Serial.print(") ");

    Serial.print("Closed: ");
	Serial.println(step_taster);
}

