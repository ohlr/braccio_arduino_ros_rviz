/*
  Braccio.h - board library Version 2.0
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

#ifndef BRACCIOLIB_H_
#define BRACCIOLIB_H_

#include <Arduino.h>
#include <Servo.h>

// You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
#define SOFT_START_DISABLED		-999

//The default value for the soft start
#define SOFT_START_DEFAULT		0

//The software PWM is connected to PIN 12. You cannot use the pin 12 if you are using
//a Braccio shield V4 or newer
#define SOFT_START_CONTROL_PIN	12

//Low and High Limit Timeout for the Software PWM
#define LOW_LIMIT_TIMEOUT 2000
#define HIGH_LIMIT_TIMEOUT 6000

class _Braccio {

public:
  _Braccio();

  /**
  * Braccio initializations and set intial position
  * Modifing this function you can set up the initial position of all the
  * servo motors of Braccio
  *@param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
  * You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
  */
  unsigned int begin(int soft_start_level=SOFT_START_DEFAULT);

  /**
   * This function allow the user to control all the servo motors in the Braccio
   */
  void ServoMovement(int stepDelay, int vBase, int vShoulder, int vElbow,int vWrist_ver, int vWrist_rot, int vGripper);

  void ServoStatus(int vBase, int vShoulder, int vElbow,int vWrist_ver, int vWrist_rot, int vGripper, int step_taster);

  void Grab(int stepDelay);

  int getTaster();

  int getGripper();


private:
  /*
  * This function, used only with the Braccio Shield V4 and greater,
  * turn ON the Braccio softly and save Braccio from brokes.
  * The SOFT_START_CONTROL_PIN is used as a software PWM
  * @param soft_start_level: the minimum value is -70, , default value is 0 (SOFT_START_DEFAULT)
  */
  void _softStart(int soft_start_level);

  /*
  * Software implementation of the PWM for the SOFT_START_CONTROL_PIN,HIGH
  * @param high_time: the time in the logic level high
  * @param low_time: the time in the logic level low
  */
  void _softwarePWM(int high_time, int low_time);


};

extern _Braccio Braccio;

#endif // BRACCIO_H_
