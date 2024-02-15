/*
 ESCController.cpp - Servo library driven ESC Controller for Arduino - Version 1
 This class is intended to simplify use of brushless ESC controllers in any Arduino project.
 Copyright (c) 2023 Rayane GETA.  All right reserved.

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

#if defined(ARDUINO_ARCH_AVR)

#include <Arduino.h>
#include "Servo.h"
#include "ESCController.h"

float map2(float in, float in_min, float in_max, float out_min, float out_max);

// dummy constructor
ESCController::ESCController() {}

ESCController::ESCController(int pin, long min_pulse_width, long max_pulse_width, long initDelay1, long initDelay2, long initDelay3, String name)
{
    this -> pin = pin;
    this -> name = "";
    this -> min_pulse_width = min_pulse_width;
    this -> idle_pulse_width = (min_pulse_width + max_pulse_width)/2;
    this -> max_pulse_width = max_pulse_width;
    esc.attach(pin, min_pulse_width, max_pulse_width);
}


void ESCController::init(){
  esc.writeMicroseconds(idle_pulse_width);
  delay(INIT_DELAY1); 

  esc.writeMicroseconds(max_pulse_width);
  delay(INIT_DELAY2);

  esc.writeMicroseconds(idle_pulse_width);
  delay(INIT_DELAY3); // delay to allow the ESC to recognize the stopped signal.
  isInitiated = true;
}


void ESCController::throttle(int percentage) {
  if (!isInitiated){
    return;
  }
  esc.writeMicroseconds(map2(percentage, 0, 100, min_pulse_width, max_pulse_width));
}

void ESCController::throttleFull() {
  throttle(100);
}


void ESCController::turnOff() {
  throttle(0);
}



float map2(float in, float in_min, float in_max, float out_min, float out_max){
  return (in - in_min) * (out_max - out_min)/(in_max - in_min) + out_min;
}
    


#endif // ARDUINO_ARCH_AVR
