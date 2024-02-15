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

#ifndef ESController_h
#define ESController_h

#include <inttypes.h>



//PWM Values (Microseconds)
#define MIN_PULSE_WIDTH 1000 // For max speed reverse
#define MAX_PULSE_WIDTH 2000 // Max max speed forward
#define MEDIUM_PULSE_WIDTH 1500 // Neutral : 0 speed

//ESC initiations delay (See ESC doc)
#define INIT_DELAY1 3000
#define INIT_DELAY2 4500 
#define INIT_DELAY3 1000 // Delay to allow the ESC to recognize the stopped signal.
#if !defined(ARDUINO_ARCH_STM32F4)


class ESCController {
  public:
      ESCController();
      ESCController(int pin, long min_pulse_width = MIN_PULSE_WIDTH, long max_pulse_width = MAX_PULSE_WIDTH, long initDelay1 = INIT_DELAY2, long initDelay2 = INIT_DELAY2, long initDelay3 = INIT_DELAY3, String name);

      void init(int delay1, int delay2, int delay3);
      void init();
      void throttleFull();
      void throttle(int percentage);
      void turnOff();
      String name;
    

  private:
      Servo esc; // Core object : The esc to be controlled
      int pin; 

      //Init params
      long initDelay1 = INIT_DELAY1, initDelay2 = INIT_DELAY2, initDelay3 = INIT_DELAY3;
      bool isInitiated;

      // PWM params
      long min_pulse_width = MIN_PULSE_WIDTH,  idle_pulse_width = MEDIUM_PULSE_WIDTH, max_pulse_width = MAX_PULSE_WIDTH;

};

#endif
#endif
