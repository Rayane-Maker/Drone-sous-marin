#include <Arduino.h>
#include <Servo.h>

// put function and variables declarations here:

////////////////////// Propellers //////////////////////

//Objects : Respectively Left, Right, Back, Front
Servo left_propeller;
Servo right_propeller;
Servo back_propeller;
Servo front_propeller;

//Pins
#define LEFT_PROPELLER 8
#define RIGHT_PROPELLER 9
#define BACK_PROPELLER 10
#define FRONT_PROPELLER 11

//PWM Values (Microseconds)
#define MIN_PULSE_WIDTH 1000 // For max speed reverse
#define MAX_PULSE_WIDTH 2000 // Max max speed forward
#define MEDIUM_PULSE_WIDTH 1500 // Neutral : 0 speed

//ESC initiations delay (See ESC doc)
#define INIT_DELAY1 3000
#define INIT_DELAY2 4500 
#define INIT_DELAY3 1000 // Delay to allow the ESC to recognize the stopped signal.

//Init Func
void initPropellers(void);
 

///////////////////////// IMU  //////////////////////////



void setup() {

  // put your setup code here, to run once:

  /////// Propellers Initiation ////////
  initPropellers();

}

void loop() {
  // put your main code here, to run repeatedly:
  left_propeller.writeMicroseconds(2000);
  right_propeller.writeMicroseconds(2000);
  back_propeller.writeMicroseconds(2000);
  front_propeller.writeMicroseconds(2000);
}

// put function definitions here:

void initPropellers(){
  left_propeller.attach(LEFT_PROPELLER, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  right_propeller.attach(RIGHT_PROPELLER, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  back_propeller.attach(BACK_PROPELLER, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  front_propeller.attach(FRONT_PROPELLER, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  left_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  right_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  back_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  front_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);

  delay(INIT_DELAY1); 

  left_propeller.writeMicroseconds(MAX_PULSE_WIDTH);
  right_propeller.writeMicroseconds(MAX_PULSE_WIDTH);
  back_propeller.writeMicroseconds(MAX_PULSE_WIDTH);
  front_propeller.writeMicroseconds(MAX_PULSE_WIDTH);

  delay(INIT_DELAY2); 
  
  left_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  right_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  back_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);
  front_propeller.writeMicroseconds(MEDIUM_PULSE_WIDTH);

  delay(INIT_DELAY3); // delay to allow the ESC to recognize the stopped signal.
}