#include <Arduino.h>
#include <Servo.h>
#include <ESCController.h>

// put function and variables declarations here:
ESCController escControllers[];
int motorcount = 0;

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

void initPropellers(void);
float map2(float in, float in_min, float in_max, float out_min, float out_max);
int addMotor(ESCController escControllers[], int pin);


int addMotor(ESCController escControllers[], int pin) {
    motorcount ++;
    ESCController newESCControllers[motorcount];
    newESCControllers[0] = ESCController(pin);
    if (escControllers == nullptr) {
        return 0;
    }
    //Array copy
    for (int i = 1; i < motorcount; i++){
        newESCControllers[i] = escControllers[i-1];
    }
    escControllers = newESCControllers;
    return 1;
}

int clean(ESCController escControllers[]) {
        delete[] escControllers;
}


float map2(float in, float in_min, float in_max, float out_min, float out_max){
  return (in - in_min) * (out_max - out_min)/(in_max - in_min) + out_min;
}

const int ledPin=13;
String nom = "Arduino";
String msg;

void setup() {
 	Serial.begin(9600);
}

void loop() {
 	readSerialPort();

 	if (msg == "data") {
 			sendData();
 	}else if(msg=="led0"){
 			digitalWrite(ledPin,LOW);
 			Serial.print(" Arduino set led to LOW");
 	}else if(msg=="led1"){
 			digitalWrite(ledPin,HIGH);
 			Serial.print(" Arduino set led to HIGH");
 	}
 	delay(500);
}

void readSerialPort() {
 	msg = "";
 	if (Serial.available()) {
 			delay(10);
 			while (Serial.available() > 0) {
 					msg += (char)Serial.read();
 			}
 			Serial.flush();
 	}
}

void sendData() {
 	//write data ledState x sensor1 x sensor2
 	Serial.print(digitalRead(ledPin));
 	Serial.print("x");
 	Serial.print(analogRead(A0));
 	Serial.print("x");
 	Serial.print(analogRead(A1));
}


