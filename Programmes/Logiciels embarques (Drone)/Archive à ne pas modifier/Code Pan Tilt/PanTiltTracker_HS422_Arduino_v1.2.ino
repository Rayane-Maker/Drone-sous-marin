#include <Servo.h>

//********************* Mode management *******************************
unsigned long timeSinceStart;
unsigned long ftimeSinceStart = 0;
bool full_started = false;
bool firstloop = false;

#define MODE_SWEEP 0
#define MODE_TRACKER_SPEED_CONTROLLER_V1 1
#define MODE_TRACKER_SPEED_CONTROLLER_V2 2
#define MODE_TRACKER_SPEED_CONTROLLER_AX12_BUILTIN 3
#define MODE_MONITOR 4

int mode_id = MODE_TRACKER_SPEED_CONTROLLER_V2;
bool mode_init = true;


//************************ Main actuators *****************************

//Full duplex (RX/TX) to half duplex (Data pin) Dynamixel AX12 Servos Adapter (SN74LS241N)
#define PAN_SERVO_PIN 2
#define TILT_SERVO_PIN 3
#define PAN_SERVO_ID (1u)
#define TILT_SERVO_ID (2u)
Servo panServo;  // create servo object to control a servo
Servo tiltServo;  // create servo object to control a servo

/* Constants angles : from 0 degree to 180 degrees */
#define INIT_PAN  (90u)      //  0 deg  =>  0  + 90 => 90deg(hs422) 
#define INIT_TILT (110u)     //  20deg  =>  20 + 90 => 110deg(hs422) 
#define PAN_MIN_POS (10u)    // -90deg  => -90 + 90 => 0deg(hs422)  
#define PAN_MAX_POS (175u)   //  90deg  =>  90 + 90 => 180deg(hs422) 
#define TILT_MIN_POS (87u)   // -20deg  => -20 + 90 => 70deg(hs422) 
#define TILT_MAX_POS (1808u) //  90deg  =>  90 + 90 => 180deg(hs422) 

#define INIT_SPEED (80u)
#define PAN_MIN_SPEED (0u)
#define PAN_MAX_SPEED (512u)
#define TILT_MIN_SPEED (0u)
#define TILT_MAX_SPEED (512u)

bool enable_servo[] = {true, true};
long speedCommands[2] = {0, 0};
long old_speedCommands[2] = {0, 0};
unsigned int panSpeed = 0;
unsigned int tiltSpeed = 0;
unsigned int pan = INIT_PAN;
unsigned int tilt = INIT_TILT;
unsigned int estimatedPan = INIT_PAN;
unsigned int estimatedTilt = INIT_PAN;
int updateRate = 0; // (ms) Update rate of speed commands integration
unsigned long previousEleapsedTime;

/*Enhanced SpeedController */
class ServoSpeedController {
  private:
    double wservo;
    int responseTimeServo; // (ms) - Property of servo (found experimentaly)
    int responseTimeServox2; //(ms) - Property of servo (found experimentaly)
    double wservo0; // Min speed at rate = 0 - Property of servo (found experimentaly)
    double wservo0bis;
    double time0;
    double time0bis;

    int increment;
    int rate;
    unsigned long SC_previousEleapsedTime;
    unsigned int mPosition;

  public:
    ServoSpeedController(
      int _responseTimeServo = 6, //(ms) - Property of servo (found experimentaly)
      double _wservo0 = (double)1, // Min speed at rate = 0 - Property of servo (found experimentaly)
      double _wservo0bis = (double)0.5,
      double _time0 = 1
    );

    int updatePosition(double _wservo, int _dir, int _mPosition);

};

ServoSpeedController :: ServoSpeedController(
  int _responseTimeServo,
  double _wservo0,
  double _wservo0bis,
  double _time0
) {
  responseTimeServo = _responseTimeServo;
  responseTimeServox2 = responseTimeServo * 2;
  wservo0 = _wservo0;
  wservo0bis = _wservo0bis;
  time0 = _time0;
  time0bis = time0 * 2;
}

int ServoSpeedController :: updatePosition(double _wservo, int _dir, int _mPosition) {

  mPosition = _mPosition;
  wservo = _wservo; //TODO check if correct value
  double mTime = 1 / (double)wservo;

  double ratio = (double)wservo / wservo0;
  int inpart = (int)ratio;
  double decpart = ratio - inpart;

  int increment;
  int rate;

  if (decpart < 0.001) {
    increment = (int)ratio;
    if (_dir < 0) {
      increment *= -1;
    }
    rate = 0;
  }
  else {
    increment = (int)(time0bis * wservo) + 1;
    if (_dir < 0) {
      increment *= -1;
    }
    rate = responseTimeServox2 * mTime * abs(increment) * wservo0bis;
  }

  if ((millis() - SC_previousEleapsedTime) > rate ) {
    SC_previousEleapsedTime = millis();
    mPosition += increment;
  }

  return mPosition;
}

ServoSpeedController panSpeedController = ServoSpeedController(6, 1, 0.5, 1);
ServoSpeedController tiltSpeedController = ServoSpeedController(6, 1, 0.5, 1);


//For sweep test mode
int dirr = 1;

// !!!! WARNING TO DO : Verify pins (11  !!!!!!!!!!!!!!!!!!!!!!)
//H-Bridge motor driver pins:
#define ENA1 5
#define ENA2 11
#define IN1  6
#define IN2  7
#define IN3  8
#define IN4  9


//Bluetooth communication
#define DEVICE_NAME "Pan-Tilt Tracker"
long serialbaud = 115200; //4800;//9600; //57600; //115200;
String inString = "";
String frame = "";       //String to hold input instruction packet from bluetooth
String char_command = "";
bool startReading;
bool nextDataIsComing;
long recv_data[3] = {0, 0, 0};
int recv_data_id;




// *********************************************************************SETUP INITIATION***************************************************************************************


void setup() {

  //Setup mode
  mode_id = MODE_TRACKER_SPEED_CONTROLLER_AX12_BUILTIN;

  //************Setup servos************
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  panServo.write(INIT_PAN);
  tiltServo.write(INIT_TILT);
  delay(2000);


  //************Setup motor driver************
  //  pinMode(ENA1, OUTPUT);
  //  pinMode(ENA2, OUTPUT);
  //  pinMode(IN1, OUTPUT);
  //  pinMode(IN2, OUTPUT);
  //  pinMode(IN3, OUTPUT);
  //  pinMode(IN4, OUTPUT);



  //************Setup Bluetooth Communication************
  // Open serial communications and wait for port to open:
  Serial.begin(serialbaud);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  //Time to substract to millis() in order to get real time from setup exit
  ftimeSinceStart = millis();
}



void loop() {

  timeSinceStart = millis();
  timeSinceStart -= ftimeSinceStart; //Get real time since exit from setup



  //------------------------------------------------------------------------Blutooth frame reception:---------------------------------------------------------------------------------

// Structure frame : < pan, tilt >

  while (Serial.available() > 0) {
    int command = Serial.read();
    // convert the incoming byte to a char and add it to the string:
    inString += (char)command;

    if ((char)command == '>')
    {
      frame = inString;
      inString = "";
    }
  }

  //  if (inString.length() > 1) {
  //    frame = inString;
  //    // clear the string for new input:
  //    inString = "";
  //  }


  //Bluetooth frame decoding:
  char chr = ' ';
  char_command = "";
  recv_data_id = 0;
  for (int i = 0; i < frame.length(); i++ ) {
    chr = frame[i];
    if (chr == '<') {
      startReading = true;
    }
    if (chr == '>') {
      startReading = false;
      recv_data[recv_data_id] = char_command.toInt();
      recv_data_id = 0;
      char_command = "";
    }
    if (chr == ',') {    //Sub-System Can be replaced with : Serial.readStringUntil(',');
      recv_data[recv_data_id] = char_command.toInt();
      recv_data_id ++;
      char_command = "";
    }

    if (startReading && chr != '<' && chr != ',' && chr != '>') {
      char_command += chr;

    }
  }

  //Serial.println(frame);
  //Serial.println("data[" + (String)0 + "] is " + (String)(recv_data[0]));
  //Serial.println("data[" + (String)1 + "] is " + (String)(recv_data[1]));



  //***************************************---------Tracking Mode V2 : Servo positionning using a refined custom speed control (Better speed resolution) ----------******************************************************************************

  if (mode_id == MODE_TRACKER_SPEED_CONTROLLER_V2) {


    speedCommands[0] = recv_data[0];
    speedCommands[1] = recv_data[1];

    //Outlier rejection filter : just skip abberation values
    long delta_speedCommands[2];
    delta_speedCommands[0] = speedCommands[0] - old_speedCommands[0];
    delta_speedCommands[1] = speedCommands[1] - old_speedCommands[1];
    if (abs(delta_speedCommands[0]) > 10000) {
      speedCommands[0] = old_speedCommands[0];
    }
    if (abs(delta_speedCommands[1]) > 10000) {
      speedCommands[1] = old_speedCommands[1];
    }

    int panDir = speedCommands[0] >= 0 ? 1 : -1;
    int tiltDir = speedCommands[1] >= 0 ? 1 : -1;
    pan = panSpeedController.updatePosition(abs(speedCommands[0]) / (double)100, panDir, pan);
    tilt = tiltSpeedController.updatePosition(abs(speedCommands[1]) / (double)100, tiltDir, tilt);

    //SATURATOR : positions limits !
    if (pan < PAN_MIN_POS) {
      pan = PAN_MIN_POS;
    }
    if (pan > PAN_MAX_POS) {
      pan = PAN_MAX_POS;
    }
    if (tilt < TILT_MIN_POS) {
      tilt = TILT_MIN_POS;
    }
    if (tilt > TILT_MAX_POS) {
      tilt = TILT_MAX_POS;
    }

    //Final Commands
    if (enable_servo[0]) {
      panServo.write(pan);
    }
    if (enable_servo[1]) {
      tiltServo.write(tilt);
    }

  }

  //----------------------------Tracking mode V1(Deprecated) : Servo positionning using custom speed control (Using Euler integration to retrieve servo positions) ----------------------------------------------------

  if (mode_id == MODE_TRACKER_SPEED_CONTROLLER_V1) {
    timeSinceStart = millis();
    updateRate = recv_data[2];
    if ((timeSinceStart - previousEleapsedTime) > 60 ) {
      previousEleapsedTime = timeSinceStart;

      speedCommands[0] = recv_data[0];
      speedCommands[1] = recv_data[1];

      //Outlier rejection filter : just skip abberation values
      long delta_speedCommands[2];
      delta_speedCommands[0] = speedCommands[0] - old_speedCommands[0];
      delta_speedCommands[1] = speedCommands[1] - old_speedCommands[1];
      if (abs(delta_speedCommands[0]) > 10000) {
        speedCommands[0] = old_speedCommands[0];
      }
      if (abs(delta_speedCommands[1]) > 10000) {
        speedCommands[1] = old_speedCommands[1];
      }

      //Integration
      pan += speedCommands[0]; //* updateRate;
      tilt += speedCommands[1]; //* updateRate;

    }
    //SATURATOR : positions limits !
    if (pan < PAN_MIN_POS) {
      pan = PAN_MIN_POS;
    }
    if (pan > PAN_MAX_POS) {
      pan = PAN_MAX_POS;
    }
    if (tilt < TILT_MIN_POS) {
      tilt = TILT_MIN_POS;
    }
    if (tilt > TILT_MAX_POS) {
      tilt = TILT_MAX_POS;
    }

    //Final Commands
    if (enable_servo[0]) {
      panServo.write(pan);
    }
    if (enable_servo[1]) {
      tiltServo.write(tilt);
    }

  }


  //******************************************************************---------Sweep Mode ----------******************************************************************************

  if (mode_id == MODE_SWEEP) {


    //    float wservo = 43; //in ticks per sec.
    //    float tr5 = 0.023; //0.044 //response time servo in sec.
    //    float deltaTicks = wservo*tr5;
    //    int increment = (int)deltaTicks;
    //    double decpart = deltaTicks - increment;
    //    int rate = decpart/wservo;


    double wservo = 0.5; //in Hz [0.05-0.5]
    double mTime = 1 / (double)wservo;

    int responseTimeServo = 6; // (ms)
    int responseTimeServox2 = responseTimeServo * 2; //(ms) - Property of servo (found experimentaly)
    double wservo0 = (double)1; // Min speed at rate = 0 - Property of servo (found experimentaly)
    double wservo0bis = (double)0.5;
    double time0 = 1;
    double time0bis = time0 * 2;

    double ratio = (double)wservo / wservo0;
    int inpart = (int)ratio;
    double decpart = ratio - inpart;

    int increment;
    int rate;

    if (decpart < 0.01) {
      increment = (int)ratio;
      rate = 0;
    }
    else {
      increment = (int)(time0bis * wservo) + 1;
      rate = responseTimeServox2 * mTime * increment * wservo0bis;
    }

    //Serial.println(increment);
    //Serial.println(rate);



    //Tests
    //increment = 1;
    //rate = 5;

    if ((timeSinceStart - previousEleapsedTime) > rate ) {
      previousEleapsedTime = timeSinceStart;

      pan += dirr * increment;

    }
    //SATURATOR : positions limits !
    if (pan < PAN_MIN_POS) {
      pan = PAN_MIN_POS;
      dirr *= -1;
    }
    if (pan > PAN_MAX_POS) {
      pan = PAN_MAX_POS;
      dirr *= -1;
    }
    if (tilt < TILT_MIN_POS) {
      tilt = TILT_MIN_POS;
    }
    if (tilt > TILT_MAX_POS) {
      tilt = TILT_MAX_POS;
    }

    //Final Commands
    if (enable_servo[0]) {
      panServo.write(pan);
    }
    if (enable_servo[1]) {
      tiltServo.write(tilt);
    }

  }



  //---------------------------------------------------------------------------Motors Test----------------------------------------------------------------


  //    analogWrite (6, map(POT_VALUE, 0, 1023, 0, 255)) ;
  //    analogWrite (5, 0) ;
  //    digitalWrite (IN1, HIGH) ;
  //    digitalWrite (IN2, LOW) ;
  //    digitalWrite (IN3, HIGH) ;
  //    digitalWrite (IN4, LOW) ;



}
//****************************************************************************--------------END--------------**************************************************************************************************************************************
