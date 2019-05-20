// DEMO BOT CONTROL
// UKRMT KATS SMALL ROBOT CONTROL SOFTWARE
// UNIVERSITY OF KENTUCKY 2019
//   _  __    _  _____ ____    ____   ___ _____
//  | |/ /   / \|_   _/ ___|  | __ ) / _ \_   _|
//  | ' /   / _ \ | | \___ \  |  _ \| | | || |
//  | . \  / ___ \| |  ___) | | |_) | |_| || |
//  |_|\_\/_/   \_\_| |____/  |____/ \___/ |_|


// THE ROBOT USES 4 OF THIS POLOLU MOTOR DRIVER
// https://www.pololu.com/product/1451

#include <string.h>
#define DEBUG true

// MOTOR CONTROL PINS
#define MOTOR1_INA    23
#define MOTOR1_INB    21
#define MOTOR1_PWM    22
#define MOTOR2_INA    2
#define MOTOR2_INB    5
#define MOTOR2_PWM    3
#define MOTOR3_INA    17
#define MOTOR3_INB    15
#define MOTOR3_PWM    16
#define MOTOR4_INA    19
#define MOTOR4_INB    18
#define MOTOR4_PWM    20
String data_in;
char tmp;

enum {
  MOTOR1,
  MOTOR2,
  MOTOR3,
  MOTOR4,
  MOTORS_LEFT,  // LEFT SIDE MOTORS 3 AND 4
  MOTORS_RIGHT, // RIGHT SIDE MOTORS 1 AND 2
  MOTORS_ALL    // ALL MOTORS
} motors;

enum {
  DS_STOP,
  DS_FWD,
  DS_REV,
  DS_RT,
  DS_LT,
} drive_state;


void drive(  int m, int d, int spd ) {
  if ( spd > 255)  spd = 255;
  if ( spd < 0)    spd = 0;


  Serial1.print("driving motor "); Serial1.print(m); Serial1.print(" at speed ");
  Serial1.print(spd); Serial1.print(" in direction "); Serial1.println(d);

  switch (m)
  {
    case MOTOR1:
      if ( d == 1 )
      {
        Serial1.println("In and driving");
        digitalWrite(MOTOR1_INA, LOW );
        digitalWrite(MOTOR1_INB, HIGH );
        analogWrite(MOTOR1_PWM, spd);
      }

      // DRIVE REVERSE
      else if ( d == 0 )
      {
        digitalWrite(MOTOR1_INA, HIGH );
        digitalWrite(MOTOR1_INB, LOW );
        analogWrite(MOTOR1_PWM, spd);
      }
      break;

    case MOTOR2:
      if ( d == 1 )
      {
        digitalWrite(MOTOR2_INA, LOW );
        digitalWrite(MOTOR2_INB, HIGH );
        analogWrite(MOTOR2_PWM, spd);
      }

      // DRIVE REVERSE
      else if ( d == 0 )
      {
        digitalWrite(MOTOR2_INA, HIGH );
        digitalWrite(MOTOR2_INB, LOW );
        analogWrite(MOTOR2_PWM, spd);
      }
      break;

    case MOTOR3:
      if ( d == 1 )
      {
        digitalWrite(MOTOR3_INA, LOW );
        digitalWrite(MOTOR3_INB, HIGH );
        analogWrite(MOTOR3_PWM, spd);
      }

      // DRIVE REVERSE
      else if ( d == 0 )
      {
        digitalWrite(MOTOR3_INA, HIGH );
        digitalWrite(MOTOR3_INB, LOW );
        analogWrite(MOTOR3_PWM, spd);
      }
      break;

    case MOTOR4:
      if ( d == 1 )
      {
        digitalWrite(MOTOR4_INA, LOW );
        digitalWrite(MOTOR4_INB, HIGH );
        analogWrite(MOTOR4_PWM, spd);
      }


      // DRIVE REVERSE
      else if ( d == 0 )
      {
        digitalWrite(MOTOR4_INA, HIGH );
        digitalWrite(MOTOR4_INB, LOW );
        analogWrite(MOTOR4_PWM, spd);
      }
      break;



  }
}

void driveDirection( int d ) {

  // ALL STOP CONDITION
  if ( d == DS_STOP ) {
    motorsOff();
  }

  // DRIVE FORWARD
  else if ( d == DS_FWD ) {
    forward();
  }

  // DRIVE REVERSE
  else if ( d == DS_REV ) {
    reverse();
  }

  // TURN RUGHT
  else if ( d == DS_RT ) {
    right();
  }

  // TURN LEFT
  else if ( d == DS_LT ) {
    left();
  }

  // ERROR STATE BAD DIRECTION PROVIDED
  else {
    if (DEBUG) {
      Serial1.print("bad drive state in call to setDirection(");
      Serial1.print(d);
      Serial1.println(");");
    }
  }
}

void driveSpeed(int m, int spd)
{
  if ( spd > 255)  spd = 255;
  if ( spd < 0)    spd = 0;

  switch (m)
  {
    case MOTOR1:
      analogWrite(MOTOR1_PWM, spd);
      break;

    case MOTOR2:
      analogWrite(MOTOR2_PWM, spd);
      break;

    case MOTOR3:
      analogWrite(MOTOR3_PWM, spd);
      break;

    case MOTOR4:
      analogWrite(MOTOR4_PWM, spd);
      break;

    case MOTORS_LEFT:
      analogWrite(MOTOR3_PWM, spd);
      analogWrite(MOTOR4_PWM, spd);
      break;

    case MOTORS_RIGHT:
      analogWrite(MOTOR1_PWM, spd);
      analogWrite(MOTOR2_PWM, spd);
      break;

    case MOTORS_ALL:
      analogWrite(MOTOR1_PWM, spd);
      analogWrite(MOTOR2_PWM, spd);
      analogWrite(MOTOR3_PWM, spd);
      analogWrite(MOTOR4_PWM, spd);
      break;

    default:
      if (DEBUG) Serial1.println("Error in driveSpeed()");
      break;
  }
}




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void forward()
{
  digitalWrite(MOTOR1_INA, LOW );
  digitalWrite(MOTOR1_INB, HIGH );
  digitalWrite(MOTOR2_INA, LOW );
  digitalWrite(MOTOR2_INB, HIGH );
  digitalWrite(MOTOR3_INA, HIGH );
  digitalWrite(MOTOR3_INB, LOW );
  digitalWrite(MOTOR4_INA, HIGH );
  digitalWrite(MOTOR4_INB, LOW );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void reverse()
{
  digitalWrite(MOTOR1_INA, HIGH );
  digitalWrite(MOTOR1_INB, LOW );
  digitalWrite(MOTOR2_INA, HIGH );
  digitalWrite(MOTOR2_INB, LOW );
  digitalWrite(MOTOR3_INA, LOW );
  digitalWrite(MOTOR3_INB, HIGH );
  digitalWrite(MOTOR4_INA, LOW );
  digitalWrite(MOTOR4_INB, HIGH );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void left()
{
  digitalWrite(MOTOR1_INA, LOW );
  digitalWrite(MOTOR1_INB, HIGH );
  digitalWrite(MOTOR2_INA, LOW );
  digitalWrite(MOTOR2_INB, HIGH );
  digitalWrite(MOTOR3_INA, LOW );
  digitalWrite(MOTOR3_INB, HIGH );
  digitalWrite(MOTOR4_INA, LOW );
  digitalWrite(MOTOR4_INB, HIGH );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void right()
{
  digitalWrite(MOTOR1_INA, HIGH );
  digitalWrite(MOTOR1_INB, LOW );
  digitalWrite(MOTOR2_INA, HIGH );
  digitalWrite(MOTOR2_INB, LOW );
  digitalWrite(MOTOR3_INA, HIGH);
  digitalWrite(MOTOR3_INB, LOW );
  digitalWrite(MOTOR4_INA, HIGH );
  digitalWrite(MOTOR4_INB, LOW );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void motorsOff()
{
  digitalWrite(MOTOR1_INA, LOW );
  digitalWrite(MOTOR1_INB, LOW );
  digitalWrite(MOTOR2_INA, LOW );
  digitalWrite(MOTOR2_INB, LOW );
  digitalWrite(MOTOR3_INA, LOW );
  digitalWrite(MOTOR3_INB, LOW );
  digitalWrite(MOTOR4_INA, LOW );
  digitalWrite(MOTOR4_INB, LOW );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setupMotorControlPins()
{
  // MOTOR 1 INIT
  pinMode(MOTOR1_INA, OUTPUT);
  pinMode(MOTOR1_INB, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);

  // MOTOR 2 INIT
  pinMode(MOTOR2_INA, OUTPUT);
  pinMode(MOTOR2_INB, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  // MOTOR 3 INIT
  pinMode(MOTOR3_INA, OUTPUT);
  pinMode(MOTOR3_INB, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);

  // MOTOR 4 INIT
  pinMode(MOTOR4_INA, OUTPUT);
  pinMode(MOTOR4_INB, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
}

void waitForStartupMessage() {
  bool gotit = false;
  String message = F("KATSBOT2019");
  int index = 0;
  unsigned long startTime = millis();
  unsigned long timeout = 60000;
  
  while( !gotit ){
    if( millis() - startTime > timeout ){
      return;
    }
    if( Serial1.available() ){
      if( Serial.peek()==message[index] ){
        index++;
      } else {
        index =0;
      }
      if( index==11 ){
        gotit = true;
      }
    }
  }
}

int batterySampleInterval=500;
unsigned long lastBatterySample=0;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(100);

  setupMotorControlPins();

  pinMode(A0, INPUT);

  driveDirection(DS_STOP);
  driveSpeed(MOTORS_ALL, 0);


  //waitForStartupMessage();
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop() {
  // PROCESS Serial1 MESSAGES AND DRIVE MOTORS ACCORDINGLY
//
//  if( millis()-lastBatterySample > batterySampleInterval ){
//    float value = 0.0;
//    unsigned long sum = 0;
//    int i=0;
//    for( i=0; i<10; i++) {
//      sum += analogRead(A0);
//      delay(5);
//    }
//    value = 3.3*((float)sum/10.0/1023.0)/(10.0/59.0);
//    Serial.println(value, 2);
//    lastBatterySample=millis();
//  }

  while (Serial1.available() > 0) {
    // read the incoming byte:
    char tmp = Serial1.read();

    if (tmp == '\n')
    {
      if ( data_in == "stop" || data_in == "STOP" ) {
        driveDirection(DS_STOP);
        driveSpeed(MOTORS_ALL, 0);
      } 
      else if(data_in == "battery" || data_in == "BATTERY") {
        float value = 0.0;
        unsigned long sum = 0;
        int i=0;
        for( i=0; i<10; i++) {
          sum += analogRead(A0);
          delay(5);
        }
        sum/=10;
        value = 3.3*((float)sum/1023.0)/(10.0/59.0);
        Serial1.println(value, 2);
        Serial.println(value, 2);
      } else {

        //FORMAT: 'M' proceeded by four four character substrings consisting of 0 or 1 for direction and a three digit pwm value
        //Example: 'M1255025511270127' would correspond to:
        //Motor 1 fwd at full speed, Motor 2 rvs at full speed, Motor 3 fwd at half speed, Motor 4 rvs at half speed
        drive(MOTOR1, data_in.substring(1, 2).toInt(), data_in.substring(2, 5).toInt());
        drive(MOTOR2, data_in.substring(5, 6).toInt(), data_in.substring(6, 9).toInt());
        drive(MOTOR3, data_in.substring(9, 10).toInt(), data_in.substring(10, 13).toInt());
        drive(MOTOR4, data_in.substring(13, 14).toInt(), data_in.substring(14, 17).toInt());

      }
      Serial1.println(data_in);
      data_in = "";
      break;
    }
    else data_in += tmp;
  }

}

