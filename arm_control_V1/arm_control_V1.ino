/*
This code is for controlling the first version of the Robotic Arm

The Arm consists of :-
    1) 3 Linear Actuators.
    2) 3 Servo Motors.
    **Base control is not included in this code**

The Arm Configuration is as follows:-
    1) Linear Actuator.
    2) Linear Actuator.
    3) Linear Actuator.
    4) Servo.
    5) Servo.
    6) Servo.

*/

#include <Servo.h>
#include <Wire.h>

#define LINEAR1_PWM 9 // PWM
#define LINEAR1_DIR 4 // Direction
#define LINEAR1_FB A2 // Feedback

#define LINEAR2_PWM 10 // PWM
#define LINEAR2_DIR 7  // Direction
#define LINEAR2_FB A1  // Feedback

#define LINEAR3_PWM 11 // PWM
#define LINEAR3_DIR 8  // Direction
#define LINEAR3_FB A0  // Feedback

#define SERVO1 3
#define SERVO2 5
#define SERVO3 6

#define PWM_VEL 255 // Velocity of Linear Actuators

#define NO_REC_VALS 6 // Ture number (1-Based index)
// Still Unknown (may icrease)
/*

Sugguestions for the commands fields (Generic):-
index-->    0)Speed x (mm/sec) [Time Problem ???? (i.e sync/delay)]
            1)Speed y (mm/sec)
            2)Speed z (mm/sec)
            3)Angle of approach [n] (Degrees)
            4)Modes (0-->Normal Mode of Operation ,1-->Home Position
                                            ,2-->Position_1 , ...etc)
            5)Increment in links (Trinary) (Including Gripper and
Twist Servos) (i.e 01201.. means 1st link static, 2nd link forward, 3rd link
backward,...etc). (NOTE 000... means static or inverse)

Feedback:-

*/

char rec;
String out;
int count;
String recArray[NO_REC_VALS - 1];
String links_states;

String feedback_str;

int i = 0;
bool valid = false;
bool init_arm = true;

float d1_desired = 451;
float d2_desired = 334;
float d3_desired = 260;

float d1_actual;
float d2_actual;
float d3_actual;
int feedback_str_length=20;

void send_feedback(){ // not the sending code yet
feedback_str="$"+String(feedback_str_length,0)+","+String(d1_actual,0)+","+String(d2_actual,0)+","+String(d3_actual,0)+"*";
feedback_str_length=feedback_str.length();
Wire.write(feedback_str);
}

bool do_linear(int linear_number, float desired_length) {
  switch (linear_number) {
  case 1:
    d1_actual = map(analogRead(LINEAR1_FB), 29.0, 663.0, 420.0, 590.0);
    // d1_actual = map(reading_feedback(1), 29.0, 663.0, 420.0, 590.0);
    if (abs(desired_length - d1_actual) < 2) {
      analogWrite(LINEAR1_PWM, 0);
      return true;
    } else if (init_arm || (d1_desired >= 420 && d1_desired <= 580)) // In Range
    {
      // Serial.println("Je Suis Avec l' Arduino, J'aime l' Arduino");
      analogWrite(LINEAR1_PWM, PWM_VEL);
      if (d1_actual > desired_length)
        digitalWrite(LINEAR1_DIR, LOW);
      else
        digitalWrite(LINEAR1_DIR, HIGH);
    } else {
      analogWrite(LINEAR1_PWM, 0);
      // Serial.print("  Linear 1 : Out of Range     ");
      return true;
    }

    // Serial.print(d1_actual );
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 2:
    d2_actual = map(analogRead(LINEAR2_FB), 99.0, 398.0, 330.0, 380.0);
    // d2_actual = map(reading_feedback(2), 99.0, 398.0, 330.0, 380.0);
    if (abs(desired_length - d2_actual) < 2) {
      analogWrite(LINEAR2_PWM, 0);
      return true;
    } else if (init_arm ||
               (d2_desired > 330 && d2_desired < 380)) // In **new Range
    {
      if (d2_actual > desired_length)
        digitalWrite(LINEAR2_DIR, LOW);
      else
        digitalWrite(LINEAR2_DIR, HIGH);
      analogWrite(LINEAR2_PWM, PWM_VEL / 2);
    } else {
      analogWrite(LINEAR2_PWM, 0);
      // Serial.print("  Linear 2 : Out of Range     ");
      return true;
    }

    // Serial.print(d2_actual);
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 3:
    d3_actual = map(analogRead(LINEAR3_FB), 44.0, 428.0, 260.0, 300.0);
    // d3_actual = map(reading_feedback(3), 44.0, 428.0, 260.0, 300.0);
    if (abs(desired_length - d3_actual) < 2) {
      analogWrite(LINEAR3_PWM, 0);
      return true;
    } else if (init_arm || (d3_desired > 260 && d3_desired < 300)) // In Range
    {
      if (d3_actual > desired_length)
        digitalWrite(LINEAR3_DIR, LOW);
      else
        digitalWrite(LINEAR3_DIR, HIGH);
      analogWrite(LINEAR3_PWM, PWM_VEL / 2);
    } else {
      analogWrite(LINEAR3_PWM, 0);
      // Serial.print("  Linear 3 : Out of Range     ");
      return true;
    }

    // Serial.print(d3_actual);
    // Serial.print("        ");
    // Serial.println(desired_length);
    break;
  default:
    break;
  }
  return false;
}

void read_i2c(int n) {
  // Serial.print("Voala:  ");
  rec = '0';
  out = "";

  while (Wire.available() > 0) {
    rec = Wire.read();
    if (rec == '*') {
      // Serial.println(rec);
      valid = false;
    }
    if (valid && rec != ',') {
      // Serial.print(rec);
      out += rec;
      // Serial.print(out);
    } else if (valid && rec == ',') {
      // Serial.print(rec);
      recArray[i] = out;
      out = "";
      i++;
    }
    if (!valid && rec == '$') {
      // Serial.print(rec);
      valid = true;
      i = 0;
    }
  }

  links_states = recArray[5];

  // First Linear Actuator
  if (links_states[0] != '0') {
    d1_desired = d1_actual + 20 * (2 * links_states[0] - 3); // 1..2 ==> -1..1
    do_linear(1, d1_desired);
  }
  // Second Linear Actuator
  if (links_states[1] != '0') {
    d2_desired = d2_actual + 20 * (2 * links_states[1] - 3); // 1..2 ==> -1..1
    do_linear(2, d2_desired);
  }
  // Third Linear Actuator
  if (links_states[2] != '0') {
    d3_desired = d3_actual + 20 * (2 * links_states[2] - 3); // 1..2 ==> -1..1
    do_linear(3, d3_desired);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Estableshed");

  Wire.begin(44);
  Wire.onReceive(read_i2c);
  Wire.onRequest(send_feedback);
  Serial.println("I2C Estableshed");
  Serial.println(".... Waiting ....");
}

void loop() { delay(2); }
