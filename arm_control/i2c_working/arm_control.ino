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

#define NO_REC_VALS 7 // d1 d2 d3 beta new_link twist gripper

Servo new_link_servo;

char rec;
String out;
int count;
String recArray[NO_REC_VALS - 1]; // 0-Based Indexing
int i = 0;
bool valid = false;
bool init_arm = true;
bool postion_reached = false;

float d1_actual;
float d2_actual;
float d3_actual;

const int numReadings = 10;
int readIndex = 0;
int d1_total = 0;
int d2_total = 0;
int d3_total = 0;
float d1_actual_readings[numReadings] = {0};
float d2_actual_readings[numReadings] = {0};
float d3_actual_readings[numReadings] = {0};

float d1_desired = 451;
float d2_desired = 334;
float d3_desired = 260;

float beta;
float new_link = 90;

float reading_feedback(int linear_number) {
  int inputPin;
  float *readings;
  float total;
  switch (linear_number) {
  case 1:
    readings = d1_actual_readings;
    inputPin = LINEAR1_FB;
    total = d1_total;
    break;
  case 2:
    readings = d2_actual_readings;
    inputPin = LINEAR2_FB;
    total = d2_total;
    break;
  case 3:
    readings = d3_actual_readings;
    inputPin = LINEAR3_FB;
    total = d3_total;
    break;
  default:
    break;
  }
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  switch (linear_number) {
  case 1:
    d1_total = total;
    break;
  case 2:
    d2_total = total;
    break;
  case 3:
    d3_total = total;
    break;
  default:
    break;
  }
  // calculate the average:
  return total / numReadings;
  delay(1);
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

void angles_ctrl(int n) {
  // Serial.print("Voala:  ");
  rec = '0';
  out = "";
  while (Wire.available() > 0) {

    rec = Wire.read();
    if (rec == '*') {
      Serial.println(rec);
      valid = false;
    }
    if (valid && rec != ',') {
      Serial.print(rec);
      out += rec;
      // Serial.print(out);
    } else if (valid && rec == ',') {
      Serial.print(rec);
      recArray[i] = out;
      out = "";
      i++;
    }
    if (!valid && rec == '$') {
      Serial.print(rec);
      valid = true;
      i = 0;
    }
  }

  d1_desired = recArray[0].toInt();
  d2_desired = recArray[1].toInt();
  d3_desired = recArray[2].toInt();
  beta = recArray[3].toInt();

  new_link = recArray[4].toInt();

  if (new_link != 0) {
    new_link_servo.write(new_link);
  } else {
    new_link_servo.write(90);
  }

  // Serial.println(new_link);

  do_linear(1, d1_desired);
  do_linear(2, d2_desired);
  do_linear(3, d3_desired);

  Serial.print("d1=  ");
  Serial.print(d1_desired);
  Serial.print("      ");
  Serial.print("d2=  ");
  Serial.print(d2_desired);
  Serial.print("      ");
  Serial.print("d3=  ");
  Serial.print(d3_desired);
  Serial.print("      ");
  Serial.print("New Link Servo=  ");
  Serial.println(new_link);
  delay(10);
}

void setup() {
  pinMode(LINEAR1_PWM, OUTPUT);
  pinMode(LINEAR1_DIR, OUTPUT);

  pinMode(LINEAR2_PWM, OUTPUT);
  pinMode(LINEAR2_DIR, OUTPUT);

  pinMode(LINEAR3_PWM, OUTPUT);
  pinMode(LINEAR3_DIR, OUTPUT);

  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);

  new_link_servo.attach(SERVO1);
  new_link_servo.write(90);
  Serial.begin(115200);
  Serial.println("Serial Estableshed");

  bool home_reached = false;

  d1_desired = 451; // 451
  d2_desired = 334; // 334
  d3_desired = 260; // 260

  Serial.println("Going to Home Position...");

  while (!home_reached) {
    home_reached = do_linear(1, d1_desired) && do_linear(2, d2_desired) &&
                   do_linear(3, d3_desired);
  }
  Serial.println("Home Position Reached..");
  init_arm = false;
  Wire.begin(44);
  Wire.onReceive(angles_ctrl);
  Serial.println("I2C Estableshed");
  Serial.println(".... Waiting ....");
}

void loop() { delay(2); }