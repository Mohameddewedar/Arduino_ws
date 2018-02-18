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

#define NO_REC_VALS 4 // d1 d2 d3 beta

double lin1_vel;
double lin2_vel;
double lin3_vel;

char rec;
String out;
int count;
String recArray[NO_REC_VALS - 1];
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

float d1_desired = 500;
float d2_desired = 334;
float d3_desired = 270;

float d1_desired_last = 500;
float d2_desired_last = 334;
float d3_desired_last = 270;

inline bool dir(float vel) {
  if (vel >= 0)
    return true;
  else
    return false;
}

float beta;
float timer, last_timer;
unsigned long d_time;
float d_linear;

float calculate_vel(int linear_number) {
  
  timer = micros();
  d_time = timer - last_timer;

  switch (linear_number) {
  case 1:
    d_linear = d1_desired - d1_desired_last;
    Serial.println();
    Serial.print("Velocity 1 :  ");
    Serial.println((d_linear / d_time) * 1000);
    break;
  case 2:
    d_linear = d2_desired - d2_desired_last;
    break;
  case 3:
    d_linear = d3_desired - d3_desired_last;
    break;
  default:
    break;
  }
  delay(100);
  last_timer = micros();

  return (d_linear / d_time) * 1000; // requrired Velocity in m/s
}

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
  delay(5);
  // calculate the average:
  return total / numReadings;
}

bool do_linear(int linear_number, float desired_length) {
  switch (linear_number) {
  case 1:
    d1_actual = map(reading_feedback(1), 29.0, 663.0, 420.0, 590.0);
    if (abs(desired_length - d1_actual) < 2) {
      analogWrite(LINEAR1_PWM, 0);
      return true;
    } else if (init_arm || (d1_desired >= 420 && d1_desired <= 580)) // In Range
    {
      if (init_arm)
        analogWrite(LINEAR1_PWM, PWM_VEL);
      else
        analogWrite(LINEAR1_PWM,
                    constrain(map(calculate_vel(1), 0, 0.0113, 0, PWM_VEL), 0,
                              PWM_VEL)); // Maping from m/s to PWM
      if (d1_actual > desired_length)
        digitalWrite(LINEAR1_DIR, HIGH);
      else
        digitalWrite(LINEAR1_DIR, LOW);
    } else {
      analogWrite(LINEAR1_PWM, 0);
      Serial.print("  Linear 1 : Out of Range     ");
      return true;
    }

    // Serial.print(d1_actual );
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 2:
    d2_actual = map(reading_feedback(2), 99.0, 398.0, 330.0, 380.0);
    if (abs(desired_length - d2_actual) < 2) {
      analogWrite(LINEAR2_PWM, 0);
      return true;
    } else if (init_arm || (d2_desired > 330 && d2_desired < 400)) // In Range
    {
      if (init_arm)
        analogWrite(LINEAR2_PWM, PWM_VEL / 2);
      else
        analogWrite(LINEAR2_PWM,
                    constrain(map(calculate_vel(2), 0, 0.042799, 0, PWM_VEL), 0,
                              PWM_VEL)); // Maping from m/s to PWM

      if (d2_actual > desired_length)
        digitalWrite(LINEAR2_DIR, LOW);
      else
        digitalWrite(LINEAR2_DIR, HIGH);
    } else {
      analogWrite(LINEAR2_PWM, 0);
      Serial.print("  Linear 2 : Out of Range     ");
      return true;
    }

    // Serial.print(d2_actual);
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 3:
    d3_actual = map(reading_feedback(3), 44.0, 428.0, 260.0, 300.0);
    if (abs(desired_length - d3_actual) < 2) {
      analogWrite(LINEAR3_PWM, 0);
      return true;
    } else if (init_arm || (d3_desired > 260 && d3_desired < 300)) // In Range
    {
      if (init_arm)
        analogWrite(LINEAR3_PWM, PWM_VEL);
      else
        analogWrite(LINEAR3_PWM,
                    constrain(map(calculate_vel(3), 0, 0.042799, 0, PWM_VEL), 0,
                              PWM_VEL)); // Maping from m/s to PWM

      if (d3_actual > desired_length)
        digitalWrite(LINEAR3_DIR, LOW);
      else
        digitalWrite(LINEAR3_DIR, HIGH);
    } else {
      analogWrite(LINEAR3_PWM, 0);
      Serial.print("  Linear 3 : Out of Range     ");
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
  Serial.print("Voala:  ");
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

  d1_desired = recArray[0].toFloat();
  d2_desired = recArray[1].toFloat();
  d3_desired = recArray[2].toFloat();
  beta = recArray[3].toFloat();

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
  Serial.println(d3_desired);

  delay(25);
}

void setup() {
  unsigned long timer;
  unsigned long last_timer;

  last_timer = micros();

  pinMode(LINEAR1_PWM, OUTPUT);
  pinMode(LINEAR1_DIR, OUTPUT);

  pinMode(LINEAR2_PWM, OUTPUT);
  pinMode(LINEAR2_DIR, OUTPUT);

  pinMode(LINEAR3_PWM, OUTPUT);
  pinMode(LINEAR3_DIR, OUTPUT);

  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);

  Serial.begin(115200);
  Serial.println("Serial Estableshed");

  for (size_t i = 0; i < 15; i++) {
    reading_feedback(1);
    delay(10);
    reading_feedback(2);
    delay(10);
    reading_feedback(3);
    delay(10);
  }
  Serial.println("Arm Calibrated");
  init_arm = true;
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
}

void loop() { delay(10); }