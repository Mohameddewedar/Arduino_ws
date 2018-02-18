#include <PID_v1.h>
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

double d1_actual;
double d2_actual;
double d3_actual;

double d1_desired = 500;
double d2_desired = 334;
double d3_desired = 270;

float beta;

double Kp = 0.7;
double Ki = 0.3;
double Kd = 0.00001;

PID lin1_PID(&d1_actual, &lin1_vel, &d1_desired, Kp, Ki, Kd, DIRECT);
PID lin2_PID(&d2_actual, &lin2_vel, &d2_desired, Kp, Ki, Kd, DIRECT);
PID lin3_PID(&d3_actual, &lin3_vel, &d3_desired, Kp, Ki, Kd, DIRECT);

inline bool dir(float vel) {
  if (vel >= 0)
    return true;
  else
    return false;
}

bool do_linear(int linear_number, float desired_length) {
  switch (linear_number) {
  case 1:
    d1_actual = map(analogRead(LINEAR1_FB), 29.0, 663.0, 420.0, 590.0);
    lin1_PID.Compute();
    // Serial.print(d1_actual - d1_desired);
    // Serial.print("\t");
    // Serial.println(lin1_vel);
    if (abs(lin1_vel) < 1) {
      analogWrite(LINEAR1_PWM, 0);
      if (init_arm)
        Serial.println("ONE :Done");
      return true;
    } else if (init_arm || (d1_desired >= 420 && d1_desired <= 580)) // In Range
    {
      analogWrite(LINEAR1_PWM, PWM_VEL * abs(lin1_vel));
      digitalWrite(LINEAR1_DIR, dir(lin1_vel));
      return false;
    } else {
      analogWrite(LINEAR1_PWM, 0);
      Serial.print("  Linear 1 : Out of Range     ");
      return true;
    }

    { // Old Control Without PID
      // if (abs(desired_length - d1_actual) < 2) {
      //   analogWrite(LINEAR1_PWM, 0);
      //   return true;
      // } else if (init_arm || (d1_desired >= 420 && d1_desired <= 580)) // In
      // Range
      // {
      //   if (d1_actual > desired_length) {
      //     digitalWrite(LINEAR1_DIR, LOW);
      //   } else {
      //     digitalWrite(LINEAR1_DIR, HIGH);
      //   }
      //   analogWrite(LINEAR1_PWM, PWM_VEL);
      // } else {
      //   analogWrite(LINEAR1_PWM, 0);
      //   Serial.print("  Linear 1 : Out of Range     ");
      //   return true;
      // }
    }

    // Serial.print(d1_actual );
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 2:
    d2_actual = map(analogRead(LINEAR2_FB), 99.0, 398.0, 330.0, 380.0);
    lin2_PID.Compute();
    if (abs(lin2_vel) <1) {
      analogWrite(LINEAR2_PWM, 0);
      if (init_arm)
        Serial.println("TWO :Done");
      return true;
    } else if (init_arm || (d2_desired > 330 && d2_desired < 400)) // In Range
    {
      analogWrite(LINEAR2_PWM, PWM_VEL * abs(lin2_vel));
      digitalWrite(LINEAR2_DIR, dir(lin2_vel));
      return false;
    } else {
      analogWrite(LINEAR2_PWM, 0);
      Serial.print("  Linear 2 : Out of Range     ");
      return true;
    }

    { // Old Control Without PID
      // if (abs(desired_length - d2_actual) < 2) {
      //   analogWrite(LINEAR2_PWM, 0);
      //   return true;
      // } else if (init_arm || (d2_desired > 330 && d2_desired < 400)) // In
      // Range
      // {
      //   if (d2_actual > desired_length) {
      //     analogWrite(LINEAR2_PWM, PWM_VEL);
      //     digitalWrite(LINEAR2_DIR, LOW);
      //   } else {
      //     analogWrite(LINEAR2_PWM, PWM_VEL);
      //     digitalWrite(LINEAR2_DIR, HIGH);
      //   }
      // } else {
      //   analogWrite(LINEAR2_PWM, 0);
      //   Serial.print("  Linear 2 : Out of Range     ");
      //   return true;
      // }
    }

    // Serial.print(d2_actual);
    // Serial.print("        ");
    // Serial.print(desired_length);
    break;

  case 3:
    d3_actual = map(analogRead(LINEAR3_FB), 44.0, 428.0, 260.0, 300.0);
    lin3_PID.Compute();
    if (abs(lin3_vel) < 1) {
      analogWrite(LINEAR3_PWM, 0);
      if (init_arm)
        Serial.println("THREE :Done");
      return true;
    } else if (init_arm || (d3_desired > 260 && d3_desired < 300)) // In Range
    {
      analogWrite(LINEAR3_PWM, PWM_VEL * abs(lin3_vel));
      digitalWrite(LINEAR3_DIR, dir(lin3_vel));
      return false;
    } else {
      analogWrite(LINEAR3_PWM, 0);
      Serial.print("  Linear 3 : Out of Range     ");
      return true;
    }

    { // Old Control Without PID
      // {if (abs(desired_length - d3_actual) < 2) {
      //   analogWrite(LINEAR3_PWM, 0);
      //   return true;
      // } else if (init_arm || (d3_desired > 260 && d3_desired < 300)) // In
      // Range
      // {
      //   if (d3_actual > desired_length) {
      //     analogWrite(LINEAR3_PWM, PWM_VEL);
      //     digitalWrite(LINEAR3_DIR, LOW);
      //     // Serial.print("Up");
      //   } else {
      //     analogWrite(LINEAR3_PWM, PWM_VEL);
      //     digitalWrite(LINEAR3_DIR, HIGH);
      //     // Serial.print("Down");
      //   }
      // } else {
      //   analogWrite(LINEAR3_PWM, 0);
      //   Serial.print("  Linear 3 : Out of Range     ");
      //   return true;
      // }
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

  delay(50);
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

  Serial.begin(115200);
  Serial.println("Serial Estableshed");

  bool home_reached = false;

  lin1_PID.SetOutputLimits(-1, 1);
  lin2_PID.SetOutputLimits(-1, 1);
  lin3_PID.SetOutputLimits(-1, 1);

  lin1_PID.SetMode(AUTOMATIC);
  lin2_PID.SetMode(AUTOMATIC);
  lin3_PID.SetMode(AUTOMATIC);

  Serial.println("PID set & Turned ON...");

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