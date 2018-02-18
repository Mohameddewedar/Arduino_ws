#include <Wire.h>

#define RIGHT_FRONT 11
#define RIGHT_MIDDLE 10
#define RIGHT_REAR 9
#define LEFT_FRONT 3
#define LEFT_MIDDLE 5
#define LEFT_REAR 6
#define RIGHT_DIRECTION 4
#define LEFT_DIRECTION 12

#define NO_REC_VALS 2

char rec;
String out;
int count;
String recArray[NO_REC_VALS-1];
int i = 0;
bool valid = false;

float v, vr, vl, theta = 0;

inline bool dir(float vel) {
  if (vel >= 0)
    return true;
  else
    return false;
}

void vel_ctrl(int n) {
  // if (recArray[0] < 0)
  //   theta += M_PI;
  // while (theta >= 2 * M_PI || theta < 0)
  // {
  //   if (theta >= 2 * M_PI)
  //     theta -= 2 * M_PI;
  //   if (theta < 0)
  //     theta += 2 * M_PI;
  // }

  Serial.print("DOES :");
  // Serial.println(n);
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
      // Serial.print(",");
      recArray[i] = out;
      out = "";
      i++;
    }
    if (!valid && rec == '$') {
      valid = true;
      i = 0;
    }
  }

  // Serial.print("First number =");
  // Serial.print(recArray[0]);
  // Serial.print("  Second number =");
  // Serial.println(recArray[2]);

  theta = recArray[1].toInt() / 100.00;

  if (theta >= 0) {
    vl = 255 * abs(recArray[0].toInt() / 999.0) * (1 - sin(abs(theta)));
    vr = 255 * abs(recArray[0].toInt() / 999.0);
  } else {
    vr = 255 * abs(recArray[0].toInt() / 999.0) * (1 - sin(abs(theta)));
    vl = 255 * abs(recArray[0].toInt() / 999.0);
  }

  analogWrite(RIGHT_FRONT, abs(vr));
  analogWrite(RIGHT_MIDDLE, abs(vr));
  analogWrite(RIGHT_REAR, abs(vr));
  analogWrite(LEFT_FRONT, abs(vl));
  analogWrite(LEFT_MIDDLE, abs(vl));
  analogWrite(LEFT_REAR, abs(vl));
  digitalWrite(RIGHT_DIRECTION, dir(recArray[0].toInt() / 999.0));
  digitalWrite(LEFT_DIRECTION, !(dir(recArray[0].toInt() / 999.0)));
  // Serial.print(vl);
  // Serial.print(",");
  // Serial.print(vr);
  // Serial.print(",");
  // Serial.print(recArray[0]);
  // Serial.print(",");
  // Serial.println(theta);
}

void setup() {
  pinMode(RIGHT_FRONT, OUTPUT);     // Right Front
  pinMode(RIGHT_MIDDLE, OUTPUT);    // Right Middle
  pinMode(RIGHT_REAR, OUTPUT);      // Right Rear
  pinMode(LEFT_FRONT, OUTPUT);      // Left Front
  pinMode(LEFT_MIDDLE, OUTPUT);     // Left Middle
  pinMode(LEFT_REAR, OUTPUT);       // Left Rear
  pinMode(RIGHT_DIRECTION, OUTPUT); // Right Direction
  pinMode(LEFT_DIRECTION, OUTPUT);  // Left Direction
  Serial.begin(115200);
  // Serial.println("Serial Estableshed");
  Wire.begin(42);
  Wire.onReceive(vel_ctrl);
  // Serial.println("I2C Estableshed");
}

void loop() { delay(1); }
