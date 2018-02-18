#define FEEDBACK_PIN A2
#define PWM_PIN 11
#define DIR_PIN 8
#include <time.h>

void setup() {
  // pinMode(FEEDBACK_PIN, INPUT);
//  pinMode(PWM_PIN, OUTPUT);
//  pinMode(DIR_PIN, OUTPUT);
  // Serial.begin(9600);
}

void loop() {
  // analogWrite(PWM_PIN, 255);
  // digitalWrite(DIR_PIN, LOW);
Serial.print(time(NULL));
  // Serial.println(analogRead(PIN));
}
