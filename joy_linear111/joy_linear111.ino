/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include "Servo.h"
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
// #include <stdlib.h>

ros::NodeHandle nh;
float x, y;
float x1, x2;
float twist_ang = 90;
// const int z = A2;
// const int z = A1;
const int z = A0;

String s;
char b[10];
char d[10];

Servo new_link_servo;
Servo gripper_servo;
Servo twist_servo;

// String s1;
// char b1[10];
// char d1[10];
// String s2;
// char b1[10];
// char d1[10];

void messageCb(const sensor_msgs::Joy &joy_msg) {
  x = (joy_msg.axes[0]) * 255;
  x1 = (joy_msg.axes[1]) * 255;
  x2 = (joy_msg.axes[2]) * 255;

  if (joy_msg.buttons[10] ^ joy_msg.buttons[11])
    if (joy_msg.buttons[11]) {
      digitalWrite(4, HIGH);
      analogWrite(9, 255);
      nh.loginfo("High");
    } else {
      digitalWrite(4, LOW);
      analogWrite(9, 255);
      nh.loginfo("LOW");
    }
  else {
    analogWrite(9, 0);
    nh.loginfo("0");
  }

  ///////////////////////////////////////////////////////////////////////
  if (joy_msg.buttons[8] ^ joy_msg.buttons[9])
    if (joy_msg.buttons[9]) {
      digitalWrite(7, HIGH);
      analogWrite(10, 255);
      nh.loginfo("High");
    } else {
      digitalWrite(7, LOW);
      analogWrite(10, 255);
      nh.loginfo("LOW");
    }
  else {
    analogWrite(10, 0);
    nh.loginfo("0");
  }

  ////////////////////////////////////////////////////////////////////////////

  if (joy_msg.buttons[6] ^ joy_msg.buttons[7])
    if (joy_msg.buttons[7]) {
      digitalWrite(8, HIGH);
      analogWrite(11, 100);
      nh.loginfo("High");
    } else {
      digitalWrite(8, LOW);
      analogWrite(11, 100);
      nh.loginfo("LOW");
    }
  else {
    analogWrite(11, 0);
    nh.loginfo("0");
  }

  twist_ang -= joy_msg.axes[5];
  twist_servo.write(twist_ang);

  if (joy_msg.buttons[3]) {
    gripper_servo.write(125);
  }
  if (joy_msg.buttons[5]) {
    gripper_servo.write(142);
  }
  new_link_servo.write((joy_msg.axes[3] + 1) * 90);
}
ros::Subscriber<sensor_msgs::Joy> sub("joy", messageCb);

void setup() {
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  y = analogRead(z);
  s = dtostrf(y, 6, 1, b);
  s.toCharArray(d, 10);

  new_link_servo.attach(3);
  gripper_servo.attach(6);
  twist_servo.attach(5);
  nh.spinOnce();
  delay(10);
  // nh.loginfo(d);
}
