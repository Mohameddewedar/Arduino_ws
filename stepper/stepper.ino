
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <stdlib.h>


const int t=A0;


float desired_angle=180;

void setup() {
  Serial.begin(9600);
  pinMode(4,OUTPUT);//ms
  pinMode(6,OUTPUT);//on/of
  pinMode(5,OUTPUT);//ms
  pinMode(9,OUTPUT);//clock
  pinMode(10,OUTPUT);//dir
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(10,HIGH);
}

void fun(float& desired){

  float  x= analogRead(t);

  float  current= map(x ,0 ,104 ,0 ,360 );//103 for 1_to_1 ratio //1015 for 10 ratio
  if ( current < desired-3){
    digitalWrite(10,LOW);

    for(long i=0;i< 100;i++)
    {
      digitalWrite(9,LOW);
      delayMicroseconds (65);
      digitalWrite(9,HIGH);
      delayMicroseconds (65);

    }
  }
  else if (current > desired+3 ){

    digitalWrite(10,HIGH);


    for(long i=0;i< 100;i++)
    {
      digitalWrite(9,LOW);
      delayMicroseconds(65);
      digitalWrite(9,HIGH);
      delayMicroseconds(65);

    }
  }
  else  {

    digitalWrite(9,LOW);


  }

}
void loop() {

  fun(desired_angle);
  Serial.println(map(analogRead(t),0,104,0,360));

}


















