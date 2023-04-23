

#include "CytronMotorDriver.h"
// Configure the motor driver.
CytronMD motor1(PWM_DIR, 3, 2);  // PWM 1 = Pin 2, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 5, 4); // PWM 2 = Pin 3, DIR 2 = Pin 5.

//motor 1 -> left, motor 2 -> right

#include <Encoder.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <MotorDriver.h>
ros::NodeHandle nh;
geometry_msgs::Twist msg;

float move1;
float move2;
//int linearRight = 67;
//int linearLeft = 70;
//int angularSpeed = 45;

//encoded motors inports-------------------
#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 20

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21

Encoder leftMotor(ENC_IN_LEFT_A, ENC_IN_LEFT_B);
Encoder rightMotor(ENC_IN_RIGHT_A, ENC_IN_RIGHT_B);

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

//speed parameters
const int linearSafe = 100;
const int rotSafe = 60;
const float linearGradient = linearSafe/0.22;
const float angGradient = rotSafe/2.84;
const float offset = 0.95714;
//call back function for subscriber

void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
  if (move2 == 0)
  {
    linear(-linearGradient * move1);
  }
  else if (move1 == 0 )
  {
    angular(angGradient * move2);
  }
  else if (abs(move1/0.22) < abs(move2/2.84)) {
    angular(angGradient * move2);
  }

  else if (abs(move1/0.22) > abs(move2/2.84)) {
    linear(-linearGradient * move1);
  }
  else
  {
    die();
  }
}


// Keep track of the number of wheel ticks
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &callback);

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);





// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;


//use only two motors here
const int rightback = 4;
const int leftback = 3;

void setup() {
  // pinMode(leftforw, OUTPUT);
  pinMode(leftback, OUTPUT);
  // pinMode(rightforw, OUTPUT);
  pinMode(rightback, OUTPUT);

  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
  nh.subscribe(sub);
 
}

void left_wheel_tick(){
  left_wheel_tick_count.data = static_cast<int16_t>(-leftMotor.read());
  
}
void right_wheel_tick(){
  right_wheel_tick_count.data = static_cast<int16_t>(rightMotor.read());  
  
}
void loop() {
  currentMillis = millis();
  leftPub.publish( &left_wheel_tick_count );
  rightPub.publish( &right_wheel_tick_count );
  // If 100ms have passed, print the number of ticks
  nh.spinOnce();
  delay(100);
}

void linear(float speed)
{
  motor1.setSpeed(speed);
  motor2.setSpeed(speed *offset);
  delay(100);
}

void angular(float speed)
{
  motor1.setSpeed(speed);
  motor2.setSpeed(-speed*offset);
}

void die()
{
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
