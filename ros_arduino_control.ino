
/* This is an Arduino-ROS program that allows the robot to:
   1) Receive velocity commands for the left and right motors from ROS.
   2) Send encoder pulse counts back to ROS for the left and right motors.
   3) Control the motors based on the received velocity commands.
*/

/////////////////////////////////////////////////////////////////////
//                Import Required Libraries
/////////////////////////////////////////////////////////////////////
// ROS Serial communication with Arduino and sending/receiving ROS messages.
#include <ros.h>
// Include the data type for sending and receiving integer messages.
#include <std_msgs/Int32.h>

/////////////////////////////////////////////////////////////////////
//                Arduino Pin Setup
/////////////////////////////////////////////////////////////////////
// Pin definitions for encoder and motor control.

// Left motor encoder pin (connected to interrupt).
int encoderPinLeft = 2;
// Right motor encoder pin (connected to interrupt).
int encoderPinRight = 3;

// Right motor control pins (PWM and direction).
int IN3 = 9;
int IN4 = 10;
int ENB = 11;

// Left motor control pins (PWM and direction).
int ENA = 5;
int IN1 = 6;
int IN2 = 7;

/////////////////////////////////////////////////////////////////////
//                Global Variables
/////////////////////////////////////////////////////////////////////
// Variables to keep track of encoder pulses.
volatile unsigned long totalPulsesLeft = 0;  
volatile unsigned long totalPulsesRight = 0;  

// Motor speed control variables (0 to 255), values will be set by ROS messages.
int motorVelocityLeft = 0;  
int motorVelocityRight = 0;

/////////////////////////////////////////////////////////////////////
//                ROS Setup and Communication
/////////////////////////////////////////////////////////////////////
// ROS node handle
ros::NodeHandle nh;

// Callback function for left motor velocity command from ROS.
void callBackFunctionMotorLeft(const std_msgs::Int32 &motorVelocityLeftROS){
  motorVelocityLeft = motorVelocityLeftROS.data;  
}

// Callback function for right motor velocity command from ROS.
void callBackFunctionMotorRight(const std_msgs::Int32 &motorVelocityRightROS){
  motorVelocityRight = motorVelocityRightROS.data;  
}

// ROS Publishers for sending encoder pulses.
std_msgs::Int32 leftEncoderROS;
ros::Publisher leftEncoderROSPublisher("left_encoder_pulses", &leftEncoderROS);
std_msgs::Int32 rightEncoderROS;
ros::Publisher rightEncoderROSPublisher("right_encoder_pulses", &rightEncoderROS);

// ROS Subscribers for receiving motor velocity commands.
ros::Subscriber<std_msgs::Int32> leftMotorROSSubscriber("left_motor_velocity", &callBackFunctionMotorLeft);  
ros::Subscriber<std_msgs::Int32> rightMotorROSSubscriber("right_motor_velocity", &callBackFunctionMotorRight);  

/////////////////////////////////////////////////////////////////////
//                Arduino Setup
/////////////////////////////////////////////////////////////////////
void setup() {
  // Set encoder pins as input for counting pulses.
  pinMode(encoderPinLeft, INPUT);
  pinMode(encoderPinRight, INPUT);
  // Attach interrupts for reading encoder pulses.
  attachInterrupt(digitalPinToInterrupt(encoderPinLeft), interruptFunctionLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinRight), interruptFunctionRight, RISING);

  // Motor pin setup for both left and right motors.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set motors off initially.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Initialize ROS communication.
  nh.getHardware()->setBaud(9600);
  nh.initNode();

  // Advertise publishers for encoder data.
  nh.advertise(leftEncoderROSPublisher);
  nh.advertise(rightEncoderROSPublisher);

  // Subscribe to ROS topics for motor velocity control.
  nh.subscribe(leftMotorROSSubscriber);
  nh.subscribe(rightMotorROSSubscriber);
}

/////////////////////////////////////////////////////////////////////
//                Main Loop
/////////////////////////////////////////////////////////////////////
void loop() {
  nh.spinOnce();  // Handle incoming and outgoing ROS messages.

  // Set motor speed based on the received velocity commands.
  analogWrite(ENA, motorVelocityLeft);
  analogWrite(ENB, motorVelocityRight);

  // Set motor direction: forward for left motor.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Set motor direction: forward for right motor.
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Publish encoder pulse counts to ROS.
  leftEncoderROS.data = totalPulsesLeft;
  rightEncoderROS.data = totalPulsesRight;
  leftEncoderROSPublisher.publish(&leftEncoderROS);
  rightEncoderROSPublisher.publish(&rightEncoderROS);

  nh.spinOnce();  // Ensure ROS communication is up to date.

  delay(20);  // Small delay to regulate the loop frequency.
}

/////////////////////////////////////////////////////////////////////
//                Interrupt Handlers for Encoders
/////////////////////////////////////////////////////////////////////
// Increment pulse count for left encoder.
void interruptFunctionLeft(){
  totalPulsesLeft++;
}

// Increment pulse count for right encoder.
void interruptFunctionRight(){
  totalPulsesRight++;
}
