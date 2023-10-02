#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>

// Motor and Encoder Pins
#define leftEncoderPinA 3
#define leftEncoderPinB 4
#define leftMotorPin1 5
#define leftMotorPin2 6
#define leftMotorPinPWM 7

#define rightEncoderPinA 8
#define rightEncoderPinB 9
#define rightMotorPin1 10
#define rightMotorPin2 11
#define rightMotorPinPWM 12

#define LED_PIN 13

#define MAX_SPEED 1.0 // Max speed in m/s
#define MAX_PWM 255 // Max PWM value

// Global variables for motor and encoders
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 10;  // Interval in milliseconds

// Robot specifications
const double wheelDiameter = 0.13;
const double wheelbase = 0.30;
const double wheelCircumference = wheelDiameter * PI;
const double encoderPulsesPerRevolution = 741.0; 
const double distancePerPulse = wheelCircumference / encoderPulsesPerRevolution; 

// PID Global variables
float targetSpeedLinear = 0.0;
float targetSpeedAngular = 0.0;
float linearSpeed = 0.0;
float angularSpeed = 0.0;

// PID Constants and Variables
float Kp_linear = 0.5;
float Ki_linear = 0.00001;
float Kd_linear = 0.00003;

float Kp_angular = 0.3;
float Ki_angular = 0.00001;
float Kd_angular = 0.00003;

float errorLinear = 0.0;
float previousErrorLinear = 0.0;
float integralLinear = 0.0;
float derivativeLinear = 0.0;

float errorAngular = 0.0;
float previousErrorAngular = 0.0;
float integralAngular = 0.0;
float derivativeAngular = 0.0;

// Odometry values 
float rightWheelSpeed = 0.0;
float leftWheelSpeed = 0.0;

float pos_x = 0;
float pos_y = 0;
float angle_z = 0;

float qw = 0;
float qz = 0;

// Micro-ROS specific definitions
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist t_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
nav_msgs__msg__Odometry odom;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * t_msg = (const geometry_msgs__msg__Twist *)msgin;
  targetSpeedLinear = t_msg->linear.x;
  targetSpeedAngular = t_msg->angular.z;
}

void setup() {
  // Micro-ROS Setup
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "micro_ros_arduino_node_publisher"));

  // Executor and Timer setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &t_msg, &subscription_callback, ON_NEW_DATA));// Executor and Timer setup
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  

  // Motor and Encoder setup
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);

    leftEncoderCount = 0;
    rightEncoderCount = 0;

    linearSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    angularSpeed = (rightWheelSpeed - leftWheelSpeed) / wheelbase;

    // Update Odometry
    pos_x += interval * 0.001 * ((leftWheelSpeed + rightWheelSpeed) / 2) * cos(angle_z);
    pos_y += interval * 0.001 * ((leftWheelSpeed + rightWheelSpeed) / 2) * sin(angle_z);
    angle_z += interval * 0.001 * ((rightWheelSpeed - leftWheelSpeed) / wheelbase);

    // angle_z overflow 방지 
    if (angle_z > PI) { angle_z -= 2 * PI; } 
    else if (angle_z < -PI) { angle_z += 2 * PI; }
    
    // Calculate Quaternion from Current Angle (Yaw)
    calc_quat(-angle_z, qz, qw);

    // Set Odometry data
    
    
    // Calculate motor control here using PID and set motor speeds
    float linearOutput = PIDControl(targetSpeedLinear, linearSpeed, previousErrorLinear, integralLinear, Kp_linear, Ki_linear, Kd_linear);
    float angularOutput = PIDControl(targetSpeedAngular, angularSpeed, previousErrorAngular, integralAngular, Kp_angular, Ki_angular, Kd_angular);
  
    rightWheelSpeed = linearOutput + angularOutput;
    leftWheelSpeed = linearOutput - angularOutput;

    moveMotorPID(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, leftWheelSpeed);
    moveMotorPID(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, rightWheelSpeed);

    odom.pose.pose.position.x = -pos_x;
    odom.pose.pose.position.y = -pos_y;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.z = qz;

    // Publish Odometry data
    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}

void updateLeftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void updateRightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

float PIDControl(float target, float current, float &previousError, float &integral, float Kp, float Ki, float Kd) {
    float error = target - current;
    integral += error;
    float derivative = error - previousError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return output;
}

void moveMotorPID(int motorPinPWM, int motorPin1, int motorPin2, float speedd) {
  int pwmSpeed = map(speedd * 1000, 0, MAX_SPEED * 1000, 0, MAX_PWM);
  pwmSpeed = constrain(pwmSpeed, -MAX_PWM, MAX_PWM);

  analogWrite(motorPinPWM, abs(pwmSpeed));

  if (pwmSpeed >= 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}

void calc_quat(float theta, float &qz, float &qw) {
    float cos_half_theta = cos(theta / 2.0);
    float sin_half_theta = sin(theta / 2.0);

    qw = cos_half_theta;
    qz = sin_half_theta;
}
