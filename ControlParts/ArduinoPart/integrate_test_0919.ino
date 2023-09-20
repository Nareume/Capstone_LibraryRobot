#include <Arduino.h>
// ... Micro-ROS Includes ...
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

// error debugging
#define LED_PIN 13

// define absolute values which we use
#define P_GAIN 15.0
#define I_GAIN 35.0
#define D_GAIN 20.0

#define MAX_SPEED 1.0 // Max speed in m/s
#define MAX_PWM 255 // Max PWM value

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 100;  // Interval in milliseconds

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist t_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
nav_msgs__msg__Odometry odom;

// Robot specifications
const double wheelDiameter = 0.13;
const double wheelbase = 0.30;
const double wheelCircumference = wheelDiameter * PI;
const double encoderPulsesPerRevolution = 741.0; 
const double distancePerPulse = wheelCircumference / encoderPulsesPerRevolution; 

float targetSpeedLinear = 0.0;
float targetSpeedAngular = 0.0;
float linearSpeed = 0.0;
float angularSpeed = 0.0;

float pos_x = 0;
float pos_y = 0;
float angle_z = 0;

float qx = 0;
float qz = 0;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Update targetSpeed based on the incoming cmd_vel Twist message
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * t_msg = (const geometry_msgs__msg__Twist *)msgin;
  targetSpeedLinear = t_msg->linear.x;
  targetSpeedAngular = t_msg->angular.z;
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

void calc_quat(float theta, float &qx, float &qz) {
  float cos_half_theta = cos(theta / 2.0);
  float sin_half_theta = sin(theta / 2.0);

  qx = cos_half_theta;
  qz = sin_half_theta;
}

void setup() {
  // ... Micro-ROS Setup ...
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // ... Other Motor and Encoder Setup ...
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
  Serial.begin(115200);

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

  // ... Other Executor and Timer Setup ...
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &t_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}



void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calculate wheel speeds
    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    Serial.print("leftEncoderCount: ");
    Serial.print(leftEncoderCount);
    Serial.print(", ");
    Serial.print("rightEncoderCount: ");
    Serial.print(rightEncoderCount);
    Serial.print(", ");
    
    // Reset encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Calculate linear and angular speed
    linearSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    angularSpeed = (rightWheelSpeed - leftWheelSpeed) / wheelbase;

    serActualSpeeds();

    pos_x += dt * ((leftWheelSpeed+rightWheelSpeed)/2) * cos(angle_z);
    pos_y += dt * ((leftWheelSpeed+rightWheelSpeed)/2) * sin(angle_z);
    angle_z += dt * ((rightWheelSpeed-leftWheelSpeed)/wheelbase);

    serActualOdom();

    calc_quat(angle_z, qx, qz);

    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.z = qz;
    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
  }

  // Update motor speeds based on targetSpeedLinear and targetSpeedAngular
  teleop();
  
  float rightWheelSpeed = targetSpeedLinear + targetSpeedAngular * wheelbase / 2;
  float leftWheelSpeed = targetSpeedLinear - targetSpeedAngular * wheelbase / 2;

  moveMotorSimple(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, leftWheelSpeed);
  moveMotorSimple(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, rightWheelSpeed);
}

void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void moveMotorSimple(int motorPinPWM, int motorPin1, int motorPin2, float speed) {
  int pwmSpeed = map(speed * 1000, 0, MAX_SPEED * 1000, 0, MAX_PWM);
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

void serTargetSpeeds() {
 // Output the target speeds
  Serial.print("Target Linear Speed: ");
  Serial.print(targetSpeedLinear);
  Serial.print(" m/s, Target Angular Speed: ");
  Serial.print(targetSpeedAngular);
  Serial.println(" rad/s");
}

void serActualSpeeds() {
     // Output the actual speeds
  Serial.print("Linear Speed: ");
  Serial.print(linearSpeed);
  Serial.print(" m/s, Angular Speed: ");
  Serial.print(angularSpeed);
  Serial.println(" rad/s");
}

void serActualOdom() {
    Serial.print("x: ");
    Serial.print(pos_x);
    Serial.print(", ");
    Serial.print("y: ");
    Serial.print(pos_y);
    Serial.print(", ");
    Serial.print("z: ");
    Serial.print(angle_z);
    Serial.println();
}

void teleop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
    
        if (command == "w") {
        targetSpeedLinear += 0.5;
        targetSpeedAngular = 0.0;
        Serial.println("FORWARD");
        serTargetSpeeds();
        
        } else if (command == "s") {
        targetSpeedLinear = 0.0;
        targetSpeedAngular = 0.0;
        Serial.println("STOP");
        serTargetSpeeds();
        
        } else if (command == "a") {
        targetSpeedLinear = 0.0;
        targetSpeedAngular += 0.5;
        Serial.println("LEFT");
        serTargetSpeeds();
        
        } else if (command == "d") {
        targetSpeedLinear = 0.0;
        targetSpeedAngular -= 0.5;
        Serial.println("RIGHT");
        serTargetSpeeds();
        
        } else if (command == "x") {
        targetSpeedLinear -= 0.5;
        targetSpeedAngular = 0.0;
        Serial.println("BACK");
        serTargetSpeeds();
    
        }  
    }
}