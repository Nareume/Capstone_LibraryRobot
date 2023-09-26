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

// ... PID control and motor control variables and defines ...
// define pin values
#define encoder0PinA 4
#define encoder0PinB 3
#define encoder1PinA 9
#define encoder1PinB 8
#define leftMotorPin1 11
#define leftMotorPin2 10
#define rightMotorPin1 5
#define rightMotorPin2 6
#define leftMotorPinPWM 12
#define rightMotorPinPWM 7
#define LED_PIN 13


// define absolute values which we use
#define P_GAIN 15.0
#define I_GAIN 35.0
#define D_GAIN 20.0
#define TARGET_SPEED 0.7

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist t_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
nav_msgs__msg__Odometry odom;

float targetSpeedLinear = 0.0; // 전진 속도
float targetSpeedAngular = 0.0; // 회전 속도

// ... PID and motor control global variables ...
// define robot specification values
const double gearboxRatio = 27.0;
const double encoderPulsesPerRevolution = 26.0;
const double wheelDiameter = 0.13;  // meter
const double wheelbase = 0.30;      // meter
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio);


// define related values in code
float currentSpeed0 = 0.00;
float currentSpeed1 = 0.00;
float targetSpeed = 0.00;
float error0 = 0.00;
float accError0 = 0.00;
float errorGap0 = 0.00;
float error1 = 0.00;
float accError1 = 0.00;
float errorGap1 = 0.00;
float motorSpeed = 0.00;
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
volatile long encoder0PosLast = 0;
volatile long encoder1PosLast = 0;
unsigned long lastUpdateTime = 0;

float pos_x = 0;
float pos_y = 0;
float angle_z = 0;

float qw = 0;
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

// 2체배 (left)
void updateEncoder0() {
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

// 2체배 (right)
void updateEncoder1() {
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

// Calculate Error in function
void calculateError(float* error, float* accError, float* errorGap, float currentSpeed, float targetSpeed) {
  *errorGap = targetSpeed - currentSpeed - *error;
  *error = targetSpeed - currentSpeed;
  *accError += *error;
}

// PID Control Function
float pidControlSystem(float error, float accError, float errorGap, float dt) {
  float pControl = P_GAIN * error;
  float iControl = I_GAIN * (accError * dt);
  float dControl = D_GAIN * (errorGap / dt);
  return abs(pControl + iControl + dControl);
}

// Moving Motor by PWM constant which is regulated 
void moveMotor(int motorPinPWM, int motorPin1, int motorPin2, float pidControl) {
  motorSpeed = constrain(abs(pidControl), 0, 255);
  analogWrite(motorPinPWM, motorSpeed);

  if (pidControl >= 0) {
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

  qz = sin_half_theta;
  qw = cos_half_theta;
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
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);
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
  // Spin for micro-ROS
  

  // Motor Control Loop
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds

  if (dt >= 0.05) {
    currentSpeed0 = (encoder0Pos - encoder0PosLast) * distancePerPulse / dt;  // 50ms
    currentSpeed1 = (encoder1Pos - encoder1PosLast) * distancePerPulse / dt;
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    encoder0PosLast = encoder0Pos;
    encoder1PosLast = encoder1Pos;
    lastUpdateTime = currentTime;

    pos_x += dt * ((currentSpeed0+currentSpeed1)/2) * cos(angle_z);
    pos_y += dt * ((currentSpeed0+currentSpeed1)/2) * sin(angle_z);
    angle_z += dt * ((currentSpeed1-currentSpeed0)/wheelbase);

    calc_quat(angle_z, qz, qw);

    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
  }

  // Update motor speeds based on targetSpeedLinear and targetSpeedAngular
  float rightWheelSpeed = targetSpeedLinear + targetSpeedAngular * wheelbase / 2;
  float leftWheelSpeed = targetSpeedLinear - targetSpeedAngular * wheelbase / 2;

  calculateError(&error0, &accError0, &errorGap0, currentSpeed0, rightWheelSpeed); // Use rightWheelSpeed here
  float pidControl0 = pidControlSystem(error0, accError0, errorGap0, dt);
  moveMotor(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, pidControl0);
  
  calculateError(&error1, &accError1, &errorGap1, currentSpeed1, leftWheelSpeed); // And use leftWheelSpeed here
  float pidControl1 = pidControlSystem(error1, accError1, errorGap1, dt);
  moveMotor(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, pidControl1);
}
