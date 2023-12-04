#include <Arduino.h>
#include <PID_v1.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>

// Constants for Motor and Encoder Pins
#define LEFT_ENCODER_PIN_A 4
#define LEFT_ENCODER_PIN_B 3
#define LEFT_MOTOR_PIN_1 6
#define LEFT_MOTOR_PIN_2 5
#define LEFT_MOTOR_PIN_PWM 7

#define RIGHT_ENCODER_PIN_A 9
#define RIGHT_ENCODER_PIN_B 8
#define RIGHT_MOTOR_PIN_1 11
#define RIGHT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_PWM 12

#define LED_PIN 13

// Constants for Robot Specifications
#define MAX_SPEED 1.5
#define MAX_PWM 255

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 100; // Interval in milliseconds

// Robot specifications
const double WHEEL_DIAMETER = 0.13;
const double WHEELBASE = 0.30;
const double GEARBOX_RATIO = 27.0;
const double ENCODER_RESOLUTION = 26.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const double ENCODER_PULSES_PER_REVOLUTION = 741.0;
const double DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REVOLUTION;

// PID variables for left and right motors
double SetpointLeft = 0.0, InputLeft = 0.0, OutputLeft = 0.0;
double SetpointRight = 0.0, InputRight = 0.0, OutputRight = 0.0;
double SetpointLinear = 0.0, SetpointAngular = 0.0, InputLinear = 0.0, InputAngular = 0.0;

float pos_x = 0;
float pos_y = 0;
float angle_z = 0;
float qw = 0;
float qz = 0;

// PID objects for left and right motors
PID PIDLeft(&InputLeft, &OutputLeft, &SetpointLeft, 600.0, 5.0, 1.0, DIRECT);
PID PIDRight(&InputRight, &OutputRight, &SetpointRight, 600.0, 5.0, 1.0, DIRECT);

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

class Motor {
public:
  Motor(int pwmPin, int in1Pin, int in2Pin) : pwmPin(pwmPin), in1Pin(in1Pin), in2Pin(in2Pin) {}

  void move(double output) {
    if (output > 0) {
      analogWrite(pwmPin, static_cast<int>(output));
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    } else if (output < 0) {
      analogWrite(pwmPin, static_cast<int>(-output));
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    } else {
      stop();
    }
  }

  void stop() {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

private:
  int pwmPin;
  int in1Pin;
  int in2Pin;
};

class Odometry {
public:
  Odometry() : pos_x(0), pos_y(0), angle_z(0), qw(0), qz(0) {}

  void update(float leftWheelmove, float rightWheelmove, double interval) {
    pos_x += interval * ((leftWheelmove + rightWheelmove) / 2) * cos(angle_z);
    pos_y += interval * ((leftWheelmove + rightWheelmove) / 2) * sin(angle_z);
    angle_z += interval * ((rightWheelmove - leftWheelmove) / WHEELBASE);
  }

  void setPose(nav_msgs__msg__Odometry& odom) {
    odom.pose.pose.position.x = -pos_x;
    odom.pose.pose.position.y = -pos_y;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.z = qz;
  }

private:
  float pos_x;
  float pos_y;
  float angle_z;
  float qw;
  float qz;
};

class Robot {
public:
  Robot() : leftMotor(LEFT_MOTOR_PIN_PWM, LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2),
            rightMotor(RIGHT_MOTOR_PIN_PWM, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2),
            odometry(), previousMillis(0) {
    // PID objects for left and right motors
    PIDLeft = new PID(&InputLeft, &OutputLeft, &SetpointLeft, 600.0, 5.0, 1.0, DIRECT);
    PIDRight = new PID(&InputRight, &OutputRight, &SetpointRight, 600.0, 5.0, 1.0, DIRECT);
  }

  ~Robot() {
    delete PIDLeft;
    delete PIDRight;
  }

  void setupMicroROS() {
    // Micro-ROS Setup
    set_microros_transports();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    initializeROSNode();
    initializeSubscribersAndPublishers();
    initializeExecutorAndTimer();
  }

  void setupPIDControllers() {
    // PID Setup
    PIDLeft->SetMode(AUTOMATIC);
    PIDRight->SetMode(AUTOMATIC);
    PIDLeft->SetOutputLimits(-MAX_PWM, MAX_PWM);
    PIDRight->SetOutputLimits(-MAX_PWM, MAX_PWM);
  }

  void initializeROSNode() {
    // Node Initialization
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);
    pinMode(leftMotorPinPWM, OUTPUT);
    pinMode(rightMotorPinPWM, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), updateLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), updateRightEncoder, CHANGE);

    // Initialize Serial
    Serial.begin(115200);
  }

  void initializeSubscribersAndPublishers() {
    // Subscriber and Publisher Initialization
    RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

    RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "micro_ros_arduino_node_publisher"));
  }

  void initializeExecutorAndTimer() {
    // Executor and Timer setup
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &t_msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
  }

  void updateLeftEncoder() {
    if (digitalRead(LEFT_ENCODER_PIN_A) == digitalRead(LEFT_ENCODER_PIN_B)) {
      leftEncoderCount++;
    } else {
      leftEncoderCount--;
    }
  }

  void updateRightEncoder() {
    if (digitalRead(RIGHT_ENCODER_PIN_A) == digitalRead(RIGHT_ENCODER_PIN_B)) {
      rightEncoderCount++;
    } else {
      rightEncoderCount--;
    }
  }

  void updateOdometry() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Calculate speed from encoder counts
      double leftWheelSpeed = (leftEncoderCount * DISTANCE_PER_PULSE) / (interval / 1000.0);
      double rightWheelSpeed = (rightEncoderCount * DISTANCE_PER_PULSE) / (interval / 1000.0);

      // Reset encoder counts
      leftEncoderCount = 0;
      rightEncoderCount = 0;

      // Compute linear and angular setpoints from desired values
      SetpointLeft = SetpointLinear - (SetpointAngular * WHEELBASE / 2.0);
      SetpointRight = SetpointLinear + (SetpointAngular * WHEELBASE / 2.0);

      // Update Odometry
      float leftWheelmove = ((WHEEL_CIRCUMFERENCE * leftEncoderCount) / ENCODER_RESOLUTION) / (interval / 1000.0);
      float rightWheelmove = ((WHEEL_CIRCUMFERENCE * leftEncoderCount) / ENCODER_RESOLUTION) / (interval / 1000.0);

      odometry.update(leftWheelmove, rightWheelmove, interval);

      // Update the PID control inputs
      InputLeft = leftWheelSpeed;
      InputRight = rightWheelSpeed;
    }
  }

  void updateMotorSpeeds() {
    // Compute PID update for each motor
    PIDLeft->Compute();
    PIDRight->Compute();

    // Drive motors using the PID outputs
    leftMotor.move(OutputLeft);
    rightMotor.move(OutputRight);

    // Calculate actual linear and angular speeds based on motor outputs
    InputLinear = (InputLeft + InputRight) / 2.0;
    InputAngular = (InputRight - InputLeft) / WHEELBASE;

    // Calculate Quaternion from Current Angle (Yaw)
    calc_quat(-odometry.getAngle(), qz, qw);

    odom.pose.pose.position.x = -odometry.getX();
    odom.pose.pose.position.y = -odometry.getY();
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.z = qz;

    // Publish Odometry data
    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }

  void printDebugInfo() {
    // For debugging, you might want to print out values to the Serial
    PrintTargetSpeeds();
    PrintActualSpeeds();
  }

private:
  Motor leftMotor;
  Motor rightMotor;
  Odometry odometry;
  PID* PIDLeft;
  PID* PIDRight;
};
