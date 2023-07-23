// Motor Pin Configuration
#define leftMotorPin1 11
#define leftMotorPin2 10
#define rightMotorPin1 6
#define rightMotorPin2 5
#define leftMotorPinPWM 12  // Left Motor PWM Pin
#define rightMotorPinPWM 7  // Right Motor PWM Pin

// Encoder Pin Configuration
#define leftEncoderPinA 8
#define leftEncoderPinB 9
#define rightEncoderPinA 2
#define rightEncoderPinB 3

#define UPDATE_INTERVAL 100

volatile int leftEncoderPos = 0;
volatile int rightEncoderPos = 0;
int prevLeftEncoderPos = 0;
int prevRightEncoderPos = 0;
unsigned long prevLeftTime = 0;
unsigned long prevRightTime = 0;

unsigned long lastUpdateTime = 0; // The last time the speed was updated


double totalErrorLeft = 0;
double totalErrorRight = 0;
double prevErrorLeft = 0;
double prevErrorRight = 0;

// Robot specifications
const double gearboxRatio = 27.0;
const double encoderPulsesPerRevolution = 13.0 * 4;
const double wheelDiameter = 13.0; // m
const double wheelbase = 30.0; // m
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio); // m/pulse

double speed = 200;

void setup() {
  Serial.begin(9600);

  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPinPWM, OUTPUT);
  pinMode(rightMotorPinPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
}

void loop() {
    if (Serial.available() > 0) {
      char command = Serial.read();

    switch(command) {
      case 'F': // Forward
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, speed);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, speed);
        delay(150); 
        break;
      case 'B': // Backward
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, -speed);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, -speed);
        delay(150);
        break;
      case 'L': // Turn left
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, speed);
        delay(150);
        break;
      case 'R': // Turn right
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, speed);
        delay(150);
        break;
      case 'S':
        // Stop both motors
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, 0);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, 0);
        delay(150);
        break;

    }

    // After each movement command, compute the speed
    double leftSpeed = getSpeed(leftEncoderPos, prevLeftEncoderPos, prevLeftTime);
    double rightSpeed = getSpeed(rightEncoderPos, prevRightEncoderPos, prevRightTime);

    Serial.println(leftSpeed, rightSpeed);

    Serial.print("Left Speed: ");
    Serial.println(leftSpeed);

    Serial.print("Right Speed: ");
    Serial.println(rightSpeed);

    // Print the current encoder positions (pulses)
    Serial.print("Left Encoder Pulses: ");
    Serial.println(leftEncoderPos);

    Serial.print("Right Encoder Pulses: ");
    Serial.println(rightEncoderPos);

    Serial.println("---------");
    }
  
}



void moveMotor(int motorPin1, int motorPin2, int motorPinPWM, double speed) {
  digitalWrite(motorPin1, speed >= 0);
  digitalWrite(motorPin2, speed < 0);
  analogWrite(motorPinPWM, abs(speed));
}


void updateLeftEncoder() {
  updateEncoder(leftEncoderPinA, leftEncoderPinB, leftEncoderPos);
}

void updateRightEncoder() {
  updateEncoder(rightEncoderPinA, rightEncoderPinB, rightEncoderPos);
}

void updateEncoder(int encoderPinA, int encoderPinB, volatile int &encoderPos) {
  int MSB = digitalRead(encoderPinA); 
  int LSB = digitalRead(encoderPinB); 

  int encoded = (MSB << 1) | LSB; 
  int sum = (encoderPos << 2) | encoded; 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;
}



double getSpeed(volatile int &currentEncoderPos, int &prevEncoderPos, unsigned long &prevTime) {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - prevTime; // Time since last check, in ms

//  if (deltaTime == 0) { // if deltaTime is 0, return 0 to avoid division by zero.
//    return 0;
//  }
  
  double deltaEncoderPos = currentEncoderPos - prevEncoderPos; // Change in encoder position
  
  // Convert deltaTime to seconds
  double deltaTimeSec = deltaTime / 1000.00;
  Serial.print("deltaTime : ");
  Serial.println(deltaTime);
  Serial.print("deltaTimeSec : ");
  Serial.println(deltaTimeSec);
  Serial.print("deltaEncoderPos : ");
  Serial.println(deltaEncoderPos);

  // Calculate speed (pulse/s)
  double speedInPulsePerSec = deltaEncoderPos / deltaTimeSec;


  // Convert speed to m/s using distancePerPulse
  //double speedInmPerSec = speedInPulsePerSec * distancePerPulse;
  
  // Update previous time and encoder position
  prevTime = currentTime;
  prevEncoderPos = currentEncoderPos;
  
  return speedInPulsePerSec;
}

