#define leftEncoderPinA 3
#define leftEncoderPinB 4
#define rightEncoderPinA 8
#define rightEncoderPinB 9
#define leftMotorPin1 5
#define leftMotorPin2 6
#define rightMotorPin1 10
#define rightMotorPin2 11
#define leftMotorEnable 13
#define rightMotorEnable 12

volatile int leftEncoderPos = 0;
volatile int rightEncoderPos = 0;

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
  pinMode(leftMotorEnable, OUTPUT);
  pinMode(rightMotorEnable, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), updateRightEncoder, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    int speed = 180; // Default speed
    if (command == 'S') {
      speed = Serial.parseInt(); // Read the next integers as speed
      if (speed < 0) speed = 0; // Ensure speed is not negative
      if (speed > 255) speed = 255; // Ensure speed does not exceed 255
      command = Serial.read(); // Read the next character as command
    }

    analogWrite(leftMotorEnable, speed);
    analogWrite(rightMotorEnable, speed);

    switch(command) {
      case 'F': // Forward
        moveMotor(leftMotorPin1, leftMotorPin2, HIGH, LOW);
        moveMotor(rightMotorPin1, rightMotorPin2, HIGH, LOW);
        delay(3000); 
        break;
      case 'B': // Backward
        moveMotor(leftMotorPin1, leftMotorPin2, LOW, HIGH);
        moveMotor(rightMotorPin1, rightMotorPin2, LOW, HIGH);
        delay(3000);
        break;
      case 'L': // Turn left
        moveMotor(rightMotorPin1, rightMotorPin2, HIGH, LOW);
        delay(3000);
        break;
      case 'R': // Turn right
        moveMotor(leftMotorPin1, leftMotorPin2, HIGH, LOW);
        delay(3000);
        break;
    }
    
    // Stop both motors
    moveMotor(leftMotorPin1, leftMotorPin2, LOW, LOW);
    moveMotor(rightMotorPin1, rightMotorPin2, LOW, LOW);
    delay(1000);
  }
}

void moveMotor(int motorPin1, int motorPin2, int pin1State, int pin2State) {
  digitalWrite(motorPin1, pin1State);
  digitalWrite(motorPin2, pin2State);
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
