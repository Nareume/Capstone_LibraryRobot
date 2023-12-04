/*
바퀴 한바퀴 회전, 4체배, 90도 엔코더 Setting 확인
*/
#include <Arduino.h>

class WheelEncoder {
private:
  int encoderPinA;
  int encoderPinB;
  volatile long encoderCount;

public:
  // Constructor
  WheelEncoder(int pinA, int pinB) : encoderPinA(pinA), encoderPinB(pinB), encoderCount(0) {}

  // Initialize encoder pins and interrupts
  void init() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  }

  // Update encoder count based on A and B channel states
  void updateEncoder() {
    if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  // Get the current encoder count
  long getCount() const {
    return encoderCount;
  }
  
  // Reset encoder count
  void resetCount() {
    encoderCount = 0;
  }

private:
  // Static method to be used as an ISR for the interrupt
  static void updateEncoder() {
    // This is a static method, but it has access to the WheelEncoder's member variables
    encoder.updateEncoder();
  }
};

// Create instances for left and right wheel encoders
WheelEncoder leftWheelEncoder(leftEncoderPinA, leftEncoderPinB);
WheelEncoder rightWheelEncoder(rightEncoderPinA, rightEncoderPinB);

void setup() {
  // Initialize wheel encoders
  leftWheelEncoder.init();
  rightWheelEncoder.init();

  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Perform logic using left and right wheel encoder counts
  Serial.print("Left Encoder Count: ");
  Serial.println(leftWheelEncoder.getCount());
  Serial.print("Right Encoder Count: ");
  Serial.println(rightWheelEncoder.getCount());

  delay(1000); // Wait for 1 second
}

  delay(1000); // 1초 동안 대기
}
