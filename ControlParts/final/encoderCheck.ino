'''
바퀴 한바퀴 회전, 4체배, 90도 엔코더 Setting 확인
'''

#define leftEncoderPinA 3
#define leftEncoderPinB 4
#define rightEncoderPinA 8
#define rightEncoderPinB 9

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void setup() {
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);

  Serial.begin(115200);
}

void updateLeftEncoder() {
  // A 채널 상태가 변경될 때마다 B 채널 상태 확인
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void updateRightEncoder() {
  // A 채널 상태가 변경될 때마다 B 채널 상태 확인
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void loop() {
  // 여기에서 leftEncoderCount와 rightEncoderCount를 사용하여 로직을 수행
  Serial.print("Left Encoder Count: ");
  Serial.println(leftEncoderCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightEncoderCount);

  delay(1000); // 1초 동안 대기
}
