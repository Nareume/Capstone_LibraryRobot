// Hall effect encoder 핀 및 인터럽트를 초기화합니다.
const int hallA1 = 2; // Purple wire connected to digital pin 2
const int hallB1 = 3; // Blue wire connected to digital pin 3
const int hallA2 = 18; // Purple wire connected to digital pin 18
const int hallB2 = 19; // Blue wire connected to digital pin 19

// 모터 제어 핀을 정의합니다.
const int motorPwm1 = 5; // PWM signal to control motor A speed
const int motorDir1A = 6; // Direction A to control motor A rotation
const int motorDir1B = 7; // Direction B to control motor A rotation
const int motorPwm2 = 4; // PWM signal to control motor B speed
const int motorDir2A = 8; // Direction A to control motor B rotation
const int motorDir2B = 9; // Direction B to control motor B rotation

// 엔코더 카운트와 속도를 저장할 변수를 정의합니다.
volatile int count1 = 0; // Encoder count (incremented/decremented in ISR)
volatile int lastCount1 = 0; // Last encoder count for velocity calculation
volatile unsigned long lastTime1 = 0; // Last time for velocity calculation
volatile int count2 = 0; // Encoder count (incremented/decremented in ISR)
volatile int lastCount2 = 0; // Last encoder count for velocity calculation
volatile unsigned long lastTime2 = 0; // Last time for velocity calculation

// 인터럽트 서비스 루틴(ISR)을 정의합니다.
void hallInterrupt1()
{
  if (digitalRead(hallA1) == digitalRead(hallB1)) // 두 신호가 같은 경우
  {
    count1++; // 카운터를 증가시킵니다.
  }
  else // 두 신호가 다른 경우
  {
    count1--; // 카운터를 감소시킵니다.
  }
}

void hallInterrupt2()
{
  if (digitalRead(hallA2) == digitalRead(hallB2)) // 두 신호가 같은 경우
  {
    count2++; // 카운터를 증가시킵니다.
  }
  else // 두 신호가 다른 경우
  {
    count2--; // 카운터를 감소시킵니다.
  }
}

void setup()
{
  // 모터 제어 핀 및 PWM 신호를 초기화합니다.
  pinMode(motorPwm1, OUTPUT);
  pinMode(motorDir1A, OUTPUT);
  pinMode(motorDir1B, OUTPUT);
  pinMode(motorPwm2, OUTPUT);
  pinMode(motorDir2A, OUTPUT);
  pinMode(motorDir2B, OUTPUT);

  // Hall effect encoder 핀 및 인터럽트를 초기화합니다.
  pinMode(hallA1, INPUT_PULLUP);
  pinMode(hallB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallA1), hallInterrupt1, CHANGE);
  pinMode(hallA2, INPUT_PULLUP);
  pinMode(hallB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallA2), hallInterrupt2, CHANGE);

  // 디버그용으로 시리얼 통신을 초기화합니다.
  Serial.begin(9600);
}

void loop()
{
  // 엔코더 카운트 및 속도를 읽고 계산합니다.
  int encoderCount1 = count1;
  unsigned long currentTime1 = millis();
  float velocity1 = (encoderCount1 - lastCount1) / ((float)(currentTime1 - lastTime1) / 1000);

  int encoderCount2 = count2;
  unsigned long currentTime2 = millis();
  float velocity2 = (encoderCount2 - lastCount2) / ((float)(currentTime2 - lastTime2) / 1000);

  // 엔코더 카운트와 속도에 따라 모터를 제어합니다.
  int motorSpeed1 = 0;
  if (encoderCount1 < 0)
  {
    digitalWrite(motorDir1A, HIGH);
    digitalWrite(motorDir1B, LOW);
    motorSpeed1 = -encoderCount1;
  }
  else if (encoderCount1 > 0)
  {
    digitalWrite(motorDir1A, LOW);
    digitalWrite(motorDir1B, HIGH);
    motorSpeed1 = encoderCount1;
  }
  else
  {
    digitalWrite(motorDir1A, LOW);
    digitalWrite(motorDir1B, LOW);
  }
  analogWrite(motorPwm1, motorSpeed1);

  int motorSpeed2 = 0;
  if (encoderCount2 < 0)
  {
    digitalWrite(motorDir2A, HIGH);
    digitalWrite(motorDir2B, LOW);
    motorSpeed2 = -encoderCount2;
  }
  else if (encoderCount2 > 0)
  {
    digitalWrite(motorDir2A, LOW);
    digitalWrite(motorDir2B, HIGH);
    motorSpeed2 = encoderCount2;
  }
  else
  {
    digitalWrite(motorDir2A, LOW);
    digitalWrite(motorDir2B, LOW);
  }
  analogWrite(motorPwm2, motorSpeed2);

  // 엔코더 카운트와 속도를 출력합니다.
  Serial.print("Encoder count 1: ");
  Serial.print(encoderCount1);
  Serial.print(", Velocity 1: ");
  Serial.print(velocity1);
  Serial.print(" RPM, Encoder count 2: ");
  Serial.print(encoderCount2);
  Serial.print(", Velocity 2: ");
  Serial.print(velocity2);
  Serial.println(" RPM");

  // 엔코더 카운트 및 시간을 업데이트합니다.
  lastCount1 = encoderCount1;
 
