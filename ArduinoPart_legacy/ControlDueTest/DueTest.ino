int IN1 = 3;
int IN2 = 4;
 
void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
 
  // Motor A: IN1/2로 제어
  // IN1 전압 > IN2 전압 : 전류가 1>2로 흐름. 모터 회전
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  delay(2000);  
 
  // 반대 방향 회전
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  delay(2000);
 
  // 모터 멈춤. 둘다 LOW면 모터를 꺼둔, 풀린 상태라고 함.
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, HIGH);
}
 
void loop() {
  // put your main code here, to run repeatedly:
 
}
