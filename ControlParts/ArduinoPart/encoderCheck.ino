#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

int encoder0PinA = 3;
int encoder0PinB = 4;
volatile int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;

int encoder1PinA = 9;
int encoder1PinB = 8;
volatile int encoder1Pos = 0;
int encoder1PinALast = LOW;
int m = LOW;



void setup() { 
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  Wire.begin();  // SDA = 25, SCL = 26으로 설정
  lcd.init();
  lcd.backlight();
  lcd.clear();
} 

void loop() { 
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
    lcd.setCursor(0, 0);  // 첫 번째 행
    lcd.print("Encoder 0: ");
    lcd.print(encoder0Pos);
  } 
  encoder0PinALast = n;

  m = digitalRead(encoder1PinA);
  if ((encoder1PinALast == LOW) && (m == HIGH)) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++;
    }
    lcd.setCursor(0, 1);  // 두 번째 행
    lcd.print("Encoder 1: ");
    lcd.print(encoder1Pos);
  } 
  encoder1PinALast = m;
}
