
#include <U8g2lib.h>

// Define stepper motor connections and steps per revolution:
#define dirPin 14 //
#define stepPin 27 //
#define clockwiseDir LOW  //TODO
#define anticlockwiseDir HIGH  //TODO
#define stepsPerRevolution 1000
#define stepsSize 1000 //每次按键改变幅度
#define targetOffsetStepMax 40000//最高上升
#define targetOffsetStepMin 5000//最低下降
#define stepPeriod 100//制做交流电频率
#define resetStepPeriod 100//制做交流电频率//重复了

#define powerPin 23

#define springPositionSteps 39000 //252000//上拉发射位置参数
#define initialTargetOffsetSteps 20000 //157500//默认预设下拉值

char strLEDCharBuffer[32];

#define buttonPinA 15
#define buttonPinB 2
#define buttonPinC 4
#define buttonPinD 5

#define clipPinA 12
#define clipPinB 13

int prevButtonA = 1;
int prevButtonB = 1;
int prevButtonC = 1;

int targetOffsetSteps = initialTargetOffsetSteps;//157500 //可变预设下拉值

void reset_stepper();
void moveTo(int steps);
void openClip();


U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

void setup() {

  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_helvR08_te);  // choose a suitable font
  delay(1000);


  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(clipPinA, OUTPUT);
  pinMode(clipPinB, OUTPUT);

  digitalWrite(clipPinA, LOW);
  digitalWrite(clipPinB, LOW);

  pinMode(buttonPinA, INPUT);
  pinMode(buttonPinB, INPUT);
  pinMode(buttonPinC, INPUT);
  pinMode(buttonPinD, INPUT);

  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, 1);


  Serial.begin(115200);
  Serial.println("Ready!");

  delay(10);
  openClip();//测试//放开夹子
  delay(500);
  reset_stepper();//初始化下拉复位，以及获得发射箭
  //动作结束，默认为上拉方向

  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setCursor(20, 20);
  u8g2.print("IVE TY");  // write something to the internal memory

  u8g2.sendBuffer();
  delay(100); //delay reserved

}

void loop() {
  delay(10);

  int buttonStateA = digitalRead(buttonPinA);
  int buttonStateB = digitalRead(buttonPinB);
  int buttonStateC = digitalRead(buttonPinC);

  if (buttonStateA == 0 && buttonStateA != prevButtonA) { //按下a按钮，进入发射流程
    Serial.println("load arrow....");

    // close clip
    Serial.println("Close clip");
    closeClip();//关紧夹子
    //    delay(100);

    digitalWrite(dirPin, clockwiseDir); //clockwise://设置为下拉方向
    moveTo(targetOffsetSteps);//
    delay(500);

    // open clip
    Serial.println("Open clip");
    openClip();//放开夹子，发射
    //    delay(100);

    //return to spring
    digitalWrite(dirPin, anticlockwiseDir); //anticlockwise://设置为上拉方向
    moveTo(targetOffsetSteps);//返回箭台位置
    digitalWrite(dirPin, clockwiseDir); //clockwise://设置为下拉方向

  }
  else if (buttonStateB == 0 && buttonStateB != prevButtonB) { //上拉
    if ((targetOffsetSteps + stepsSize) < targetOffsetStepMax)
    {
      targetOffsetSteps += stepsSize;
      sprintf(strLEDCharBuffer, "Offset: %d", targetOffsetSteps);
      Serial.println(targetOffsetSteps);
      u8g2.clearBuffer();
      u8g2.drawStr(0, 20, strLEDCharBuffer);
      u8g2.sendBuffer();

    }
  }
  else if (buttonStateC == 0 && buttonStateC != prevButtonC) { //下降
    if ((targetOffsetSteps - stepsSize) > targetOffsetStepMin)
    {
      targetOffsetSteps -= stepsSize;
      sprintf(strLEDCharBuffer, "Offset: %d", targetOffsetSteps);

      Serial.println(targetOffsetSteps);
      u8g2.clearBuffer();
      u8g2.drawStr(0, 20, strLEDCharBuffer);
      u8g2.sendBuffer();


    }
  }

  Serial.println(targetOffsetSteps);
  //预防重复操作
  prevButtonA = buttonStateA;
  prevButtonB = buttonStateB;
  prevButtonC = buttonStateC;

  return;

}

void reset_stepper() { //初始化下拉复位，以及获得发射箭
  Serial.println("Reset....");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0, 20, "Reset....");
  u8g2.sendBuffer();

  digitalWrite(dirPin, clockwiseDir); //clockwise://设置为下拉方向//电机的顺时针
  //进行复位程序
  while (digitalRead(buttonPinD) != 0) { //当没有碰到底座按钮
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(resetStepPeriod);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(resetStepPeriod);
    //交流电
  }
  delay(500);
  //进行上拉，获得箭
  digitalWrite(dirPin, anticlockwiseDir); //anticlockwise//设置为上拉方向
  moveTo(springPositionSteps);//到达箭台位置


  Serial.println("Ready.");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0, 20, "Ready.");
  u8g2.sendBuffer();

  delay(100);
}

void moveTo(int steps) {
  Serial.print("Moving to ");
  Serial.print(steps);
  Serial.println("...");
  for (int i = 0; i < steps; i++) {//当没有到达步骤时间时
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepPeriod);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepPeriod);
    //交流电
  }
  Serial.println("Movement completed.");

  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, "Movement");
  u8g2.drawStr(0, 50, "completed.");
  u8g2.sendBuffer();


}

void openClip() { //放开夹子
  digitalWrite(clipPinA, HIGH); //clockwise:
  delay(100);
  digitalWrite(clipPinA, LOW); //clockwise:
  delay(100);
}

void closeClip() { //关紧夹子
  digitalWrite(clipPinB, HIGH); //clockwise:
  delay(100);
  digitalWrite(clipPinB, LOW); //clockwise:
  delay(100);
}
