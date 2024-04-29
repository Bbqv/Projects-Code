#include<MsTimer2.h>//定时器库的头文件
//----------------------------------定义管脚-----------------------------------
//电机编码器
#define ENCODER_A1 2 //电机 left
#define ENCODER_B1 5
#define ENCODER_A2 3 //电机 right
#define ENCODER_B2 4
//电机驱动的控制信号
#define PWM1 10 //ena
#define PWM2 9 //enb
#define INL1 12
#define INL2 11
#define INL3 8
#define INL4 7
#define L1 17 //左红外
#define L2 18
#define L3 19
#define R1 15 //右红外
#define R2 14
#define R3 13
#define M 16 //中红外

//-----------------------------------定义常值--------------------------------------
#define PERIOD 20
#define Kp 12.0
#define Ti 35.0
#define Td 25.0

//-----------------------------------全局变量--------------------------------------
float target1 = 15.0, t1=15.0; //左 保守
float target2 = 15.0, t2=15.0; //右 速度
volatile long encoderVal1;//编码器 1 值a
float velocity1; //转速 1
volatile long encoderVal2;//编码器 2 值b
float velocity2; //转速 2
float T = PERIOD;
float u1, ek11, ek12;
float u2, ek21, ek22;

//------------------------------------测速与转向------------------------------------
#define V 20.0 //基础速度10
void control(void)
{
  if (digitalRead(M) == HIGH)
  {
    target1 = V;
    target2 = V;
  }
  if (digitalRead(L1) == HIGH) //低右转
  {
    target1 = V ;
    target2 = V * 0.4;
  }
  if (digitalRead(R1) == HIGH) //低左转
  {
    target1 = V * 0.4;
    target2 = V ;
  }
  if (digitalRead(L2) == HIGH) //中右转
  {
    target1 = V * 0.9;
    target2 = V * 0.45;
  }
  if (digitalRead(R2) == HIGH) //中左转
  {
    target1 = V * 0.45;
    target2 = V * 0.9;
  }
  if (digitalRead(L3) == HIGH) //直角右转
  {
    target1 = V* 0.9;
    target2 = 0.3 * V;
  }
  if (digitalRead(R3) == HIGH) //直角左转
  {
    target1 = 0.3 * V;
    target2 = V* 0.9;
  }
  if (digitalRead(L3) == HIGH&&digitalRead(L2) == HIGH) //直角右转
  {
    target1 = 0.75 * V;
    target2 = 0;
  }
  if (digitalRead(R3) == HIGH&&digitalRead(R2) == HIGH) //直角左转
  {
    target1 = 0;
    target2 = 0.75 * V;
  }
  
  target1 = target1 * 0.55 + t1 * 0.45;
  target2 = target2 * 0.55 + t2 * 0.45;

  velocity1 = (encoderVal1 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal1 = 0;
  velocity2 = (encoderVal2 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal2 = 0;

  int output1 = pidController1(target1, velocity1);
  if (output1 > 0)
  {
    digitalWrite(INL2, HIGH);
    digitalWrite(INL1, LOW);
    analogWrite(PWM1, abs(output1));
  }
  else
  {
    digitalWrite(INL2, LOW);
    digitalWrite(INL1, HIGH);
    analogWrite(PWM1, abs(output1));
  }
  int output2 = pidController2(-target2, velocity2);
  if (output2 > 0)
  {
    digitalWrite(INL3, HIGH);
    digitalWrite(INL4, LOW);
    analogWrite(PWM2, abs(output2));
  }
  else
  {
    digitalWrite(INL3, LOW);
    digitalWrite(INL4, HIGH);
    analogWrite(PWM2, abs(output2));
  }
  t1 = target1;
  t2 = target2;
}

//--------------------------------------主函数-------------------------------------- -
void setup() {
  //9,10 两个管脚的 PWM 由定时器 TIMER1 产生，这句程序改变 PWM 的频率，勿删
  TCCR1B = TCCR1B & B11111000 | B00000001;
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(M, INPUT);
  attachInterrupt(0, getEncoder1, CHANGE);
  attachInterrupt(1, getEncoder2, CHANGE);
  Serial.begin(9600);
  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
}

void loop() {
  Serial.print(target1); Serial.print("\t");
  Serial.print(-target2);Serial.print("\t");
  Serial.print(velocity1);Serial.print("\t");
  Serial.println(velocity2);
  //Serial.print("\n");
  //Serial.println(u2);
  //Serial.println(u2);
}
//----------------------------------编码器中断函数------------------------------------ -
void getEncoder1(void)
{
  if (digitalRead(ENCODER_A1) == LOW)
  {
    if (digitalRead(ENCODER_B1) == LOW)
    {
      encoderVal1--;
    }
    else
    {
      encoderVal1++;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B1) == LOW)
    {
      encoderVal1++;
    }
    else
    {
      encoderVal1--;
    }
  }
}
void getEncoder2(void)
{
  if (digitalRead(ENCODER_A2) == LOW)
  {
    if (digitalRead(ENCODER_B2) == LOW)
    {
      encoderVal2--;
    }
    else
    {
      encoderVal2++;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B2) == LOW)
    {
      encoderVal2++;
    }
    else
    {
      encoderVal2--;
    }
  }
}
//-------------------------------------PID 控制器
int pidController1(float targetVelocity, float currentVelocity)
{
  float ek10;
  float q10 = Kp * (1 + T / Ti + Td / T);
  float q11 = -Kp * (1 + 2 * Td / T);
  float q12 = Kp * Td / T;
  ek10 = targetVelocity - currentVelocity;
  u1 = u1 + q10 * ek10 + q11 * ek11 + q12 * ek12;
  if (u1 > 255)
  {
    u1 = 255;
  }
  if (u1 < -255)
  {
    u1 = -255;
  }
  ek12 = ek11;
  ek11 = ek10;
  return (int)u1;
}
int pidController2(float targetVelocity, float currentVelocity)
{
  float ek20;
  float q20 = Kp * (1 + T / Ti + Td / T);
  float q21 = -Kp * (1 + 2 * Td / T);
  float q22 = Kp * Td / T;
  ek20 = targetVelocity - currentVelocity;
  u2 = u2 + q20 * ek20 + q21 * ek21 + q22 * ek22;
  if (u2 > 255)
  {
    u2 = 255;
  }
  if (u2 < -255)
  {
    u2 = -255;
  }
  ek22 = ek21;
  ek21 = ek20;
  return (int)u2;
}
