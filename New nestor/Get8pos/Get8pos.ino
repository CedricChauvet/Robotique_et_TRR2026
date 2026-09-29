/*
回读所有舵机反馈参数:位置、速度、负载、电压、温度、移动状态；
FeedBack函数回读舵机参数于缓冲区，Readxxx(-1)函数返回缓冲区中相应的舵机状态；
函数Readxxx(ID)，ID=-1返回FeedBack缓冲区参数；ID>=0，通过读指令直接返回指定ID舵机状态,
无需调用FeedBack函数。
Read back all feedback parameters: position, speed, load, voltage, temperature, movement status;
The FeedBack function reads back the servo parameters in the buffer, and the Readxxx (-1) function returns the corresponding servo state in the buffer;
Function Readxxx (ID), ID=1 returns the FeedBack buffer parameter; ID > 0, and directly returns the specified ID rudder state by reading the instruction.
There is no need to call the FeedBack function.
*/

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#include <SCServo.h>

SCSCL sc;

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(115200);
  sc.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  int Pos;

  Pos = sc.ReadPos(1);
  if(Pos!=-1){
    Serial.print("Servo position1: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }

  Pos = sc.ReadPos(2);
  if(Pos!=-1){
    Serial.print("Servo position2: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }
  
  Pos = sc.ReadPos(3);
  if(Pos!=-1){
    Serial.print("Servo position3: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }
  
  
  Pos = sc.ReadPos(4);
  if(Pos!=-1){
    Serial.print("Servo position4: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }


  Pos = sc.ReadPos(5);
  if(Pos!=-1){
    Serial.print("Servo position5: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }

  Pos = sc.ReadPos(6);
  if(Pos!=-1){
    Serial.print("Servo position6: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }

  Pos = sc.ReadPos(7);
  if(Pos!=-1){
    Serial.print("Servo position7: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }

  Pos = sc.ReadPos(8);
  if(Pos!=-1){
    Serial.print("Servo position8: ");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }

 Serial.println("");
 
 delay (1000);




}
