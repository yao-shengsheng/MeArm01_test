#include <Servo.h>                //使用servo库
Servo base, fArm, rArm, claw ;    //创建4个servo对象
 
// 建立4个int型变量存储当前电机角度值
// 初始角度值为设备启动后初始状态所需要的电机角度数值
int basePos = 90;
int rArmPos = 90;
int fArmPos = 60;
int clawPos = 60;
 
//存储电机极限值(const指定该数值为常量,常量数值在程序运行中不能改变)
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 20;
const int rArmMax = 120;
const int fArmMin = 0;
const int fArmMax = 180;
const int clawMin = 0;
const int clawMax = 70;
 
void setup(){
  base.attach(9);     // base 伺服舵机连接引脚11 舵机代号'b'
  delay(200);          // 稳定性等待
  rArm.attach(8);     // rArm 伺服舵机连接引脚10 舵机代号'r'
  delay(200);          // 稳定性等待
  fArm.attach(7);      // fArm 伺服舵机连接引脚9  舵机代号'f'
  delay(200);          // 稳定性等待
  claw.attach(6);      // claw 伺服舵机连接引脚6  舵机代号'c'
  delay(200);          // 稳定性等待
  Serial.begin(9600); 
  Serial.println("Welcome to Taichi-Maker Robot Arm Tutorial");   
}
 
void loop(){
  if (Serial.available()>0) {  
    char serialCmd = Serial.read();
    armDataCmd(serialCmd);
  }
 
  base.write(basePos); 
  delay(10);
  fArm.write(fArmPos); 
  delay(10);
  rArm.write(rArmPos); 
  delay(10);
  claw.write(clawPos);  
  delay(10);   
}
 
void armDataCmd(char serialCmd){
  Serial.print("serialCmd = ");
  Serial.print(serialCmd);  
  
  int servoData = Serial.parseInt(); 
 
  int fromPos;
  int toPos;
  
  switch(serialCmd){
    case 'b':  
      fromPos = base.read();
      toPos = servoData;
      
      if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
        for (int i=fromPos; i<=toPos; i++){
          base.write(i);
          delay (15);
        }
      } else {  //否则“起始角度值”大于“目标角度值”
        for (int i=fromPos; i>=toPos; i--){
          base.write(i);
          delay (15);
        }
      }
      
      basePos = servoData;
      Serial.print("  Set base servo value: ");
      Serial.println(servoData);
      break;
    case 'c':  
      clawPos = servoData;
      Serial.print("  Set claw servo value: ");
      Serial.println(servoData);
      break;
    case 'f':  
      fArmPos = servoData;
      Serial.print("  Set fArm servo value: ");
      Serial.println(servoData);
      break;
    case 'r':  
      rArmPos = servoData;
      Serial.print("  Set rArm servo value: ");
      Serial.println(servoData);
      break;
    case 'o':  
      reportStatus();
      break;
    default:
      Serial.println(" Unknown Command.");
  }  
}
 
void reportStatus(){
  Serial.println("");
  Serial.println("");
  Serial.println("++++++ Robot-Arm Status Report +++++");
  Serial.print("Claw Position: clawPos = "); Serial.println(claw.read());
  Serial.print("Base Position: basePos = "); Serial.println(base.read());
  Serial.print("Rear  Arm Position: rArmPos = "); Serial.println(rArm.read());
  Serial.print("Front Arm Position: fArmPos = "); Serial.println(fArm.read());
  Serial.println("++++++++++++++++++++++++++++++++++++");
  Serial.println("");
}
