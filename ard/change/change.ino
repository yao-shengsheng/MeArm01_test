#include <Servo.h>                //使用servo库
Servo base, fArm, rArm, claw ;    //创建4个servo对象
 
//存储电机极限值(const指定该数值为常量,常量数值在程序运行中不能改变)
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 0;
const int rArmMax = 180;
const int fArmMin = 0;
const int fArmMax = 180;
const int clawMin = 0;
const int clawMax = 60;
 
int DSD = 18; //Default Servo Delay (默认电机运动延迟时间)
              //此变量用于控制电机运行速度.增大此变量数值将
              //降低电机运行速度从而控制机械臂动作速度。
bool mode;  //mode = 1: 指令模式，  mode = 0: 手柄模式
int moveStep = 3;  // 每一次按下手柄按键，舵机移动量（仅适用于手柄模式）           
void setup(){
  base.attach(9);     // base 伺服舵机连接引脚9 舵机代号'b'
  delay(200);          // 稳定性等待
  rArm.attach(8);     // rArm 伺服舵机连接引脚8 舵机代号'r'(左侧)
  delay(200);          // 稳定性等待
  fArm.attach(7);      // fArm 伺服舵机连接引脚7  舵机代号'f'(右侧)
  delay(200);          // 稳定性等待
  claw.attach(6);      // claw 伺服舵机连接引脚6  舵机代号'c'
  delay(200);          // 稳定性等待
 
  base.write(90); 
  delay(10);
  fArm.write(90); 
  delay(10);
  rArm.write(90); 
  delay(10);
  claw.write(0);  
  delay(10); // 设置初始化
 
  Serial.begin(9600); 
  Serial.println("Welcome to Taichi-Maker Robot Arm Tutorial");   
}
 
void loop(){
  if (Serial.available()>0) {  
    char serialCmd = Serial.read();
    if( mode == 1 ){
      armDataCmd(serialCmd); //指令模式 对应一个函数 见53
    } else {
      armJoyCmd(serialCmd); //手柄模式 对应另一个函数
    }
  }
}
 
void armDataCmd(char serialCmd){ //Arduino根据串行指令执行相应操作
                                 //指令示例：b48 底盘转到48度角度位置
                                 //          o 输出机械臂舵机状态信息
  //判断人类是否因搞错模式而输入错误的指令信息（指令模式下输入手柄按键信息）
  if (   serialCmd == 'w' || serialCmd == 's' || serialCmd == 'a' || serialCmd == 'd' 
      || serialCmd == '8' || serialCmd == '4' || serialCmd == '6' || serialCmd == '8' ){
    Serial.println("+Warning: Robot in Instruction Mode..."); 
    delay(100);
    while(Serial.available()>0) char wrongCommand = Serial.read();  //清除串口缓存的错误指令
    return;
  } // 如果在mode = 0时输入手柄的消息 则不走这条函数
 
                                 
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r'){
    int servoData = Serial.parseInt();// 读取后面的数据
    servoCmd(serialCmd, servoData, DSD);  // 机械臂舵机运行函数（参数：舵机名，目标角度，延迟/速度）
  } else {
    switch(serialCmd){   
 
      case 'm' :   //切换至手柄模式 
        mode = 0; 
        Serial.println("Command: Switch to Joy-Stick Mode.");
        break;
        
      case 'o':  // 输出舵机状态信息
        reportStatus();
        break;
 
      case 'i':
        armIniPos();  
        break;
         
      default:  //未知指令反馈
        Serial.println("Unknown Command.");
    }
  }  
}
 
void armJoyCmd(char serialCmd){ //Arduino根据手柄按键执行相应操作
 
  //判断人类是否因搞错模式而输入错误的指令信息（手柄模式下输入舵机指令）
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r'){
    Serial.println("+Warning: Robot in Joy-Stick Mode...");
    delay(100);
    while(Serial.available()>0) char wrongCommand = Serial.read();  //清除串口缓存的错误指令
    return;
  } 
 
  
  int baseJoyPos;
  int rArmJoyPos;
  int fArmJoyPos;
  int clawJoyPos;
  switch(serialCmd){
    case 'a':  // Base向左
      Serial.println("Received Command: Base Turn Left");                
      baseJoyPos = base.read() - moveStep;// 向左移动moveStep个单位
      servoCmd('b', baseJoyPos, DSD);
      break;  
      
    case 'd':  // Base向右
      Serial.println("Received Command: Base Turn Right");                
      baseJoyPos = base.read() + moveStep;// 向右移动moveStep个单位
      servoCmd('b', baseJoyPos, DSD);
      break;        
 
    case 's':  // rArm向下
    Serial.println("Received Command: Rear Arm Down");                
      rArmJoyPos = rArm.read() + moveStep;// 同理
      servoCmd('r', rArmJoyPos, DSD);
      break;  
                 
    case 'w':  // rArm向上
      Serial.println("Received Command: Rear Arm Up");     
      rArmJoyPos = rArm.read() - moveStep;// 同理
      servoCmd('r', rArmJoyPos, DSD);
      break;  
 
    case '5':  // fArm向上
      Serial.println("Received Command: Front Arm Up");        
      fArmJoyPos = fArm.read() + moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;  
      
    case '2':  // fArm向下
      Serial.println("Received Command: Front Arm Down");        
      fArmJoyPos = fArm.read() - moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;  
      
    case '1':  // Claw关闭
      Serial.println("Received Command: Claw Close Down");        
      clawJoyPos = claw.read() + moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;  
      
    case '3':  // Claw打开
      Serial.println("Received Command: Claw Open Up");     
      clawJoyPos = claw.read() - moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;  
      
    case 'm' :   //切换至指令模式 
      mode = 1; 
      Serial.println("Command: Switch to Instruction Mode.");
      break;
 
    case 'o':  
      reportStatus();// 显示状态
      break;
 
    case 'i':  
      armIniPos();// 复原
      break;
      
    default:
      Serial.println("Unknown Command.");
      return;
      
  }  
}
void servoCmd(char servoName, int toPos, int servoDelay){  
  Servo servo2go;  //创建servo对象
 
  //串口监视器输出接收指令信息
  Serial.println("");
  Serial.print("+Command: Servo ");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");
  
  int fromPos; //建立变量，存储电机起始运动角度值
  
  switch(servoName){
    case 'b':
      if(toPos >= baseMin && toPos <= baseMax){
        servo2go = base;
        fromPos = base.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
        return;
      }
 
    case 'c':
      if(toPos >= clawMin && toPos <= clawMax){    
        servo2go = claw;
        fromPos = claw.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
        return;        
      }
 
    case 'f':
      if(toPos >= fArmMin && toPos <= fArmMax){
        servo2go = fArm;
        fromPos = fArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: fArm Servo Value Out Of Limit!");
        return;
      }
         
    case 'r':
      if(toPos >= rArmMin && toPos <= rArmMax){
        servo2go = rArm;
        fromPos = rArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
        return;
      }      
  }
 
  //指挥电机运行
  if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
    for (int i=fromPos; i<=toPos; i++){
      servo2go.write(i);
      delay (servoDelay);
    }
  }  else { //否则“起始角度值”大于“目标角度值”
    for (int i=fromPos; i>=toPos; i--){
      servo2go.write(i);
      delay (servoDelay);
    }
  }
}
 
void reportStatus(){  //舵机状态信息
  Serial.println("");
  Serial.println("");
  Serial.println("+ Robot-Arm Status Report +");
  Serial.print("Claw Position: "); Serial.println(claw.read());
  Serial.print("Base Position: "); Serial.println(base.read());
  Serial.print("Rear  Arm Position:"); Serial.println(rArm.read());
  Serial.print("Front Arm Position:"); Serial.println(fArm.read());
  Serial.println("++++++++++++++++++++++++++");
  Serial.println("");
}

// 复原函数
void armIniPos(){
  Serial.println("+Command: Restore Initial Position.");
  int robotIniPosArray[4][3] = {
    {'b', 90, DSD},
    {'r', 90, DSD},
    {'f', 90, DSD},
    {'c', 90, DSD} 
  };
 
  for (int i = 0; i < 4; i++){
    servoCmd(robotIniPosArray[i][0], robotIniPosArray[i][1], robotIniPosArray[i][2]);
  }
}
