/* ============================================================================
 *  MeArm 机械臂 —— 摇杆板(ArmJoyStick)遥控版
 *  在太极创客《meArm 篇》change.ino 的基础上扩展而来
 *
 *  新增/改造内容：
 *    1. 保留原有「指令模式」：串口输入 b90 / r45 / f60 / c30 / o / i / m
 *    2. 改造「手柄模式」：不再只等串口送来的 wsad 字符，
 *       而是直接读取摇杆板上的双摇杆(A0~A3) + 两个按键，
 *       实时、连续、带限位地驱动 4 个舵机
 *    3. 新增 'j' 查看摇杆原始值、'k' 重新标定摇杆中位（调机用）
 *
 *  三个关键细节（摇杆控制能"手感对"的原因）：
 *    ① 中位标定：上电时读一次中位，避免摇杆本身有偏差导致一直漂
 *    ② 死区 DEADZONE：摇杆松手不会精确回到中位，没死区机械臂会慢慢爬
 *    ③ 比例调速 + 限位：推杆幅度 → 每周期角度增量，推得越多动得越快；
 *       同时硬性夹在 [min, max] 之间，保护舵机和机械结构
 *
 *  注意：摇杆控制必须"非阻塞"。原 servoCmd() 用 for + delay() 一点点转，
 *        在摇杆模式下会把手感拖成卡顿，所以这里用 millis() 定时 + 直接增量写角度。
 * ==========================================================================*/

#include <Servo.h>                 // 使用 servo 库

Servo base, fArm, rArm, claw;      // 4 个舵机对象

/* ======================= 1. 舵机引脚（★按实际接线修改） =================== */
const int pinBase = 9;             // base 底盘    舵机代号 'b'
const int pinFArm = 7;             // fArm 前臂    舵机代号 'f'
const int pinRArm = 8;             // rArm 后臂    舵机代号 'r'
const int pinClaw = 6;             // claw 夹爪    舵机代号 'c'

/* ======================= 2. 摇杆板引脚（★按实际接线修改） ================= */
/*  常见的 meArm 摇杆扩展板：左摇杆 HOR=A0 / VER=A1，右摇杆 HOR=A2 / VER=A3
 *  按键 A = D2，按键 B = D4，状态灯 = D3
 *  若你的板子丝印不同，只改这几行即可。                                */
const int joyL_X = A0;             // 左摇杆 水平 → 底盘 base
const int joyL_Y = A1;             // 左摇杆 垂直 → 后臂 rArm
const int joyR_X = A2;             // 右摇杆 水平 → 前臂 fArm
const int joyR_Y = A3;             // 右摇杆 垂直 → 夹爪 claw
const int pinBtnA = 2;             // 按键 A
const int pinBtnB = 4;             // 按键 B

/* ======================= 3. 摇杆方向修正 ================================= */
/*  推杆方向和你想要的舵机转向不一致时，把对应的 1 改成 -1 即可            */
const int DIR_L_X = 1;             // 左摇杆左右
const int DIR_L_Y = 1;             // 左摇杆上下
const int DIR_R_X = 1;             // 右摇杆左右
const int DIR_R_Y = 1;             // 右摇杆上下

/* ======================= 4. 舵机行程极限 ================================= */
/*  请按你的机械结构实际能转到的范围收紧，别让舵机顶死              */
const int baseMin = 0,   baseMax = 180;
const int rArmMin = 0,   rArmMax = 180;
const int fArmMin = 0,   fArmMax = 180;
const int clawMin = 0,   clawMax = 180;

/* ======================= 5. 运行参数 ===================================== */
int DSD = 18;                      // 指令模式下舵机运动速度(ms/度)
const bool MODE_CMD = true;        // mode = true : 指令模式
const bool MODE_JOY = false;       // mode = false: 摇杆模式
bool mode = MODE_JOY;              // 上电默认进入摇杆模式
int moveStep = 3;                  // 收到 wsad/1235 字符时每次移动的角度

const int DEADZONE = 60;           // 摇杆死区（±60/1023，约 ±6%）
const int MAX_STEP = 4;            // 每 20ms 最多转多少度 → 最高速度 200°/s
const int UPDATE_MS = 20;          // 摇杆刷新周期(ms)
const int AXIS_SAMPLES = 4;        // 模拟量多次采样取平均，抗抖动

/* ======================= 6. 摇杆中位（开机自动标定） ===================== */
int cLX, cLY, cRX, cRY;

unsigned long lastJoyMs = 0;       // 摇杆刷新计时

/* ======================= 7. 函数原型 ===================================== */
/*  Arduino IDE 会自动生成，这里显式写出便于阅读，也方便用命令行工具链编译 */
void armDataCmd(char serialCmd);
void armJoyCmd(char serialCmd);
void servoCmd(char servoName, int toPos, int servoDelay);
void reportStatus();
void armIniPos();
int  readAxisAvg(int pin);
void calibrateJoystick();
int  axisToStep(int joyPin, int center, int dir);
void nudgeServo(Servo &s, int step, int minA, int maxA);
void updateJoystick();
void updateButtons();
void reportJoyStick();

/* ========================================================================== */
void setup() {
  base.attach(pinBase);  delay(200);   // 稳定性等待
  rArm.attach(pinRArm);  delay(200);
  fArm.attach(pinFArm);  delay(200);
  claw.attach(pinClaw);  delay(200);

  base.write(90); delay(10);           // 初始化到中位
  fArm.write(90); delay(10);
  rArm.write(90); delay(10);
  claw.write(90); delay(10);

  pinMode(pinBtnA, INPUT_PULLUP);      // 摇杆板按键：内部上拉，按下为 LOW
  pinMode(pinBtnB, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Welcome to Taichi-Maker Robot Arm Tutorial");
  Serial.println("Mode: JOY-STICK (摇杆板)  --  long press Btn A to switch mode");

  delay(300);                          // 给摇杆一点回中时间
  calibrateJoystick();                 // ★上电标定中位，此时不要去碰摇杆
}

/* ========================================================================== */
void loop() {
  /* ---- 串口：指令模式 / 蓝牙 App 单字符，原功能完整保留 ---- */
  while (Serial.available() > 0) {
    char serialCmd = Serial.read();
    if (mode == MODE_CMD) {
      armDataCmd(serialCmd);
    } else {
      armJoyCmd(serialCmd);
    }
  }

  /* ---- 摇杆：只有摇杆模式才实时读取 ---- */
  if (mode == MODE_JOY) {
    updateJoystick();
    updateButtons();
  }
}

/* ==========================================================================
 *  摇杆读取相关
 * ==========================================================================*/
int readAxisAvg(int pin) {                 // 多次采样取平均，抑制抖动
  long sum = 0;
  for (int i = 0; i < AXIS_SAMPLES; i++) sum += analogRead(pin);
  return (int)(sum / AXIS_SAMPLES);
}

void calibrateJoystick() {                 // 标定中位：以"静止时的读数"为中位
  cLX = readAxisAvg(joyL_X);
  cLY = readAxisAvg(joyL_Y);
  cRX = readAxisAvg(joyR_X);
  cRY = readAxisAvg(joyR_Y);
  Serial.print("+JoyStick center: ");
  Serial.print(cLX); Serial.print(", ");
  Serial.print(cLY); Serial.print(", ");
  Serial.print(cRX); Serial.print(", ");
  Serial.println(cRY);
}

/*  把一路模拟量换算成"本周期该转多少度"
 *  返回值范围 -MAX_STEP ~ +MAX_STEP，0 表示在死区内不动            */
int axisToStep(int joyPin, int center, int dir) {
  int d = readAxisAvg(joyPin) - center;
  if (abs(d) <= DEADZONE) return 0;                 // 死区 → 不动

  // 死区之外还剩多少行程，用于把"推杆幅度"线性映射成"速度"
  int span = (d > 0) ? (1023 - center - DEADZONE)
                     : (center - DEADZONE);
  if (span < 1) span = 1;

  int mag  = abs(d) - DEADZONE;                     // 0 .. span
  int step = (int)((long)mag * MAX_STEP / span);    // 0 .. MAX_STEP
  if (step < 1) step = 1;                           // 一出死区至少动 1 度
  if (step > MAX_STEP) step = MAX_STEP;

  return (d > 0 ? step : -step) * dir;
}

/*  按增量移动舵机，并夹在行程极限内    */
void nudgeServo(Servo &s, int step, int minA, int maxA) {
  if (step == 0) return;
  int cur = s.read();                 // Servo.read() 返回上次写入的角度
  int to  = cur + step;
  if (to < minA) to = minA;
  if (to > maxA) to = maxA;
  if (to != cur) s.write(to);
}

/*  摇杆主循环：定时刷新，非阻塞    */
void updateJoystick() {
  if (millis() - lastJoyMs < UPDATE_MS) return;
  lastJoyMs = millis();

  nudgeServo(base, axisToStep(joyL_X, cLX, DIR_L_X), baseMin, baseMax);
  nudgeServo(rArm, axisToStep(joyL_Y, cLY, DIR_L_Y), rArmMin, rArmMax);
  nudgeServo(fArm, axisToStep(joyR_X, cRX, DIR_R_X), fArmMin, fArmMax);
  nudgeServo(claw, axisToStep(joyR_Y, cRY, DIR_R_Y), clawMin, clawMax);
}

/*  摇杆板两个按键：短按 A 回初始位置，长按 A 切换模式，按 B 输出状态 */
void updateButtons() {
  static bool lastA = HIGH, lastB = HIGH;
  static unsigned long aDownMs = 0;
  static bool aLongHandled = false;

  bool a = digitalRead(pinBtnA);
  bool b = digitalRead(pinBtnB);

  /* ---- 按键 A ---- */
  if (a == LOW && lastA == HIGH) {           // 按下瞬间
    aDownMs = millis();
    aLongHandled = false;
  }
  if (a == LOW && !aLongHandled && (millis() - aDownMs >= 800)) {
    aLongHandled = true;                     // 长按 → 切换模式
    mode = MODE_CMD;
    Serial.println("Command: Switch to Instruction Mode.");
  }
  if (a == HIGH && lastA == LOW) {           // 抬起瞬间
    if (!aLongHandled) armIniPos();          // 短按 → 回初始位置
  }
  lastA = a;

  /* ---- 按键 B ---- */
  if (b == LOW && lastB == HIGH) reportStatus();
  lastB = b;
}

/*  摇杆原始值打印，接线/调机时很有用（指令模式下输入 j）   */
void reportJoyStick() {
  Serial.println("");
  Serial.println("+ Joy-Stick Raw Values +");
  Serial.print("L_X(A0): "); Serial.println(readAxisAvg(joyL_X));
  Serial.print("L_Y(A1): "); Serial.println(readAxisAvg(joyL_Y));
  Serial.print("R_X(A2): "); Serial.println(readAxisAvg(joyR_X));
  Serial.print("R_Y(A3): "); Serial.println(readAxisAvg(joyR_Y));
  Serial.print("Center  : ");
  Serial.print(cLX); Serial.print(", ");
  Serial.print(cLY); Serial.print(", ");
  Serial.print(cRX); Serial.print(", ");
  Serial.println(cRY);
  Serial.println("++++++++++++++++++++++++");
  Serial.println("");
}

/* ==========================================================================
 *  以下为原 change.ino 的指令模式 / 通用函数（仅做了小修）
 * ==========================================================================*/
void armDataCmd(char serialCmd) {
  /* 判断是否误用手柄按键（原代码这里写成了 '8','4','6','8'，属笔误，
     已按 armJoyCmd 的实际按键修正为 w/s/a/d/1/2/3/5）            */
  if (   serialCmd == 'w' || serialCmd == 's' || serialCmd == 'a' || serialCmd == 'd'
      || serialCmd == '1' || serialCmd == '2' || serialCmd == '3' || serialCmd == '5') {
    Serial.println("+Warning: Robot in Instruction Mode...");
    delay(100);
    while (Serial.available() > 0) Serial.read();     // 清空缓存里的错误指令
    return;
  }

  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r') {
    int servoData = Serial.parseInt();                // 读取后面的数字
    servoCmd(serialCmd, servoData, DSD);
  } else {
    switch (serialCmd) {
      case 'm':                                       // 切换至摇杆模式
        mode = MODE_JOY;
        Serial.println("Command: Switch to Joy-Stick Mode.");
        break;

      case 'o':                                       // 输出舵机状态
        reportStatus();
        break;

      case 'i':                                       // 回初始位置
        armIniPos();
        break;

      case 'j':                                       // ★新增：查看摇杆原始值
        reportJoyStick();
        break;

      case 'k':                                       // ★新增：重新标定摇杆中位
        Serial.println("+Recalibrate: keep the sticks at rest...");
        delay(300);
        calibrateJoystick();
        break;

      default:
        Serial.println("Unknown Command.");
    }
  }
}

/*  手柄模式：兼容蓝牙 App / 串口发来的 wsad、1235 单字符（原功能保留）  */
void armJoyCmd(char serialCmd) {
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r') {
    Serial.println("+Warning: Robot in Joy-Stick Mode...");
    delay(100);
    while (Serial.available() > 0) Serial.read();
    return;
  }

  int baseJoyPos, rArmJoyPos, fArmJoyPos, clawJoyPos;

  switch (serialCmd) {
    case 'a':                                   // Base 向左
      Serial.println("Received Command: Base Turn Left");
      baseJoyPos = base.read() - moveStep;
      servoCmd('b', baseJoyPos, DSD);
      break;

    case 'd':                                   // Base 向右
      Serial.println("Received Command: Base Turn Right");
      baseJoyPos = base.read() + moveStep;
      servoCmd('b', baseJoyPos, DSD);
      break;

    case 's':                                   // rArm 向下
      Serial.println("Received Command: Rear Arm Down");
      rArmJoyPos = rArm.read() + moveStep;
      servoCmd('r', rArmJoyPos, DSD);
      break;

    case 'w':                                   // rArm 向上
      Serial.println("Received Command: Rear Arm Up");
      rArmJoyPos = rArm.read() - moveStep;
      servoCmd('r', rArmJoyPos, DSD);
      break;

    case '5':                                   // fArm 向上
      Serial.println("Received Command: Front Arm Up");
      fArmJoyPos = fArm.read() + moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;

    case '2':                                   // fArm 向下
      Serial.println("Received Command: Front Arm Down");
      fArmJoyPos = fArm.read() - moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;

    case '1':                                   // Claw 关闭
      Serial.println("Received Command: Claw Close Down");
      clawJoyPos = claw.read() + moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;

    case '3':                                   // Claw 打开
      Serial.println("Received Command: Claw Open Up");
      clawJoyPos = claw.read() - moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;

    case 'm':                                   // 切换至指令模式
      mode = MODE_CMD;
      Serial.println("Command: Switch to Instruction Mode.");
      break;

    case 'o':
      reportStatus();
      break;

    case 'i':
      armIniPos();
      break;

    case 'j':                                   // ★新增：查看摇杆原始值
      reportJoyStick();
      break;

    case 'k':                                   // ★新增：重新标定摇杆中位
      calibrateJoystick();
      break;

    default:
      Serial.println("Unknown Command.");
      return;
  }
}

/*  舵机平滑运动（原样保留，用于指令模式 / 回原点）  */
void servoCmd(char servoName, int toPos, int servoDelay) {
  Servo servo2go;

  Serial.println("");
  Serial.print("+Command: Servo ");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");

  int fromPos;

  switch (servoName) {
    case 'b':
      if (toPos >= baseMin && toPos <= baseMax) {
        servo2go = base;
        fromPos = base.read();
        break;
      } else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
        return;
      }

    case 'c':
      if (toPos >= clawMin && toPos <= clawMax) {
        servo2go = claw;
        fromPos = claw.read();
        break;
      } else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
        return;
      }

    case 'f':
      if (toPos >= fArmMin && toPos <= fArmMax) {
        servo2go = fArm;
        fromPos = fArm.read();
        break;
      } else {
        Serial.println("+Warning: fArm Servo Value Out Of Limit!");
        return;
      }

    case 'r':
      if (toPos >= rArmMin && toPos <= rArmMax) {
        servo2go = rArm;
        fromPos = rArm.read();
        break;
      } else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
        return;
      }
  }

  if (fromPos <= toPos) {
    for (int i = fromPos; i <= toPos; i++) {
      servo2go.write(i);
      delay(servoDelay);
    }
  } else {
    for (int i = fromPos; i >= toPos; i--) {
      servo2go.write(i);
      delay(servoDelay);
    }
  }
}

/*  舵机状态信息  */
void reportStatus() {
  Serial.println("");
  Serial.println("");
  Serial.println("+ Robot-Arm Status Report +");
  Serial.print("Claw Position: ");      Serial.println(claw.read());
  Serial.print("Base Position: ");      Serial.println(base.read());
  Serial.print("Rear  Arm Position:");  Serial.println(rArm.read());
  Serial.print("Front Arm Position:");  Serial.println(fArm.read());
  Serial.print("Mode: ");               Serial.println(mode == MODE_CMD ? "Instruction" : "Joy-Stick");
  Serial.println("++++++++++++++++++++++++++");
  Serial.println("");
}

/*  复原函数  */
void armIniPos() {
  Serial.println("+Command: Restore Initial Position.");
  int robotIniPosArray[4][3] = {
    {'b', 90, DSD},
    {'r', 90, DSD},
    {'f', 90, DSD},
    {'c', 90, DSD}
  };

  for (int i = 0; i < 4; i++) {
    servoCmd(robotIniPosArray[i][0], robotIniPosArray[i][1], robotIniPosArray[i][2]);
  }
}
