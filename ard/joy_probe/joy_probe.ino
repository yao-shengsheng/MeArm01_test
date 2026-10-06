/* ============================================================================
 *  摇杆板引脚探测工具 —— joy_probe.ino
 *
 *  什么时候用：
 *    · 摇杆板丝印看不清，不确定哪个摇杆接哪个模拟口 (A0~A5)
 *    · 不确定按键接在哪个数字口 (D2~D13)
 *    · 不确定摇杆方向（推左时读数变大还是变小）
 *
 *  安全说明：
 *    ★ 本程序全程不 attach 任何舵机 → 机械臂完全不会动，可以放心跑。
 *
 *  用法：
 *    1. Arduino IDE 打开本文件 → 上传（板子选 Arduino Uno / Nano）
 *    2. 打开串口监视器，波特率 9600
 *    3. 把两个摇杆分别推到 上/下/左/右 各到底，再松手
 *    4. 看串口输出：哪一路 A 口数值在变，就是哪一路摇杆轴
 *    5. 顺便按一按摇杆板上的按键，看哪个 D 口变成 LOW
 *
 *  注意：Uno 只有 A0~A5；Nano 还有 A6、A7（只能当模拟输入，不能当数字口）。
 *        如果你用的是 Nano 且想一起扫，把下面 AXIS_PINS 后补上 A6, A7 即可。
 * ==========================================================================*/

/* 参与扫描的模拟口 */
const int AXIS_PINS[] = { A0, A1, A2, A3, A4, A5 };
const int AXIS_COUNT  = sizeof(AXIS_PINS) / sizeof(AXIS_PINS[0]);

const int BTN_FIRST = 2;      // 按键扫描范围 D2 ~ D13
const int BTN_LAST  = 13;

const unsigned long PRINT_MS = 300;    // 实时数值打印间隔
const unsigned long STAT_MS  = 3000;   // 统计窗口：多久结一次论
const int ACTIVE_DELTA       = 60;     // 偏离基线多少算"动了" (0~1023)
const int ACTIVE_RANGE       = 100;    // 窗口内变化幅度超过多少 → 判为一路摇杆轴

int  axisBase[AXIS_COUNT];             // 上电基线（近似中位）
int  axisMin[AXIS_COUNT];
int  axisMax[AXIS_COUNT];

unsigned long lastPrintMs = 0;
unsigned long lastStatMs  = 0;

/* ---------------------------- 函数原型 ---------------------------- */
int  readAxis(int pin);
void printLive();
void printReport();
void resetStats();

/* ---------------------------- setup ---------------------------- */
void setup() {
  Serial.begin(9600);
  delay(300);

  Serial.println();
  Serial.println("=== JoyStick Pin Probe ===");
  Serial.println("1) 请把两个摇杆分别推到 上/下/左/右 各到底，再松手");
  Serial.println("2) 顺便按一按摇杆板上的按键");
  Serial.println("3) 看下面的输出，就知道哪个口接的是哪一路");
  Serial.println();

  for (int i = 0; i < AXIS_COUNT; i++) {
    pinMode(AXIS_PINS[i], INPUT);
    axisBase[i] = readAxis(AXIS_PINS[i]);   // 上电基线 = 中位参考
    axisMin[i]  = axisBase[i];
    axisMax[i]  = axisBase[i];
  }

  for (int p = BTN_FIRST; p <= BTN_LAST; p++) pinMode(p, INPUT_PULLUP);

  Serial.print("上电基线: ");
  for (int i = 0; i < AXIS_COUNT; i++) {
    Serial.print("A");
    Serial.print(i);
    Serial.print("=");
    Serial.print(axisBase[i]);
    Serial.print("  ");
  }
  Serial.println();
  Serial.println("(基线应该在 480~540 附近，如果某一路是 0 或 1023，说明那条线没接好)");
  Serial.println();
  Serial.println("实时数值:");
}

/* ---------------------------- loop ---------------------------- */
void loop() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    int v = readAxis(AXIS_PINS[i]);
    if (v < axisMin[i]) axisMin[i] = v;
    if (v > axisMax[i]) axisMax[i] = v;
  }

  unsigned long now = millis();

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    printLive();
  }

  if (now - lastStatMs >= STAT_MS) {
    lastStatMs = now;
    printReport();
    resetStats();
  }
}

/* ---------------------------- 工具函数 ---------------------------- */

/* 多次采样取平均，抑制抖动 */
int readAxis(int pin) {
  long sum = 0;
  for (int k = 0; k < 8; k++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / 8);
}

/* 每 300ms 打印一行实时数值，偏离基线的打 * 标记 */
void printLive() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    int v = readAxis(AXIS_PINS[i]);
    Serial.print("A");
    Serial.print(i);
    Serial.print("=");
    Serial.print(v);
    if (abs(v - axisBase[i]) > ACTIVE_DELTA) Serial.print("*");   // * = 这一路在动
    Serial.print("\t");
  }

  Serial.print("| BTN: ");
  bool anyBtn = false;
  for (int p = BTN_FIRST; p <= BTN_LAST; p++) {
    if (digitalRead(p) == LOW) {                                  // 按下 = LOW
      Serial.print("D");
      Serial.print(p);
      Serial.print(" ");
      anyBtn = true;
    }
  }
  if (!anyBtn) Serial.print("-");
  Serial.println();
}

/* 每 3 秒结一次论：哪些口是摇杆轴 */
void printReport() {
  bool found = false;

  for (int i = 0; i < AXIS_COUNT; i++) {
    int range = axisMax[i] - axisMin[i];
    if (range <= ACTIVE_RANGE) continue;

    if (!found) {
      Serial.println();
      Serial.println("--- 本轮结论 ---");
      found = true;
    }

    Serial.print(">>> A");
    Serial.print(i);
    Serial.print(" 检测到摇杆轴，活动范围 ");
    Serial.print(axisMin[i]);
    Serial.print(" ~ ");
    Serial.print(axisMax[i]);
    Serial.print("  (中位基线 ");
    Serial.print(axisBase[i]);
    Serial.println(")");
    Serial.print("    → 主程序里把某一路 joyPin 改成 A");
    Serial.print(i);
    Serial.print("；若推杆方向与舵机转向相反，把对应 DIR 改成 -1");
    Serial.println();

    /* 若扫到 A4/A5，提示一下可能是 I2C 口被占用导致的假象 */
    if (i == 4 || i == 5) {
      Serial.println("    注意: A4/A5 也是 I2C(SDA/SCL)，若有 I2C 模块在用，这里的读数不可信");
    }
  }

  if (!found) {
    Serial.println();
    Serial.println("--- 本轮无变化：摇杆没动、或这一轮没人碰摇杆 ---");
  }
  Serial.println();
}

/* 换窗口时把统计值拉回当前读数，避免旧动作一直触发结论 */
void resetStats() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    int v = readAxis(AXIS_PINS[i]);
    axisMin[i] = v;
    axisMax[i] = v;
  }
}
