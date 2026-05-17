# Akm_Car（阿克曼转向车型）运动控制技术文档

## 一、概述

Akm_Car 是 Origincar 项目支持的6种车型之一，采用**阿克曼转向机构**：前轮舵机转向，后轮双电机差速驱动。这种结构常见于真实汽车，具有转弯半径大、行驶稳定的特点。

## 二、关键机械参数

**来源文件**: `BALANCE/robot_select_init.h`

| 参数 | 宏定义 | 值 | 单位 | 说明 |
|------|--------|-----|------|------|
| 轮距 | `Akm_wheelspacing` | 0.162 | m | 左右后轮之间的距离 |
| 轴距 | `Akm_axlespacing` | 0.144 | m | 前后轴之间的距离 |
| 电机减速比 | `HALL_30F` | 30 | -- | 30:1 减速比 |
| 编码器线数 | `Hall_13` | 13 | 线 | Hall 编码器 13 线 |
| 编码器倍频 | `EncoderMultiples` | 4 | -- | 4x 倍频（TI12 模式） |
| 轮胎直径 | `Black_WheelDiameter` | 0.065 | m | 黑色轮胎直径 65mm |
| 最小转弯半径 | `MINI_AKM_MIN_TURN_RADIUS` | 0.350 | m | 由机械结构决定 |
| 舵机零点 | `SERVO_INIT` | 1500 | us | 舵机中位 PWM 脉宽 |

**派生计算值**:
```
编码器精度 = EncoderMultiples(4) × 编码器线数(13) × 减速比(30) = 1560
轮子周长   = 轮胎直径(0.065) × π = 0.2042 m
```

## 三、运动学逆解

### 3.1 输入参数定义

对于 Akm_Car，三个轴的含义与其他车型不同：
- **Vx**: 前进/后退速度 (m/s)
- **Vy**: 不使用（Akm_Car 不能横向移动），恒为 0
- **Vz**: **右前轮转向角度** (rad)，并非角速度

### 3.2 阿克曼转向几何模型

```
                    ┌─────────────┐
                    │   前轴      │
                    │  ┌─────┐    │
           δ_left ← │  │舵机 │    │ → δ_right
                    │  └─────┘    │
                    │      L      │
                    │   (轴距)    │
                    ├─────────────┤
                    │   后轴      │
          V_A ← ── │ ──────── ── │ ── → V_B
          (左轮)   │      W      │   (右轮)
                   │    (轮距)    │
                    └─────────────┘
                         ↓
                      车体前进方向
```

**关键公式**:
```
转弯半径:     R = L / tan(δ)
左后轮速度:   V_A = Vx × (R - W/2) / R
右后轮速度:   V_B = Vx × (R + W/2) / R
```

**几何直觉**: 转弯时内侧轮（左轮，MOTOR_A）走的圆弧半径小，速度慢；外侧轮（右轮，MOTOR_B）走的圆弧半径大，速度快。差值由 `W/R` 决定。

### 3.3 核心逆解代码

**来源文件**: `BALANCE/balance.c` 第 74-114 行

```c
else if (Car_Mode == Akm_Car) 
{
    double R, Ratio = 636.56, AngleR, Angle_Servo;
    
    // Vz 代表右前轮转向角度，先做角度换算
    AngleR = -Vz * 3.1415 / 120;
    
    // 转弯半径 R = 轴距 / tan(右前轮转角)
    R = Axle_spacing / tan(-AngleR);
    
    // 前轮转向角度限幅: ±0.80 rad (约 ±45.8°)
    AngleR = target_limit_float(AngleR, -0.80f, 0.80f);
    
    // 左右后轮差速逆解
    if (AngleR != 0)
    {
        // 左轮(MOTOR_A) = Vx * (R - 半轮距) / R
        MOTOR_A.Target = Vx * (R - 0.5f * Wheel_spacing) / R;
        // 右轮(MOTOR_B) = Vx * (R + 半轮距) / R
        MOTOR_B.Target = Vx * (R + 0.5f * Wheel_spacing) / R;
    }
    else  // 直行
    {
        MOTOR_A.Target = Vx;
        MOTOR_B.Target = Vx;
    }
    
    // 舵机 PWM 角度转换
    Angle_Servo = AngleR * 750 * 4 / 3.1415;
    Servo = SERVO_INIT + Angle_Servo;
    
    // 速度限幅 ±10 m/s
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
    MOTOR_C.Target = 0;  // 不使用
    MOTOR_D.Target = 0;  // 不使用
    
    // 舵机 PWM 限幅 750~2250 us
    Servo = target_limit_int(Servo, 750, 2250);
}
```

### 3.4 计算示例

**场景**: 以 0.5 m/s 速度右转，右前轮转角 0.3 rad

```
1. 转弯半径:
   R = 0.144 / tan(0.3) = 0.144 / 0.309 = 0.466 m

2. 左后轮速度:
   V_A = 0.5 × (0.466 - 0.081) / 0.466 = 0.5 × 0.826 = 0.413 m/s

3. 右后轮速度:
   V_B = 0.5 × (0.466 + 0.081) / 0.466 = 0.5 × 1.174 = 0.587 m/s

4. 舵机 PWM:
   Angle_Servo = 0.3 × 750 × 4 / 3.1415 = 286.5 us
   Servo = 1500 + 286.5 = 1786.5 us
```

## 四、舵机转向控制

### 4.1 舵机硬件

```c
// HARDWARE/motor.h
#define Servo_PWM  TIM12->CCR2    // 舵机 PWM 输出寄存器
#define SERVO_INIT 1500           // 舵机零点：1500us 脉宽
```

舵机由 **TIM12 通道2** (PB15 引脚) 输出 PWM，频率 100Hz，配置在 `systemInit()` 中:
```c
TIM12_SERVO_Init(9999, 84-1);  
// APB1 时钟 84MHz, 频率 = 84M / ((9999+1) × (83+1)) = 100Hz
```

### 4.2 角度到 PWM 的转换

```c
// 线性映射: AngleR (rad) -> PWM 偏移量
Angle_Servo = AngleR * 750 * 4 / 3.1415;
Servo = SERVO_INIT + Angle_Servo;
```

**计算过程**:
```
系数 = 750 × 4 / π = 750 × 4 / 3.1415 ≈ 954.93 us/rad

当 AngleR = +0.80 rad (+45.8°):
  Angle_Servo = 0.80 × 954.93 ≈ 763.9 us
  Servo = 1500 + 764 = 2264 us → 限幅到 2250 us

当 AngleR = -0.80 rad (-45.8°):
  Angle_Servo = -0.80 × 954.93 ≈ -763.9 us
  Servo = 1500 - 764 = 736 us → 限幅到 750 us

当 AngleR = 0 (直行):
  Servo = 1500 us (零点)
```

### 4.3 舵机 PWM 限幅

```c
Servo = target_limit_int(Servo, 750, 2250);  // cmj: default 800,2200
```

PWM 范围 **750 ~ 2250 us**，对应舵机左右极限转角。

### 4.4 Set_Pwm 中的舵机输出

```c
// BALANCE/balance.c 第 230 行
case Akm_Car: Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, 16799, -16799, Servo); break;
```

注意：Akm_Car 只使用 MOTOR_A（左后轮）和 MOTOR_B（右后轮），MOTOR_C 和 MOTOR_D 被设为固定值 16799 和 -16799（即 C、D 电机不输出实际转速，PWM 值使其处于"空转"状态）。

## 五、电机控制

### 5.1 使用电机

- **MOTOR_A**: 左后轮（PB8/PB9, TIM10/TIM11）
- **MOTOR_B**: 右后轮（PE5/PE6, TIM9）
- **MOTOR_C**: 未使用（固定 16799）
- **MOTOR_D**: 未使用（固定 -16799）

### 5.2 速度闭环控制

```c
// BALANCE/balance.c 第 217-220 行
MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
```

每个电机有独立的 **增量式 PI 控制器**:

```c
int Incremental_PI_A(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder;              // 计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    // Pwm += Kp × [e(k) - e(k-1)] + Ki × e(k)
    if (Pwm > 16700)  Pwm = 16700;
    if (Pwm < -16700) Pwm = -16700;
    Last_bias = Bias;
    return Pwm;
}
```

**PID 参数**: `Velocity_KP = 300`, `Velocity_KI = 300`（无 D 项），PWM 限幅 ±16700。

### 5.3 电机 PWM 输出极性

```c
// BALANCE/balance.c 第 230 行
case Akm_Car: Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, 16799, -16799, Servo); break;
```

- MOTOR_A 和 MOTOR_B 使用**正极性**输出（不取反）
- MOTOR_C = 16799, MOTOR_D = -16799（固定值，对应空转状态）
- 最后一个参数 Servo 赋值给舵机

### 5.4 Set_Pwm 正反转逻辑

```c
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
    // 采用双路 PWM 差分驱动
    if (motor_a < 0)  PWMA1 = 16799, PWMA2 = 16799 + motor_a;  // 反转
    else              PWMA2 = 16799, PWMA1 = 16799 - motor_a;  // 正转
    
    // ... 类似处理 motor_b, motor_c, motor_d
    
    Servo_PWM = servo;  // TIM12->CCR2 = servo
}
```

PWM 频率 10kHz (ARR=16799, PSC=0)，占空比通过两个互补通道的差值实现正反转。

## 六、编码器处理

### 6.1 编码器读取

```c
void Get_Velocity_Form_Encoder(void)
{
    OriginalEncoder.A = Read_Encoder(2);  // TIM2 - Motor A
    OriginalEncoder.B = Read_Encoder(3);  // TIM3 - Motor B
    OriginalEncoder.C = Read_Encoder(4);  // TIM4 - Motor C
    OriginalEncoder.D = Read_Encoder(5);  // TIM5 - Motor D
```

### 6.2 Akm_Car 编码器方向调整

```c
case Akm_Car:
    Encoder_A_pr =  OriginalEncoder.A;   // 左轮：正方向
    Encoder_B_pr = -OriginalEncoder.B;   // 右轮：取反（安装方向相反）
    Encoder_C_pr =  OriginalEncoder.C;   // 未使用
    Encoder_D_pr =  OriginalEncoder.D;   // 未使用
    break;
```

**关键**: MOTOR_B（右轮）编码器值需要**取反**，这是因为右轮电机安装方向与左轮相反，为了使两个轮子的前进方向编码器值符号一致。

### 6.3 编码器值转换为速度

```c
// 编码器原始数据转换为车轮速度，单位 m/s
MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
```

代入数值:
```
速度 = 编码器脉冲数 × 100Hz × 0.2042m / 1560
     = 编码器脉冲数 × 0.01309 m/s per pulse
```

## 七、运动学正解（反馈上报）

**来源文件**: `HARDWARE/usartx.c` 第 62-66 行

```c
case Akm_Car:
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000;  // 纵向速度
    Send_Data.Sensor_Str.Y_speed = 0;                                                  // 无横向速度
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing) * 1000;  // 角速度
    break;
```

**正解公式**:
```
Vx = (V_left + V_right) / 2         -- 车体纵向速度
Vy = 0                               -- 无横向运动
ωz = (V_right - V_left) / W         -- 车体角速度 (rad/s)
```

## 八、遥控输入映射到运动控制

### 8.1 APP 遥控 (Get_RC)

**来源文件**: `BALANCE/balance.c` 第 428-492 行

非全向车的控制逻辑:
```c
switch(Flag_Direction)
{
    case 1: Move_X = +RC_Velocity; Move_Z = 0;      break;  // 前进
    case 2: Move_X = +RC_Velocity; Move_Z = -PI/2;  break;  // 前进+右转
    case 3: Move_X = 0;            Move_Z = -PI/2;  break;  // 原地右转
    case 4: Move_X = -RC_Velocity; Move_Z = -PI/2;  break;  // 后退+右转
    case 5: Move_X = -RC_Velocity; Move_Z = 0;      break;  // 后退
    case 6: Move_X = -RC_Velocity; Move_Z = +PI/2;  break;  // 后退+左转
    case 7: Move_X = 0;            Move_Z = +PI/2;  break;  // 原地左转
    case 8: Move_X = +RC_Velocity; Move_Z = +PI/2;  break;  // 前进+左转
    default: Move_X = 0;           Move_Z = 0;      break;  // 停止
}
```

**Akm_Car 专用的 Z 轴转换**:
```c
if (Car_Mode == Akm_Car)
{
    // 将 Move_Z 从通用角速度值转换为前轮转角
    Move_Z = Move_Z * 2 / 9;
}
```

当 `Move_Z = PI/2 ≈ 1.5708` 时:
```
转换后 Move_Z = 1.5708 * 2/9 ≈ 0.349 rad
```
这个值传入 `Drive_Motor()` 后在 Akm 分支中被进一步处理为前轮转角。

**单位转换**:
```c
Move_X = Move_X / 1000;  // mm/s → m/s
```

### 8.2 PS2 手柄控制 (PS2_control)

**来源文件**: `BALANCE/balance.c` 第 502-555 行

```c
void PS2_control(void)
{
    // 128 为中值，坐标系转换
    LY = -(PS2_LX - 128);   // 左摇杆 X → LY
    LX = -(PS2_LY - 128);   // 左摇杆 Y → LX（前进后退）
    RY = -(PS2_RX - 128);   // 右摇杆 X → RY（转向）
    
    // 死区阈值 = 20
    if (LX > -Threshold && LX < Threshold) LX = 0;
    if (LY > -Threshold && LY < Threshold) LY = 0;
    if (RY > -Threshold && RY < Threshold) RY = 0;
    
    // 加减速按键
    if (PS2_KEY == 11)  RC_Velocity += 5;   // R1 加速
    if (PS2_KEY == 9)   RC_Velocity -= 5;   // L1 减速
    
    // 映射到三轴速度
    Move_X = LX * RC_Velocity / 128;    // 前进后退
    Move_Y = LY * RC_Velocity / 128;    // 横向（Akm 不用）
    Move_Z = RY * (PI/2) / 128;         // 转向角度
```

**Akm_Car 专用转换**:
```c
else if (Car_Mode == Akm_Car)
{
    Move_Z = Move_Z * 2 / 9;  // 转换为前轮转角
}
```

### 8.3 航模遥控 (Remote_Control)

**来源文件**: `BALANCE/balance.c` 第 565-641 行

```c
void Remote_Control(void)
{
    // 通道映射（以 1500 为中心，范围 1000~2000）
    LX = Remoter_Ch2 - 1500;  // 左摇杆前后 → 前进后退
    LY = Remoter_Ch4 - 1500;  // 左摇杆左右 → 横向/舵机
    RX = Remoter_Ch3 - 1500;  // 右摇杆前后 → 油门/加减速
    RY = Remoter_Ch1 - 1500;  // 右摇杆左右 → 转向
    
    // 死区阈值 = 100
    // 油门叠加
    Remote_RCvelocity = RC_Velocity + RX;
    
    Move_X =  LX * Remote_RCvelocity / 500;
    Move_Y = -LY * Remote_RCvelocity / 500;
    Move_Z = -RY * (PI/2) / 500;
    
    // Akm_Car Z轴转换
    if (Car_Mode == Akm_Car)
    {
        Move_Z = Move_Z * 2 / 9;
    }
}
```

**航模遥控特殊处理**: 进入航模模式后前 1 秒内数据不处理（`thrice` 计数器），防止误操作。

### 8.4 串口/CAN/ROS 控制

**来源文件**: `HARDWARE/usartx.c`

**串口1** (第 509-519 行):
```c
Move_X = XYZ_Target_Speed_transition(rxbuf[3], rxbuf[4]);
Vz     = XYZ_Target_Speed_transition(rxbuf[7], rxbuf[8]);
if (Car_Mode == Akm_Car)
{
    Move_Z = Vz_to_Akm_Angle(Move_X, Vz);  // 将 Vx,Vz 转换为右前轮转角
}
```

**串口3/串口5** (第 694-698 行):
```c
if (Car_Mode == Akm_Car)
{
    Move_Z = Vz_to_Akm_Angle_cmj(Vz);  // 直接透传 Vz 作为转角
}
```

**CAN 接收** (`HARDWARE/can.c` 第 328-330 行):
```c
if (Car_Mode == Akm_Car)
{
    Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
}
```

### 8.5 Vz_to_Akm_Angle 函数详解

**来源文件**: `HARDWARE/usartx.c` 第 788-824 行

这个函数将 ROS 上位机发来的 **(Vx, Vz)** 对转换为阿克曼的**右前轮转角**:

```c
float Vz_to_Akm_Angle(float Vx, float Vz)
{
    float R, AngleR, Min_Turn_Radius;
    Min_Turn_Radius = MINI_AKM_MIN_TURN_RADIUS;  // 0.350m
    
    if (Vz != 0 && Vx != 0)
    {
        // 如果要求的转弯半径 < 最小转弯半径，则限制角速度
        if (float_abs(Vx/Vz) <= Min_Turn_Radius)
        {
            if (Vz > 0)
                Vz =  float_abs(Vx) / Min_Turn_Radius;
            else
                Vz = -float_abs(Vx) / Min_Turn_Radius;
        }
        R = Vx / Vz;  // 转弯半径
        AngleR = atan(Axle_spacing / (R + 0.5f * Wheel_spacing));
        // 右前轮转角 = arctan(轴距 / (转弯半径 + 半轮距))
    }
    else
    {
        AngleR = 0;  // 直行
    }
    return AngleR;
}
```

**阿克曼转向几何**:
```
右前轮转角 δ_right = arctan(L / (R + W/2))
左前轮转角 δ_left  = arctan(L / (R - W/2))  -- 代码中未使用

其中 R = Vx / Vz 为转弯半径
```

## 九、完整数据流

```
遥控输入 (APP/PS2/航模/串口/CAN)
    │
    ▼
┌──────────────────────────────────────────────────────┐
│  输入映射 & 轴转换                                     │
│  - APP:  Flag_Direction → Move_X, Move_Z             │
│  - PS2:  摇杆模拟量 → Move_X, Move_Z                  │
│  - 航模: PWM捕获 → Move_X, Move_Z                    │
│  - 串口: 帧解析 → Vx, Vz → Vz_to_Akm_Angle()        │
│  - Akm专用: Move_Z = Move_Z × 2/9                    │
│  - 单位: mm/s → m/s                                  │
└──────────────┬───────────────────────────────────────┘
               │ Drive_Motor(Move_X, Move_Y=0, Move_Z)
               ▼
┌──────────────────────────────────────────────────────┐
│  运动学逆解 (Akm_Car 分支)                             │
│  AngleR = -Vz × PI/120                               │
│  R = Axle_spacing / tan(-AngleR)                     │
│  MOTOR_A.Target = Vx × (R - W/2) / R   (左后轮)      │
│  MOTOR_B.Target = Vx × (R + W/2) / R   (右后轮)      │
│  Servo = 1500 + AngleR × 954.93          (舵机PWM)    │
└──────────────┬───────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────┐
│  增量式 PI 速度闭环 (100Hz)                            │
│  MOTOR_A.Pwm = PI(MOTOR_A.Encoder, MOTOR_A.Target)   │
│  MOTOR_B.Pwm = PI(MOTOR_B.Encoder, MOTOR_B.Target)   │
│  Kp=300, Ki=300, PWM限幅=±16700                      │
└──────────────┬───────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────┐
│  Set_Pwm(A_pwm, B_pwm, 16799, -16799, Servo)        │
│  - Motor A (左后轮): PB8/PB9 (TIM10/TIM11)          │
│  - Motor B (右后轮): PE5/PE6 (TIM9)                  │
│  - Motor C: 固定16799 (不驱动)                        │
│  - Motor D: 固定-16799 (不驱动)                       │
│  - Servo: PB15 (TIM12_CH2), 100Hz PWM               │
└──────────────────────────────────────────────────────┘
```

## 十、关键文件索引

| 文件路径 | 相关内容 |
|---------|---------|
| `BALANCE/balance.c:74-114` | 运动学逆解 |
| `BALANCE/balance.c:217-220` | 速度闭环 PI 控制 |
| `BALANCE/balance.c:230` | Set_Pwm 输出 |
| `BALANCE/balance.c:428-492` | APP 遥控处理 |
| `BALANCE/balance.c:502-555` | PS2 手柄控制 |
| `BALANCE/balance.c:565-641` | 航模遥控处理 |
| `BALANCE/balance.c:664-694` | 编码器读取与转换 |
| `BALANCE/robot_select_init.h:37-100` | Akm_Car 全部物理参数定义 |
| `BALANCE/robot_select_init.c:28` | Akm_Car 参数初始化调用 |
| `BALANCE/system.c:17` | 全局变量 Servo 定义 |
| `BALANCE/system.c:22` | 全局变量 RC_Velocity 定义 |
| `BALANCE/system.c:30` | 全局变量 KP/KI 定义 |
| `BALANCE/system.h:52-59` | Motor_parameter 结构体定义 |
| `BALANCE/system.h:40-48` | CarMode 枚举定义 |
| `HARDWARE/motor.h:51-52` | 舵机 PWM 宏定义 |
| `HARDWARE/timer.c:404-473` | 舵机 PWM 初始化 TIM12_SERVO_Init |
| `HARDWARE/timer.c:51-153` | 航模遥控 TIM8 输入捕获 |
| `HARDWARE/encoder.c` | 编码器 TIM2/3/4/5 初始化与读取 |
| `HARDWARE/usartx.c:62-66` | 运动学正解（反馈上报） |
| `HARDWARE/usartx.c:509-519` | 串口1 Akm 处理 |
| `HARDWARE/usartx.c:694-698` | 串口3/5 Akm 处理 |
| `HARDWARE/usartx.c:788-824` | Vz_to_Akm_Angle 函数 |
| `HARDWARE/can.c:328-330` | CAN 接收 Akm 处理 |
| `BALANCE/show.c:248-257` | OLED 显示 Akm 特有内容（舵机 PWM 值） |

---
*文档生成时间: 2026年5月17日*