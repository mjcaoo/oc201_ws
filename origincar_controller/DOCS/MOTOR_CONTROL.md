# Origincar 电机控制系统技术文档

## 一、整体架构

该项目采用 **STM32F4 + FreeRTOS** 实现多模式机器人电机控制，支持6种车型。控制架构分为4层：

```
┌─────────────────────────────────────────────────────────┐
│               应用层 (Application Layer)                 │
│  main.c → FreeRTOS 任务调度                              │
├─────────────────────────────────────────────────────────┤
│               控制层 (Control Layer)                     │
│  balance.c (Balance_task) → 运动学逆解 + 速度闭环PI      │
│  control.c (EXTI15_10_IRQHandler) → 旧版中断控制路径     │
├─────────────────────────────────────────────────────────┤
│               驱动层 (Driver Layer)                      │
│  motor.c   → PWM输出 (TIM1/9/10/11)                     │
│  encoder.c → 编码器接口 (TIM2/3/4/5)                     │
│  timer.c   → 舵机PWM + 航模遥控捕获 (TIM8/12)            │
├─────────────────────────────────────────────────────────┤
│               硬件层 (Hardware Layer)                    │
│  STM32F4xx GPIO / Timer / NVIC / ADC / I2C / USART      │
└─────────────────────────────────────────────────────────┘
```

**核心控制周期**: 10ms（100Hz），由 FreeRTOS `Balance_task` 精确定时驱动。

## 二、电机驱动设计

### 2.1 电机引脚与PWM映射

**4路电机的 PWM 双通道 H 桥驱动设计**（每路电机需要 2 个 PWM 引脚控制正反转）：

| 电机 | PWM正极引脚 | 定时器通道 | PWM负极引脚 | 定时器通道 |
|------|------------|-----------|------------|-----------|
| Motor_A | PB8 | `TIM10->CCR1` (PWMA1) | PB9 | `TIM11->CCR1` (PWMA2) |
| Motor_B | PE5 | `TIM9->CCR1` (PWMB1) | PE6 | `TIM9->CCR2` (PWMB2) |
| Motor_C | PE11 | `TIM1->CCR2` (PWMC1) | PE9 | `TIM1->CCR1` (PWMC2) |
| Motor_D | PE14 | `TIM1->CCR4` (PWMD1) | PE13 | `TIM1->CCR3` (PWMD2) |

**关键宏定义**:
```c
#define EN     PDin(3)          // 使能开关，PD3上拉输入
#define Servo_PWM  TIM12->CCR2  // 舵机PWM输出
#define SERVO_INIT 1500         // 舵机零点值（1500us脉宽）
```

### 2.2 PWM定时器初始化

**文件路径**: `HARDWARE/motor.c`

包含 5 个初始化函数：

| 函数 | 定时器 | 用途 | GPIO引脚 | 通道数 |
|------|--------|------|----------|--------|
| `Enable_Pin()` | — | 使能开关引脚初始化 | PD3 (上拉输入) | — |
| `TIM1_PWM_Init(arr, psc)` | TIM1 (APB2) | Motor_C + Motor_D PWM | PE9/PE11/PE13/PE14 | 4通道 (CH1-CH4) |
| `TIM9_PWM_Init(arr, psc)` | TIM9 (APB2) | Motor_B PWM | PE5/PE6 | 2通道 (CH1-CH2) |
| `TIM10_PWM_Init(arr, psc)` | TIM10 (APB2) | Motor_A PWM (正) | PB8 | 1通道 (CH1) |
| `TIM11_PWM_Init(arr, psc)` | TIM11 (APB2) | Motor_A PWM (负) | PB9 | 1通道 (CH1) |

**PWM频率计算**: `频率 = 168MHz / ((16799+1) × (0+1)) = 10kHz`

## 三、控制算法

### 3.1 核心数据结构

**文件路径**: `BALANCE/system.h`

```c
// 电机参数结构体
typedef struct  
{
    float Encoder;      // 编码器实时速度值 (m/s)
    float Motor_Pwm;    // 电机PWM输出值
    float Target;       // 目标速度值 (m/s)
    float Velocity_KP;  // 速度控制Kp参数
    float Velocity_KI;  // 速度控制Ki参数
} Motor_parameter;

// 4个电机实例
Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

// 全局PID参数
float Velocity_KP = 300, Velocity_KI = 300;
```

### 3.2 增量式PI控制器实现

**文件路径**: `BALANCE/balance.c`（第380-419行）

4 路电机各自有独立的增量式 PI 控制器（`Incremental_PI_A/B/C/D`），算法完全相同：

```c
int Incremental_PI_A(float Encoder, float Target)
{ 	
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder;               // 计算当前偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;  // 增量式PI
    if(Pwm > 16700)  Pwm = 16700;          // 上限幅
    if(Pwm < -16700) Pwm = -16700;         // 下限幅
    Last_bias = Bias;                       // 保存上次偏差
    return Pwm;                            
}
```

**算法公式**:
```
ΔPWM = Kp × [e(k) - e(k-1)] + Ki × e(k)
PWM  = PWM + ΔPWM
```

- 只使用 **PI 控制**（无微分项 D），适合速度环
- PWM 范围: **-16700 ~ +16700**（对应 ARR=16799）
- 增量式PID的优点：无需积分累加，切换时无冲击

### 3.3 PWM限幅与安全保护

```c
// PWM限幅函数 (balance.c)
void Limit_Pwm(int amplitude)
{	
    MOTOR_A.Motor_Pwm = target_limit_float(MOTOR_A.Motor_Pwm, -amplitude, amplitude);
    // ... B, C, D 同理
}

// 电压过低/使能开关/软件失能 检测 (balance.c)
u8 Turn_Off(int voltage)
{
    if(voltage < 10 || EN == 0 || Flag_Stop == 1) {
        PWMA1=0; PWMA2=0; PWMB1=0; PWMB2=0;
        PWMC1=0; PWMC1=0; PWMD1=0; PWMD2=0;
        return 1;  // 异常，禁止运动
    }
    return 0;      // 正常
}
```

## 四、编码器反馈

### 4.1 编码器初始化

**文件路径**: `HARDWARE/encoder.c`

4 路编码器分别使用 TIM2/TIM3/TIM4/TIM5，全部配置为 **编码器接口模式3**（TI12，双通道同时计数，4倍频）：

| 编码器 | 定时器 | GPIO | 对应电机 |
|--------|--------|------|----------|
| Encoder_A | TIM2 | PA15 + PB3 | Motor_A (在balance.c中) |
| Encoder_B | TIM3 | PB4 + PB5 | Motor_B |
| Encoder_C | TIM4 | PB6 + PB7 | Motor_C |
| Encoder_D | TIM5 | PA0 + PA1 | Motor_D |

**编码器配置要点**:
- `ENCODER_TIM_PERIOD = 65535`（16位计数器最大值）
- 预分频 = 0（不分频）
- `TIM_EncoderMode_TI12` = 模式3，TI1和TI2都计数，4倍频
- GPIO 配置为**开漏上拉**（`GPIO_OType_OD`, `GPIO_PuPd_UP`）

### 4.2 编码器读取函数

```c
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX) {
        case 2: Encoder_TIM = (short)TIM2->CNT; TIM2->CNT = 0; break;
        case 3: Encoder_TIM = (short)TIM3->CNT; TIM3->CNT = 0; break;
        case 4: Encoder_TIM = (short)TIM4->CNT; TIM4->CNT = 0; break;	
        case 5: Encoder_TIM = (short)TIM5->CNT; TIM5->CNT = 0; break;	
        default: Encoder_TIM = 0;
    }
    return Encoder_TIM;
}
```

**关键特性**: 读取后**立即清零**（`CNT=0`），实现增量式测速，每次读取得到的是自上次读取以来的脉冲增量。强制转换为 `short` 处理溢出情况。

### 4.3 编码器数据到速度的转换

**文件路径**: `BALANCE/balance.c`（第664-694行）

```c
void Get_Velocity_Form_Encoder(void)
{
    // 1. 读取原始编码器增量
    OriginalEncoder.A = Read_Encoder(2);  // TIM2
    OriginalEncoder.B = Read_Encoder(3);  // TIM3
    OriginalEncoder.C = Read_Encoder(4);  // TIM4
    OriginalEncoder.D = Read_Encoder(5);  // TIM5

    // 2. 根据车型调整极性（不同车型安装方向不同）
    switch(Car_Mode) {
        case Akm_Car: 
            Encoder_A_pr = OriginalEncoder.A; 
            Encoder_B_pr = -OriginalEncoder.B;  // 取反
            ...
    }
    
    // 3. 转换为国际单位 m/s
    // 公式: 速度 = 脉冲数 × 控制频率 × 轮子周长 / 编码器精度
    MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
}
```

**转换参数**:
- `CONTROL_FREQUENCY = 100` (Hz，即 10ms 读取一次)
- `Encoder_precision = EncoderMultiples(4) × 编码器线数(13) × 减速比(30) = 1560`
- `Wheel_perimeter = 直径 × π`

## 五、核心控制流程

### 5.1 Balance_task 主循环

**文件路径**: `BALANCE/balance.c`（第171-241行）

```
Balance_task (100Hz, 10ms周期)
 │
 ├── Get_Velocity_Form_Encoder()     // 读4路编码器 → 转换为m/s
 │
 ├── 遥控源判断:
 │   ├── APP_ON_Flag    → Get_RC()           // 蓝牙APP
 │   ├── Remote_ON_Flag → Remote_Control()   // 航模遥控
 │   ├── PS2_ON_Flag    → PS2_control()      // PS2手柄
 │   └── 默认           → Drive_Motor(Move_X, Move_Y, Move_Z)
 │
 ├── Drive_Motor(Vx, Vy, Vz)         // 运动学逆解
 │   ├── Mec_Car:  麦克纳姆轮逆解
 │   ├── Omni_Car: 全向轮逆解
 │   ├── Akm_Car:  阿克曼逆解 + 舵机角度
 │   ├── Diff_Car: 差速逆解
 │   ├── FourWheel_Car: 四驱逆解
 │   └── Tank_Car: 履带逆解
 │
 ├── Turn_Off(Voltage)               // 安全检测（电压/使能/软件标志）
 │
 ├── Incremental_PI_A/B/C/D()       // 4路增量式PI速度闭环
 │
 ├── Limit_Pwm(16700)                // PWM限幅
 │
 └── Set_Pwm(A, B, C, D, Servo)     // 写入PWM寄存器
      ├── H桥方向控制（两通道差值控制正反转）
      └── Servo_PWM = servo          // 舵机输出
```

### 5.2 Set_Pwm 的 H 桥驱动原理

```c
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
    // 以Motor_A为例：双通道差值驱动H桥
    if(motor_a < 0)
        PWMA1 = 16799, PWMA2 = 16799 + motor_a;  // 反转：PWMA1满占空比，PWMA2减小
    else
        PWMA2 = 16799, PWMA1 = 16799 - motor_a;  // 正转：PWMA2满占空比，PWMA1减小
    
    Servo_PWM = servo;  // 舵机直接写入
}
```

这种"一满一调"的差值 PWM 驱动方式是 H 桥电机驱动的典型实现，通过两个通道的占空比差来控制电机的转速和方向。

## 六、车型支持

### 6.1 车型选择机制

通过**电位器 ADC 值**在开机时自动选择（`BALANCE/robot_select_init.c` 的 `Robot_Select()` 函数）。

### 6.2 支持的车型

| 车型编号 | 枚举值 | 使用电机 | 运动学模型 |
|----------|--------|----------|-----------|
| 0 - Mec_Car | `Mec_Car` | A, B, C, D | 麦克纳姆轮全向 |
| 1 - Omni_Car | `Omni_Car` | A, B, C | 三轮全向 |
| 2 - Akm_Car | `Akm_Car` | A, B + 舵机 | 阿克曼转向 |
| 3 - Diff_Car | `Diff_Car` | A, B | 两轮差速 |
| 4 - FourWheel_Car | `FourWheel_Car` | A, B, C, D | 四驱差速 |
| 5 - Tank_Car | `Tank_Car` | A, B | 履带差速 |

### 6.3 车型参数初始化

**文件路径**: `BALANCE/robot_select_init.c`

```c
void Robot_Init(float Wheel_spacing, float Axle_spacing, int Encoder_multiples, 
                float Wheel_perimeter, float Reduction_ratio, int Encoder_precision)
{
    // 计算编码器精度
    Encoder_precision = Encoder_multiples * 编码器线数 * Reduction_ratio;
    // 计算轮子周长
    Wheel_perimeter = 直径 * π;
    // 设置轮距、轴距等参数
}
```

## 七、定时器配置汇总

### 7.1 PWM输出定时器（电机驱动）

| 定时器 | 时钟源 | ARR | PSC | PWM频率 | 用途 |
|--------|--------|-----|-----|---------|------|
| **TIM1** | APB2 (168MHz) | 16799 | 0 | **10kHz** | Motor_C (CH1,CH2) + Motor_D (CH3,CH4) |
| **TIM9** | APB2 (168MHz) | 16799 | 0 | **10kHz** | Motor_B (CH1,CH2) |
| **TIM10** | APB2 (168MHz) | 16799 | 0 | **10kHz** | Motor_A 正 (CH1) |
| **TIM11** | APB2 (168MHz) | 16799 | 0 | **10kHz** | Motor_A 负 (CH1) |

### 7.2 舵机/遥控定时器

| 定时器 | 时钟源 | ARR | PSC | 频率 | 用途 |
|--------|--------|-----|-----|------|------|
| **TIM12** | APB1 (84MHz) | 9999 | 83 | **100Hz** | 舵机PWM输出 (CH1,CH2) |
| **TIM8** | APB2 (168MHz) | 9999 | 167 | **100Hz** | 航模遥控输入捕获 (CH1-CH4) |

### 7.3 编码器定时器

| 定时器 | 时钟源 | 模式 | 用途 |
|--------|--------|------|------|
| **TIM2** | APB1 | 编码器模式3 (TI12) | 编码器A (PA15+PB3) |
| **TIM3** | APB1 | 编码器模式3 (TI12) | 编码器B (PB4+PB5) |
| **TIM4** | APB1 | 编码器模式3 (TI12) | 编码器C (PB6+PB7) |
| **TIM5** | APB1 | 编码器模式3 (TI12) | 编码器D (PA0+PA1) |

## 八、初始化流程

### 8.1 完整初始化序列

**文件路径**: `BALANCE/system.c`（`systemInit()`函数）

```
main()
 └─ systemInit()
     │
     ├── 1. NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)  // 4位抢占优先级
     ├── 2. delay_init(168)                                   // 延时函数（168MHz）
     ├── 3. LED_Init() / Buzzer_Init()                       // LED和蜂鸣器
     ├── 4. Enable_Pin()                                      // 使能开关 PD3
     ├── 5. OLED_Init() / KEY_Init()                         // 显示屏和按键
     ├── 6. uart1/2/3/5_init()                               // 4路串口
     ├── 7. Adc_Init() / Adc_POWER_Init()                   // ADC（电池电压+电位器）
     ├── 8. CAN1_Mode_Init()                                 // CAN通信
     │
     ├── 9. ★ Robot_Select()                                 // ★ 核心：根据电位器选择车型
     │      └── Robot_Init(轮距, 轴距, 自转半径, 减速比, 编码器线数, 轮径)
     │           ├── 计算 Encoder_precision = 4 × 线数 × 减速比
     │           ├── 计算 Wheel_perimeter = 直径 × π
     │           └── 设置 Wheel_spacing, Axle_spacing 等
     │
     ├── 10. Encoder_Init_TIM2/3/4/5()                       // 4路编码器（4倍频）
     ├── 11. TIM12_SERVO_Init(9999, 84-1)                   // 舵机PWM (100Hz)
     ├── 12. TIM8_Cap_Init(9999, 168-1)                     // 航模遥控输入捕获
     ├── 13. ★ TIM1/9/10/11_PWM_Init(16799, 0)             // ★ 4路电机PWM (10kHz)
     ├── 14. I2C_GPIOInit() / MPU6050_initialize()          // IMU初始化
     └── 15. PS2_Init() / PS2_SetInit()                     // PS2手柄
```

### 8.2 FreeRTOS 任务创建

**文件路径**: `USER/main.c`

```
main() → xTaskCreate(start_task) → vTaskStartScheduler()
 start_task:
   ├── Balance_task  (优先级4, 512字节栈) ← 核心运动控制
   ├── MPU6050_task  ← IMU数据读取
   ├── show_task     ← OLED显示
   ├── led_task      ← LED闪烁
   ├── pstwo_task    ← PS2手柄读取
   └── data_task     ← 串口/CAN数据发送
```

## 九、两套控制路径说明

项目中存在**两套并行的控制代码**：

| 特性 | `control.c` (旧版) | `balance.c` (新版) |
|------|---------------------|---------------------|
| 触发方式 | MPU6050外部中断 `EXTI15_10_IRQHandler` | FreeRTOS定时任务 `Balance_task` |
| 控制周期 | 10ms (由Flag_Target交替) | 10ms (100Hz) |
| 车型支持 | 仅2轮（A/B电机） | 6种车型（A/B/C/D 4电机） |
| 电机数 | 2 (Motor_A, Motor_B) | 4 (MOTOR_A~D) |
| PWM限幅 | 6900 / 7200 | 16700 |
| Set_Pwm参数 | (motor_a, motor_b, servo) | (motor_a, motor_b, motor_c, motor_d, servo) |
| 运动学 | 简单差速 + 舵机 | 6种车型完整逆解 |

**当前系统实际使用的是 `balance.c` 路径**（通过 FreeRTOS `Balance_task` 驱动）。

## 十、关键文件清单

| 文件 | 绝对路径 | 核心功能 |
|------|----------|----------|
| motor.c | `HARDWARE/motor.c` | PWM定时器初始化 (TIM1/9/10/11) |
| motor.h | `HARDWARE/motor.h` | 电机引脚/PWM映射宏定义 |
| encoder.c | `HARDWARE/encoder.c` | 编码器接口初始化 (TIM2/3/4/5) + 读取函数 |
| encoder.h | `HARDWARE/encoder.h` | 编码器函数声明 + ENCODER_TIM_PERIOD |
| timer.c | `HARDWARE/timer.c` | TIM8航模遥控捕获 + TIM8/12舵机PWM |
| timer.h | `HARDWARE/timer.h` | timer函数声明 |
| control.c | `BALANCE/control.c` | 旧版中断控制路径 (2轮) |
| control.h | `BALANCE/control.h` | 旧版控制函数声明 |
| balance.c | `BALANCE/balance.c` | **核心控制文件**：运动学逆解 + PI控制 + 遥控处理 |
| balance.h | `BALANCE/balance.h` | balance函数声明 + FreeRTOS任务参数 |
| system.c | `BALANCE/system.c` | 全局变量定义 + systemInit() 初始化入口 |
| system.h | `BALANCE/system.h` | 数据结构定义 (Motor_parameter等) + 外部变量声明 |
| robot_select_init.c | `BALANCE/robot_select_init.c` | 车型选择 + 机械参数初始化 |
| robot_select_init.h | `BALANCE/robot_select_init.h` | 车型参数宏定义 (轮距/轴距/减速比等) |
| main.c | `USER/main.c` | FreeRTOS启动 + 任务创建 |

---
*文档生成时间: 2026年5月17日*