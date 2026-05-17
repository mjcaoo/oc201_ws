# AGENTS.md — Origincar Controller

Embedded C firmware for a multi-mode robot running STM32F407 + FreeRTOS.
Supports 6 vehicle types (Mecanum, Omni, Ackermann, Diff, 4WD, Tank) with 4 motors.

## Build Toolchain

- **IDE**: Keil MDK-ARM v5 (AC5 compiler) or EIDE (Embedded IDE for VS Code)
- **No CLI build** — this project is built via IDE GUI, not `make`/`cmake`
- **Build output**: `OBJ/OriginCar.hex`
- **Flash uploader**: JLink
- **Clean script**: `keilkilll.bat` — deletes all Keil build artifacts (`.o`, `.d`, `.axf`, `.map`, etc.)

### Critical Preprocessor Defines

These must be set or the build will fail or behave incorrectly:

```
STM32F40_41xxx
USE_STDPERIPH_DRIVER
__FPU_PRESENT=1
__TARGET_FPU_VFP
ARM_MATH_CM4
__CC_ARM
```

## Directory Ownership

| Directory | Role |
|-----------|------|
| `USER/` | Entry point (`main.c`), interrupt handlers, system init, Keil project files (`.uvprojx`) |
| `BALANCE/` | **Core control logic** — motion control, kinematics, PI controllers, vehicle selection |
| `HARDWARE/` | Peripheral drivers — motor PWM, encoder, OLED, PS2, ADC, CAN, timer, MPU6050/I2C |
| `SYSTEM/` | Low-level system — delay, sys init, USART |
| `FWLIB/` | STM32 Standard Peripheral Library (vendor code, do not modify) |
| `FreeRTOS/` | FreeRTOS kernel (vendor code) — uses `portable/RVDS/ARM_CM4F` + `heap_4.c` |
| `CORE/` | ARM Cortex-M4 startup (`startup_stm32f40_41xxx.s`) and core headers |
| `OBJ/` | Build output (`.hex`) |

## Architecture — What Runs Where

- **Entry**: `USER/main.c` → `systemInit()` → FreeRTOS scheduler → `start_task` creates all other tasks
- **Control loop**: `BALANCE/balance.c:Balance_task` — 100Hz (10ms), the heart of the system
- **IMU**: `BALANCE/` → `MPU6050_task` reads gyro/accel
- **Display**: `BALANCE/show.c` → `show_task` drives OLED
- **RC input**: PS2 controller (`HARDWARE/pstwo.c`),航模遥控 (`HARDWARE/timer.c` TIM8 capture), BLE APP

### Two Control Paths (Legacy vs Active)

`BALANCE/control.c` is the **old** interrupt-driven path (2 motors, EXTI trigger). **Do not use it.**
`BALANCE/balance.c` is the **active** FreeRTOS-based path (4 motors, 6 vehicle types).
All new work goes in `balance.c`.

## File Encoding

Source files are UTF-8 (no BOM). Some header comments contain Chinese text.
If you see garbled characters, the file may still be in GBK — run `convert-encoding.ps1` to fix:
```powershell
powershell -ExecutionPolicy Bypass -File convert-encoding.ps1
```

## Code Style

- **Formatter**: clang-format config at `USER/.clang-format`
- **Base style**: Microsoft, 4-space indent, no tabs, Linux brace style
- **Naming**: snake_case for functions/variables, UPPER_CASE for macros/defines
- **Types**: `u8`, `u16`, `u32`, `s8`, `s16`, `s32` (STM32 stdperiph typedefs, not `<stdint.h>`)
- **Bilingual comments**: English + Chinese side-by-side is normal, do not remove either language

## Motor & Encoder Hardware Map

| Motor | PWM Pins | Timer | Encoder Timer | Encoder Pins |
|-------|----------|-------|---------------|--------------|
| A | PB8 (TIM10), PB9 (TIM11) | TIM10/11 | TIM2 | PA15, PB3 |
| B | PE5, PE6 | TIM9 | TIM3 | PB4, PB5 |
| C | PE11, PE9 | TIM1 | TIM4 | PB6, PB7 |
| D | PE14, PE13 | TIM1 | TIM5 | PA0, PA1 |

- PWM frequency: 10kHz (ARR=16799, PSC=0)
- Encoder mode: 4x (TIM_EncoderMode_TI12)
- Servo: TIM12 CH2, 100Hz, neutral=1500us

## Key Constants

- `CONTROL_FREQUENCY = 100` Hz
- `Velocity_KP = 300`, `Velocity_KI = 300` (incremental PI, no D term)
- PWM limit: ±16700
- 6 vehicle types selected by ADC potentiometer at boot (`Robot_Select()`)

## What NOT to Touch

- `FWLIB/` — vendor STM32 library
- `FreeRTOS/` — vendor RTOS kernel
- `CORE/` — ARM startup files
- `BALANCE/control.c` — deprecated, kept for reference only
