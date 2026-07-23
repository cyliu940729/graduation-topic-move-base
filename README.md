# STM32 Mecanum-Wheel Chassis Controller

This repository contains the baseline firmware for a four-wheel mecanum chassis based on the STM32F103ZET6.

The firmware currently provides:

- Four-wheel PWM motor control
- Four quadrature encoder inputs
- Closed-loop wheel-speed control
- Fixed-distance linear movement
- Fixed-angle rotation
- UART command reception
- FreeRTOS task scheduling
- UART debug output

## Project Information

| Item | Value |
|---|---|
| MCU | STM32F103ZET6 |
| IDE | STM32CubeIDE |
| STM32CubeMX | 6.9.2 |
| STM32CubeF1 | V1.8.6 |
| RTOS | FreeRTOS with CMSIS-RTOS V2 |
| System clock | 72 MHz |
| Control period | 10 ms |
| Control frequency | 100 Hz |
| Chassis type | Four-wheel mecanum chassis |

## System Role

The STM32 acts as the low-level controller for each vehicle.

```text
PC running ROS 2
    |
    | Wi-Fi / micro-ROS
    v
ESP32
    |
    | UART
    v
STM32F103ZET6
    |
    +-- Motor speed control
    +-- Encoder acquisition
    +-- Distance and rotation control
    +-- Chassis motion execution
```

The MPU6050 DMP driver is not integrated into this baseline version.

## Current Features

### Motor Control

TIM1 generates four PWM outputs:

| PWM channel | Motor |
|---|---|
| TIM1 CH1 | M1 |
| TIM1 CH2 | M2 |
| TIM1 CH3 | M3 |
| TIM1 CH4 | M4 |

Current PWM configuration:

```text
Timer prescaler: 0
Auto-reload value: 3599
PWM frequency: approximately 20 kHz
PWM output range: 0 to 100 percent
```

Each motor has an independent direction GPIO and supports:

- Forward rotation
- Reverse rotation
- Stop

### Encoders

| Motor | Encoder timer |
|---|---|
| M1 | TIM5 |
| M2 | TIM3 |
| M3 | TIM4 |
| M4 | TIM2 |

Current encoder and chassis parameters:

```text
ENCODER_CPR              = 61440
WHEEL_DIAMETER_CM        = 6.8
CAR_HALF_WHEELBASE_CM    = 5.5
CAR_HALF_TRACK_CM        = 9.5
ENCODER_DT               = 0.01
RPM_FILTER_ALPHA         = 0.25
```

Encoder direction correction:

```text
M1 = -1
M2 = +1
M3 = -1
M4 = +1
```

The encoder module currently provides:

- Accumulated encoder count
- Wheel angle
- Wheel RPM
- Wheel travel distance
- Chassis rotation angle estimated from wheel travel

### Wheel-Speed Control

The wheel-speed controller runs every 10 ms in `Run_Motor_Task`.

Current controller parameters:

| Motor | Kp | Ki | Kd |
|---|---:|---:|---:|
| M1 | 0.6 | 6.0 | 0.0 |
| M2 | 0.5 | 5.0 | 0.0 |
| M3 | 0.5 | 5.0 | 0.0 |
| M4 | 0.5 | 5.0 | 0.0 |

The controller includes:

- Derivative on measurement
- Derivative low-pass filter support
- Conditional-integration anti-windup
- PID reset when motor direction changes
- Derivative-state reset when the target speed changes
- PID reset when the motor stops

Because `Kd` is currently zero, the controller operates as a PI controller.

Current PWM limits:

```text
Minimum active PWM: 10 percent
Maximum PWM: 100 percent
```

## FreeRTOS Architecture

### RTOS Configuration

```text
Tick rate: 1000 Hz
Heap size: 16 KB
Preemption: enabled
Mutex support: enabled
Queue registry size: 8
```

The CubeMX-generated `defaultTask` is not used by the application.

### Run_Motor_Task

```text
Task name: run_motor
Priority: osPriorityAboveNormal
Stack size: 2048 bytes
Period: 10 ms
```

Responsibilities:

1. Start UART4 interrupt reception.
2. Receive complete commands from the message queue.
3. Parse movement direction and distance or angle.
4. Update all four encoders.
5. Calculate wheel distance and estimated chassis rotation.
6. Determine whether the requested movement is complete.
7. Run the four wheel-speed controllers.
8. Stop the motors and report completion.

### UartDebugTask

```text
Task name: uart_debug
Priority: osPriorityLow
Stack size: 1536 bytes
Trigger: thread flag
```

The task reports:

- Final distance of each wheel
- Received movement direction
- Requested distance or angle

### UART Command Queue

```text
Queue depth: 8
Message size: 32 bytes
```

The UART4 interrupt handler only assembles incoming frames and places complete frames into the RTOS message queue.

Command parsing and motor control are not performed inside the interrupt handler.

### UART5 Mutex

Task-level UART5 transmissions use a shared mutex to prevent debug messages from being interleaved.

## UART Communication

### UART4 Command Interface

UART4 currently receives movement commands from the external controller.

```text
Baud rate: 115200
Data bits: 8
Parity: none
Stop bits: 1
```

Pin assignment:

| STM32 pin | Function | Board label |
|---|---|---|
| PC10 | UART4 TX | ESP_RX |
| PC11 | UART4 RX | ESP_TX |

Command format:

```text
(direction,distance)
```

Examples:

```text
(1,30)
(5,90)
(7,0)
```

Frame rules:

- A frame starts with `(`.
- A frame ends with `)`.
- `direction` is an integer.
- `distance` is a floating-point value.
- UART4 receives one byte at a time by interrupt.
- A complete frame is passed to the RTOS message queue.

Current direction mapping:

| Direction | Wheel RPM signs | Current function |
|---:|---|---|
| 1 | `+ + + +` | Forward |
| 2 | `- - - -` | Backward |
| 3 | `+ - + -` | Lateral direction A |
| 4 | `- + - +` | Lateral direction B |
| 5 | `+ - - +` | Rotation direction A |
| 6 | `- + + -` | Rotation direction B |
| 7 | `0 0 0 0` | Immediate stop |

The exact left/right naming of directions 3 through 6 depends on the physical wheel and motor installation.

When a movement is complete, STM32 returns:

```text
1\r\n
```

### UART5 Debug Interface

```text
Baud rate: 115200
STM32 TX: PC12
STM32 RX: PD2
```

UART5 is currently used for:

- Boot messages
- RTOS initialization messages
- UART4 receive status
- Received UART frame output
- Command parsing errors
- Final wheel-distance output

UART5 is not used as the main ESP32 command interface in this baseline version.

## Current Motion-Control Flow

```text
Receive UART4 command
        |
        v
Parse direction and distance
        |
        v
Stop motors and reset encoders
        |
        v
Apply fixed target wheel speeds
        |
        v
Run four PI wheel-speed controllers at 100 Hz
        |
        v
Calculate average absolute wheel distance
        |
        +-- Translation: distance >= target distance
        |
        +-- Rotation: estimated angle >= target angle
        |
        v
Stop all motors immediately
        |
        v
Return completion message through UART4
```

The fixed movement speed is currently:

```text
DRIVE_SPEED_RPM = 30 RPM
```

## Movement Completion Logic

### Translation

Directions 1 through 4 use the average absolute wheel distance:

```text
(|M1| + |M2| + |M3| + |M4|) / 4
```

The chassis stops when the average wheel distance reaches or exceeds the requested distance.

### Rotation

Directions 5 and 6 use the average absolute wheel distance and the following turning radius:

```text
Turning radius = half wheelbase + half track width
```

The chassis stops when the estimated rotation reaches or exceeds the requested angle.

## Peripheral Configuration

### System Clock

```text
HSE: 8 MHz
PLL multiplier: 9
SYSCLK: 72 MHz
APB1: 36 MHz
APB2: 72 MHz
```

### I2C

| Peripheral | SCL | SDA | Intended use | Current status |
|---|---|---|---|---|
| I2C1 | PB6 | PB7 | LCD | Initialized but not used by application code |
| I2C2 | PB10 | PB11 | GY-87 / MPU6050 | Initialized but driver not integrated |

Both I2C peripherals are configured for 100 kHz.

### ADC1

| Pin | Label | Intended use |
|---|---|---|
| PA4 | B_V | Battery-voltage measurement |

ADC1 is initialized but is not currently used by the application.

### TIM8 Input Capture

| Pin | Label |
|---|---|
| PC6 | EC1 |
| PC7 | EC2 |
| PC8 | EC3 |
| PC9 | EC4 |

TIM8 channels 1 through 4 are configured for input capture but are not currently used by the application.

## Project Structure

```text
main/
|-- Core/
|   |-- Inc/
|   |-- Src/
|   `-- Startup/
|-- user_lib/
|   |-- motor.c
|   |-- motor.h
|   |-- encoder.c
|   `-- encoder.h
|-- Drivers/
|-- Middlewares/
|-- main.ioc
|-- STM32F103ZETX_FLASH.ld
`-- README.md
```

### `main.c`

`main.c` currently contains:

- Hardware initialization
- RTOS object creation
- UART4 frame reception
- UART command parsing
- Movement command setup
- Distance and angle completion checks
- Wheel-controller scheduling
- UART4 completion response
- UART5 debug output

### `user_lib/motor.c` and `user_lib/motor.h`

These files currently provide:

- Four-channel PWM startup
- Motor direction GPIO control
- PWM percentage output
- Four wheel PI/PID controllers
- Anti-windup handling
- Controller-state reset
- RPM and PWM state access

### `user_lib/encoder.c` and `user_lib/encoder.h`

These files currently provide:

- Four encoder timer inputs
- 16-bit counter overflow handling
- Encoder direction correction
- Accumulated count
- RPM calculation
- RPM low-pass filtering
- Wheel-angle conversion
- Wheel-distance conversion
- Chassis rotation estimation

## Build Instructions

1. Open STM32CubeIDE.
2. Import the project with `Existing Projects into Workspace`.
3. Select the project root directory.
4. Select the Debug or Release configuration.
5. Build the project.
6. Program the MCU using ST-LINK and SWD.

The linker configuration currently enables:

```text
-u _printf_float
-u _scanf_float
```

The Debug configuration also generates an Intel HEX file.

Expected build outputs:

```text
Debug/main.elf
Debug/main.hex
```

## Expected Boot Output

UART5 should print messages similar to:

```text
Init OK
RTOS 1: kernel init
RTOS 2: kernel init OK
RTOS 3: create queue
RTOS 4: queue OK
RTOS 5: create UART5 mutex
RTOS 6: UART5 mutex OK
RTOS 7: motor task OK
RTOS 8: debug task OK
RTOS 9: scheduler start
UART4 RX interrupt started
```

## Known Limitations

- Movement speed is fixed at 30 RPM.
- Target wheel speeds change immediately without acceleration limiting.
- No ramp acceleration or ramp deceleration is implemented.
- No braking-distance estimation is implemented.
- Motors stop immediately when the encoder threshold is reached.
- Movement completion does not check final wheel speed or settling time.
- Direction 7 is an immediate stop only.
- Translation distance is based on the average absolute distance of all four wheels.
- Independent mecanum forward and inverse kinematics modules are not present.
- Chassis velocity values `vx`, `vy`, and `wz` are not calculated.
- MPU6050 DMP is not integrated into the baseline project.
- IMU-based heading correction is not implemented.
- UART commands use an ASCII frame without CRC or sequence numbers.
- UART communication timeout handling is not implemented.
- Motion control and communication logic are concentrated in `main.c`.
