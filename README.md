# STM32 Mecanum-Wheel Chassis Controller

This repository contains the firmware for a four-wheel mecanum chassis based on the STM32F103ZET6.

The firmware uses FreeRTOS with CMSIS-RTOS V2 and has been refactored into independent communication, motion-control, wheel-control, mecanum-kinematics, hardware-driver, task, configuration, and debug modules.

## Current Version

| Version | Description |
|---|---|
| `v0.1.0-baseline` | Initial verified chassis firmware before architecture refactoring |
| `v0.2.0-refactored` | Verified modular chassis architecture with unchanged vehicle behavior |
| `main` | Current stable development branch |

The current firmware provides:

- Four-wheel PWM motor control
- Four quadrature encoder inputs
- Closed-loop wheel-speed control
- Fixed-distance linear movement
- Fixed-angle rotation
- Seven discrete chassis commands
- UART4 command reception and completion reporting
- UART5 debug output
- FreeRTOS task scheduling
- Independent mecanum direction mapping
- Centralized chassis configuration
- Modular motion-control architecture

## Refactoring Status

The chassis architecture refactoring was completed and verified on the physical vehicle.

The refactoring preserved:

- The original UART command format
- The original direction mapping
- The original 10 ms control period
- The original 30 RPM default movement speed
- The original distance and rotation completion behavior
- The UART4 completion response
- The UART5 encoder debug output

The main application file now focuses on hardware initialization and RTOS startup. Application responsibilities are separated into modules under `user_lib/`.

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
    +-- UART command handling
    +-- Chassis motion management
    +-- Mecanum wheel-target generation
    +-- Wheel-speed control
    +-- Encoder acquisition
    +-- Distance and rotation completion detection
```

The MPU6050 DMP driver is not integrated into the current firmware.

## Software Architecture

```text
main.c
    |
    +-- Hardware initialization
    +-- Module initialization
    +-- RTOS kernel startup
    |
    v
chassis_tasks
    |
    +-- Motor control task
    +-- Completion debug task
    +-- Module coordination
    |
    +------------------------+
    |                        |
    v                        v
communication           motion_control
                             |
                             v
                       wheel_control
                             |
                      +------+------+
                      |             |
                      v             v
             mecanum_kinematics   motor
                                    |
                                    v
                                  encoder

debug_console
    |
    +-- UART5 debug output
    +-- RTOS mutex protection

chassis_config
    |
    +-- Control period
    +-- Default RPM
    +-- Queue depth
    +-- Task stack sizes
    +-- Debug buffer size
```

### Module Responsibilities

| Module | Responsibility |
|---|---|
| `main.c` | Hardware initialization, module initialization, RTOS startup, and fatal error handling |
| `chassis_tasks` | RTOS task creation, periodic control scheduling, command dispatch, and completion reporting |
| `communication` | UART4 interrupt reception, frame assembly, internal message queue, command parsing, and UART4 text transmission |
| `motion_control` | Motion state, target distance or angle, encoder progress, completion detection, and immediate stop behavior |
| `mecanum_kinematics` | Conversion of discrete chassis direction commands into four signed wheel targets |
| `wheel_control` | Four-wheel target RPM storage, wheel PID execution, and coordinated motor stop |
| `motor` | PWM generation, direction GPIO control, and individual wheel PI/PID implementation |
| `encoder` | Encoder sampling, count accumulation, RPM filtering, wheel distance, and chassis-angle estimation |
| `debug_console` | UART5 boot and task-level debug output with mutex protection |
| `chassis_config` | Shared chassis control and RTOS configuration constants |

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

The wheel-speed controller runs every 10 ms from the motor control task implemented in `chassis_tasks.c`.

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

### Motor Control Task

```text
Task name: run_motor
Priority: osPriorityAboveNormal
Stack size: 2048 bytes
Period: 10 ms
```

Responsibilities:

1. Start UART4 interrupt reception.
2. Request a parsed command from the communication module.
3. Pass valid commands to the motion-control module.
4. Update the current motion state every 10 ms.
5. Run encoder updates and four wheel-speed controllers through the motion-control module.
6. Detect motion completion.
7. Return the completion response through the communication module.
8. Wake the debug task after a completed movement.

### Debug Task

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
Owner: communication module
```

The UART4 interrupt callback only assembles incoming frames and places complete frames into the communication module's internal RTOS message queue.

Command parsing and motor control are not performed inside the interrupt callback.

### UART5 Mutex

Task-level UART5 transmissions use the mutex owned by `debug_console.c` to prevent debug messages from being interleaved.

Boot messages are transmitted before the scheduler starts without using the mutex.

## UART Communication

### UART4 Command Interface

UART4 receives movement commands from the external controller and returns movement-completion responses.

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
- A complete frame is passed to the communication module's message queue.
- Parsing is performed in task context rather than interrupt context.

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

Direction 7 stops the current motion immediately and does not represent a completed distance or angle command.

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
- Invalid-command errors
- Final wheel-distance output

UART5 is not used as the main ESP32 command interface.

## Current Motion-Control Flow

```text
UART4 receives ASCII frame
        |
        v
communication assembles frame
        |
        v
communication queue stores frame
        |
        v
communication parses direction and target
        |
        v
chassis_tasks dispatches command
        |
        v
motion_control stops previous motion
and resets encoder state
        |
        v
mecanum_kinematics calculates
four signed wheel targets
        |
        v
wheel_control stores wheel targets
        |
        v
motion_control runs encoder updates
and four PI wheel-speed controllers at 100 Hz
        |
        v
motion_control calculates progress
        |
        +-- Translation: distance >= target distance
        |
        +-- Rotation: estimated angle >= target angle
        |
        v
wheel_control stops all motors
        |
        v
communication returns completion message
through UART4
        |
        v
debug task prints final wheel distances
through UART5
```

The default movement speed is configured in `chassis_config.h`:

```text
CHASSIS_DEFAULT_SPEED_RPM = 30 RPM
```

The control period and PID time step share the same configuration source:

```text
CHASSIS_CONTROL_PERIOD_MS = 10 ms
CHASSIS_CONTROL_DT_S      = 0.01 s
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
graduation-topic-move-base/
|-- Core/
|   |-- Inc/
|   |-- Src/
|   |   `-- main.c
|   `-- Startup/
|-- user_lib/
|   |-- chassis_config.h
|   |-- chassis_tasks.c
|   |-- chassis_tasks.h
|   |-- communication.c
|   |-- communication.h
|   |-- debug_console.c
|   |-- debug_console.h
|   |-- encoder.c
|   |-- encoder.h
|   |-- mecanum_kinematics.c
|   |-- mecanum_kinematics.h
|   |-- motion_control.c
|   |-- motion_control.h
|   |-- motor.c
|   |-- motor.h
|   |-- wheel_control.c
|   `-- wheel_control.h
|-- Drivers/
|-- Middlewares/
|-- main.ioc
|-- STM32F103ZETX_FLASH.ld
`-- README.md
```

### `Core/Src/main.c`

`main.c` is limited to:

- HAL and peripheral initialization
- Debug-console initialization
- Motor and encoder module initialization
- RTOS kernel initialization
- Chassis task initialization and creation
- Scheduler startup
- Fatal error handling

### `user_lib/chassis_tasks.c` and `user_lib/chassis_tasks.h`

These files provide:

- Creation of the motor control task
- Creation of the completion debug task
- Periodic 10 ms control scheduling
- Command dispatch between communication and motion control
- UART4 completion reporting through the communication module
- UART5 final-distance reporting through the debug console

### `user_lib/communication.c` and `user_lib/communication.h`

These files provide:

- UART4 interrupt reception
- ASCII frame assembly
- Internal RTOS message queue
- Command parsing
- UART4 text response transmission
- Optional debug logging callback

### `user_lib/motion_control.c` and `user_lib/motion_control.h`

These files provide:

- Current motion state
- Direction and target storage
- Encoder reset when a command starts
- Translation-distance progress calculation
- Rotation-angle progress calculation
- Completion detection
- Immediate-stop handling
- Periodic wheel-controller execution

### `user_lib/mecanum_kinematics.c` and `user_lib/mecanum_kinematics.h`

These files provide:

- Direction 1 through 6 mapping
- Four signed wheel-target values
- Separation of chassis movement logic from wheel PID execution

The current module implements discrete direction mapping. Continuous `vx`, `vy`, and `wz` forward or inverse kinematics are not implemented yet.

### `user_lib/wheel_control.c` and `user_lib/wheel_control.h`

These files provide:

- Four wheel target-RPM storage
- Mecanum target calculation requests
- Four-wheel speed PID execution
- Coordinated motor stop and target clearing

### `user_lib/debug_console.c` and `user_lib/debug_console.h`

These files provide:

- UART5 debug output
- Boot-time output before scheduler startup
- RTOS mutex creation
- Mutex-protected task-level transmission
- Text and binary output interfaces

### `user_lib/chassis_config.h`

This file provides shared settings for:

- UART command queue depth
- Motor control period
- PID time step
- Default movement RPM
- RTOS task stack sizes
- Debug-message buffer size

### `user_lib/motor.c` and `user_lib/motor.h`

These files provide:

- Four-channel PWM startup
- Motor direction GPIO control
- PWM percentage output
- Four wheel PI/PID controllers
- Anti-windup handling
- Controller-state reset
- RPM and PWM state access

### `user_lib/encoder.c` and `user_lib/encoder.h`

These files provide:

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
RTOS 3: communication init
RTOS 4: communication OK
RTOS 5: create debug console mutex
RTOS 6: debug console mutex OK
RTOS 7: motor task OK
RTOS 8: debug task OK
RTOS 9: scheduler start
UART4 RX interrupt started
```

## Basic Functional Test

After programming the MCU, test the following commands:

```text
(1,30)
(2,30)
(3,30)
(4,30)
(5,90)
(6,90)
(7,0)
```

Verify:

- Directions 1 through 6 match the physical chassis installation.
- Translation commands stop after the requested distance.
- Rotation commands stop after the requested angle.
- Direction 7 stops the current movement immediately.
- UART4 returns `1\r\n` after a completed movement.
- UART5 prints the final four wheel distances.
- A new command can replace a movement already in progress.
- Invalid frames do not stop the RTOS control task.

## Known Limitations

- Movement speed is fixed at 30 RPM for the current command interface.
- Target wheel speeds change immediately without acceleration limiting.
- No ramp acceleration or ramp deceleration is implemented.
- No braking-distance estimation is implemented.
- Motors stop immediately when the encoder threshold is reached.
- Movement completion does not check final wheel speed or settling time.
- Direction 7 is an immediate stop only.
- Translation distance is based on the average absolute distance of all four wheels.
- The mecanum module currently supports discrete direction mapping only.
- Continuous mecanum `vx`, `vy`, and `wz` conversion is not implemented.
- Odometry is not implemented.
- MPU6050 DMP is not integrated.
- IMU-based heading correction is not implemented.
- UART commands use an ASCII frame without CRC or sequence numbers.
- Protocol-level command timeout handling is not implemented.

## Version History

### `v0.2.0-refactored`

- Separated UART communication from `main.c`.
- Added an internal communication message queue.
- Added a dedicated debug-console module.
- Added a dedicated chassis-task module.
- Added a motion-control state module.
- Added a wheel-control module.
- Added an independent mecanum direction-mapping module.
- Added centralized chassis configuration.
- Reduced `main.c` to initialization and RTOS startup responsibilities.
- Preserved the original command format and vehicle behavior.
- Verified the refactored firmware on the physical chassis.

### `v0.1.0-baseline`

- Imported the initial verified STM32 mecanum chassis firmware.
- Preserved the original single-file application-control structure.

## Planned Development

Future work may include:

- Trapezoidal acceleration and deceleration
- Improved stop and settling criteria
- Braking-distance estimation
- Continuous `vx`, `vy`, and `wz` command support
- ROS 2 or micro-ROS velocity-command integration
- Wheel odometry
- MPU6050 DMP integration
- IMU-based heading correction
- More robust UART framing with sequence numbers and CRC
