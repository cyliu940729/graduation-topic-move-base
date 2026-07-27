# Chassis Firmware Architecture

## Overview

The firmware is organized as a layered STM32 chassis-control system.

The refactoring separates communication, task scheduling, motion control, mecanum direction mapping, wheel control, hardware drivers, configuration, and debug output.

The UART command format and physical vehicle behavior remain unchanged from `v0.1.0-baseline`.

## Architecture Diagram

```text
main.c
    |
    +-- Peripheral initialization
    +-- Module initialization
    +-- RTOS kernel startup
    |
    v
chassis_tasks
    |
    +-----------------------+
    |                       |
    v                       v
communication          motion_control
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
    +-- Shared control parameters
    +-- Queue and task settings
```

## Module Responsibilities

### `main.c`

Responsibilities:

- Initialize HAL
- Configure the system clock
- Initialize STM32 peripherals
- Initialize chassis modules
- Initialize the CMSIS-RTOS V2 kernel
- Create chassis tasks
- Start the RTOS scheduler
- Stop motors during fatal errors

`main.c` does not contain command parsing, motion completion logic, mecanum direction mapping, or wheel PID scheduling.

### `chassis_tasks`

Responsibilities:

- Create the motor control task
- Create the UART debug task
- Run the periodic 10 ms control loop
- Request parsed commands from `communication`
- Pass commands to `motion_control`
- Detect completed movements
- Send completion responses
- Trigger final encoder debug output

### `communication`

Responsibilities:

- Receive UART4 bytes using interrupts
- Detect the start and end of command frames
- Store complete frames in an internal RTOS message queue
- Parse `(direction,distance)` commands
- Send completion responses through UART4
- Report communication events through an optional logging callback

The interrupt callback only performs frame assembly and queue insertion. Parsing is performed in task context.

### `motion_control`

Responsibilities:

- Store the current movement direction
- Store the target distance or rotation angle
- Reset encoder state when a new movement starts
- Update encoder data
- Calculate translation progress
- Calculate rotation progress
- Detect movement completion
- Execute immediate stop commands
- Request periodic wheel PID updates

### `mecanum_kinematics`

Responsibilities:

- Convert direction commands 1 through 6 into four signed wheel targets
- Keep mecanum movement mapping independent from PID and motor hardware

The current implementation supports discrete directions only.

Continuous `vx`, `vy`, and `wz` conversion is planned for future development.

### `wheel_control`

Responsibilities:

- Store four target wheel RPM values
- Request target calculation from `mecanum_kinematics`
- Run the four wheel-speed PID controllers
- Stop all motors
- Clear wheel targets

### `motor`

Responsibilities:

- Start TIM1 PWM channels
- Control individual motor direction GPIOs
- Set motor PWM percentages
- Execute individual wheel PI/PID controllers
- Apply output limits and anti-windup
- Reset controller state

### `encoder`

Responsibilities:

- Read four encoder timers
- Handle 16-bit timer counter wraparound
- Apply encoder direction correction
- Calculate accumulated counts
- Calculate wheel RPM
- Apply RPM low-pass filtering
- Calculate wheel distance
- Estimate chassis rotation angle

### `debug_console`

Responsibilities:

- Send boot and runtime messages through UART5
- Create and manage the UART5 transmission mutex
- Protect task-level UART5 transmissions
- Provide text and binary output functions

### `chassis_config`

Responsibilities:

- Define the command queue depth
- Define the 10 ms control period
- Derive the PID time step
- Define the default movement RPM
- Define RTOS task stack sizes
- Define the debug-message buffer size

## Command Flow

```text
UART4 receives byte
        |
        v
communication assembles frame
        |
        v
internal message queue
        |
        v
communication parses command
        |
        v
chassis_tasks receives command
        |
        v
motion_control starts movement
        |
        v
mecanum_kinematics generates wheel targets
        |
        v
wheel_control stores wheel targets
        |
        v
motor PID drives four wheels
```

## Periodic Control Flow

The motor control task runs every 10 ms:

```text
Communication_GetCommand()
        |
        v
MotionControl_Start() when a command is ready
        |
        v
MotionControl_Update()
        |
        +-- Encoder_Update_All()
        +-- Calculate distance and angle
        +-- Check completion
        +-- WheelControl_RunSpeedPID()
```

When the target is reached:

```text
WheelControl_Stop()
        |
        v
Communication_SendText("1\r\n")
        |
        v
Wake debug task
        |
        v
Print final wheel distances through UART5
```

## Dependency Rules

The intended dependency direction is:

```text
chassis_tasks
    -> communication
    -> motion_control

motion_control
    -> wheel_control
    -> encoder

wheel_control
    -> mecanum_kinematics
    -> motor
```

Lower-level modules must not depend on higher-level task or application modules.

Examples:

- `motor` must not depend on `motion_control`.
- `mecanum_kinematics` must not access UART or encoder hardware.
- `communication` must not directly control motors.
- `main.c` must not contain chassis movement algorithms.

## Current Limitations

- Movement speed is fixed for the current UART command interface.
- Acceleration and deceleration ramps are not implemented.
- Completion detection uses encoder thresholds only.
- Final wheel speed and settling time are not checked.
- Continuous mecanum velocity commands are not implemented.
- Odometry is not implemented.
- IMU heading correction is not implemented.
- UART frames do not include CRC or sequence numbers.

## Version Reference

### `v0.1.0-baseline`

Verified firmware before architecture refactoring.

### `v0.2.0-refactored`

Verified modular architecture with unchanged UART command format and physical chassis behavior.
