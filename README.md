# STM32 Mecanum-Wheel Chassis Controller

Firmware for a four-wheel mecanum chassis based on the STM32F103ZET6.

The project uses FreeRTOS through CMSIS-RTOS V2 and separates communication, motion control, wheel control, mecanum direction mapping, encoder acquisition, IMU heading estimation, battery monitoring, ultrasonic obstacle detection, task scheduling, and debug output into independent modules.

## Current Status

The current `main` branch has been verified on the physical chassis and includes:

- Four-wheel PWM motor control
- Four quadrature encoders
- Closed-loop wheel-speed PI control
- Fixed-distance translation
- Fixed-angle rotation
- MPU6050 DMP-based relative chassis heading estimation
- Encoder fallback for rotation when IMU data is unavailable
- Battery-percentage monitoring through ADC1
- Two HC-SR04 ultrasonic sensors using TIM8 input capture interrupts
- Obstacle-triggered emergency stop with hysteresis/latching
- UART4 command and status interface
- UART5 debug output
- FreeRTOS task scheduling

Latest functional tag: `v0.5.0-ultrasonic`.

## Important IMU Note

The IMU integration is complete for relative chassis rotation-angle measurement.

The firmware does **not** use the MPU6050 native yaw value as the primary rotation reference. Because the sensor is mounted so that sensor X corresponds to the chassis vertical rotation axis, `imu.c` calculates incremental quaternion rotation around sensor X and accumulates it as `imu_heading_deg`.

The current IMU flow is:

```text
MPU6050 DMP quaternion
        |
        v
relative quaternion between consecutive samples
        |
        v
extract twist around sensor X
        |
        v
gyro-X bias compensation + deadband
        |
        v
accumulate relative chassis heading
```

Current heading-related settings:

```text
Gyro scale:            16.4 LSB/(deg/s)
Calibration samples:   200
Gyro deadband:         0.3 deg/s
IMU task period:       10 ms
IMU health timeout:    100 ms
```

At the current 10 ms IMU task period, the 200-sample stationary calibration takes approximately 2 seconds.

`IMU_ResetHeading()` resets the relative heading to zero at the start of a rotation command.

For direction 5 and direction 6:

1. If the IMU is healthy when the movement starts, the relative heading is reset and the IMU becomes the primary rotation-angle source.
2. Rotation progress uses the absolute value of `IMU_GetHeadingDeg()`.
3. If IMU updates become unhealthy during that movement, the controller permanently switches to encoder-based angle estimation for the remainder of the command.
4. If the IMU is unavailable when the movement starts, encoder-based angle estimation is used from the beginning.

This means the firmware currently provides **relative rotation-angle estimation for turning**, not an absolute compass heading and not closed-loop heading correction during straight-line movement.

## Project Information

| Item | Value |
|---|---|
| MCU | STM32F103ZET6 |
| IDE | STM32CubeIDE |
| STM32CubeMX | 6.9.2 |
| STM32CubeF1 | V1.8.6 |
| RTOS | FreeRTOS with CMSIS-RTOS V2 |
| System clock | 72 MHz |
| Motor control period | 10 ms |
| Motor control frequency | 100 Hz |
| Default movement speed | 30 RPM |
| Chassis | Four-wheel mecanum |
| IMU | GY-87 board / MPU6050 DMP |
| Ultrasonic | HC-SR04 ×2 |

## System Role

```text
PC / ROS 2
    |
    | Wi-Fi / upper-level communication
    v
ESP32
    |
    | UART4
    v
STM32F103ZET6
    |
    +-- UART command handling
    +-- Chassis motion management
    +-- Mecanum wheel-target generation
    +-- Four-wheel speed control
    +-- Encoder acquisition
    +-- MPU6050 relative heading estimation
    +-- Battery monitoring
    +-- Ultrasonic obstacle detection
    +-- Motion completion / emergency-stop reporting
```

## Software Architecture

```text
main.c
    |
    +-- Hardware initialization
    +-- Module initialization
    +-- RTOS startup
    |
    v
chassis_tasks
    |
    +-- Motor task
    +-- IMU task
    +-- Ultrasonic task
    +-- Debug task
    |
    +---------------- communication
    |
    +---------------- motion_control
    |                     |
    |                     +-- wheel_control
    |                     |      |
    |                     |      +-- mecanum_kinematics
    |                     |      +-- motor
    |                     |
    |                     +-- encoder
    |                     +-- imu
    |
    +---------------- ultrasonic
    +---------------- power_monitor

debug_console
    +-- UART5 debug output
    +-- RTOS mutex protection
```

### Module Responsibilities

| Module | Responsibility |
|---|---|
| `main.c` | Peripheral initialization, module initialization, RTOS startup, fatal error handling |
| `chassis_tasks` | Task creation, command dispatch, periodic control, IMU update, ultrasonic monitoring, status reporting |
| `communication` | UART4 interrupt reception, frame assembly, RTOS queue, command parsing, UART4 transmission |
| `motion_control` | Motion state, target distance/angle, completion detection, IMU/encoder rotation source selection, stop behavior |
| `mecanum_kinematics` | Discrete direction command to four signed wheel-RPM targets |
| `wheel_control` | Wheel target storage, four-wheel PI execution, coordinated stop |
| `motor` | TIM1 PWM, direction GPIO, individual wheel PI/PID controller |
| `encoder` | Encoder sampling, RPM filtering, distance, encoder-based chassis-angle estimation |
| `mpu6050_dmp` | MPU6050 DMP firmware loading, FIFO access, quaternion/gyro/acceleration data extraction |
| `imu` | DMP update, startup gyro calibration, relative heading calculation, health monitoring |
| `power_monitor` | ADC battery-voltage measurement and battery-percentage conversion |
| `ultrasonic` | HC-SR04 trigger control and TIM8 input-capture pulse-width measurement |
| `debug_console` | UART5 debug transmission with task-level mutex protection |
| `chassis_config` | Shared chassis, RTOS, ultrasonic, and control constants |

## Motor and Encoder Control

TIM1 generates four motor PWM channels:

| PWM channel | Motor |
|---|---|
| TIM1 CH1 | M1 |
| TIM1 CH2 | M2 |
| TIM1 CH3 | M3 |
| TIM1 CH4 | M4 |

PWM frequency is approximately 20 kHz with ARR = 3599.

Encoder mapping:

| Motor | Timer |
|---|---|
| M1 | TIM5 |
| M2 | TIM3 |
| M3 | TIM4 |
| M4 | TIM2 |

Encoder parameters:

```text
ENCODER_CPR           = 61440
WHEEL_DIAMETER_CM     = 6.8
CAR_HALF_WHEELBASE_CM = 5.5
CAR_HALF_TRACK_CM     = 9.5
ENCODER_DT            = 0.01 s
RPM_FILTER_ALPHA      = 0.25
```

Current wheel controller parameters:

| Motor | Kp | Ki | Kd |
|---|---:|---:|---:|
| M1 | 0.6 | 6.0 | 0.0 |
| M2 | 0.5 | 5.0 | 0.0 |
| M3 | 0.5 | 5.0 | 0.0 |
| M4 | 0.5 | 5.0 | 0.0 |

Because `Kd` is currently zero, the controller operates as PI control. The PID implementation still contains derivative filtering and anti-windup support.

## Motion Commands

UART4 command format:

```text
(direction,distance)
```

Examples:

```text
(1,30)
(5,90)
(7,0)
```

Direction mapping:

| Direction | Wheel signs | Function | Completion source |
|---:|---|---|---|
| 1 | `+ + + +` | Forward | Average absolute wheel distance |
| 2 | `- - - -` | Backward | Average absolute wheel distance |
| 3 | `+ - + -` | Lateral A | Average absolute wheel distance |
| 4 | `- + - +` | Lateral B | Average absolute wheel distance |
| 5 | `+ - - +` | Rotate A | IMU relative heading, encoder fallback |
| 6 | `- + + -` | Rotate B | IMU relative heading, encoder fallback |
| 7 | `0 0 0 0` | Immediate stop | N/A |

Directions 1 through 4 treat the second command value as centimeters.

Directions 5 and 6 treat the second command value as degrees.

Direction 7 stops the current movement immediately.

## UART4 Status Responses

UART4 uses status plus battery percentage:

| Response | Meaning |
|---|---|
| `0,<battery>` | IMU startup calibration completed and system is ready |
| `1,<battery>` | Requested movement completed normally |
| `2,<battery>` | Ultrasonic obstacle event stopped the current movement |

Examples:

```text
0,87
1,86
2,86
```

If battery measurement fails, the corresponding response uses `ERR`, for example `1,ERR`.

## Battery Monitoring

ADC1 monitors the battery through PA4 (`B_V`).

Current divider:

```text
R_TOP    = 27 kOhm
R_BOTTOM = 10 kOhm
```

Battery percentage is linearly mapped and clamped:

```text
12.6 V -> 100%
11.6 V ->   0%
```

Battery percentage is included in UART4 ready, completion, and obstacle responses.

## Ultrasonic Obstacle Detection

The application currently uses ultrasonic connector 1 and connector 2.

| Logical sensor | Trigger | Echo | TIM8 channel |
|---|---|---|---|
| Sensor 1 | TR1 / PA11 | EC1 / PC6 | CH1 |
| Sensor 2 | TR2 / PA10 | EC2 / PC7 | CH2 |

TIM8 is configured as a 1 MHz counter, so one timer count corresponds to approximately 1 microsecond.

Echo pulse edges are captured with TIM8 capture/compare interrupts rather than GPIO polling.

Obstacle thresholds:

| Sensor | Stop threshold | Clear threshold |
|---|---:|---:|
| Sensor 1 | < 10 cm | >= 12 cm |
| Sensor 2 | < 5 cm | >= 7 cm |

The two sensors are sampled sequentially with a 60 ms gap to reduce ultrasonic cross-talk.

When either stop threshold is crossed while the chassis is moving:

1. The ultrasonic task raises a stop request.
2. The motor task executes `MotionControl_Stop()`.
3. UART4 returns `2,<battery>` once.
4. The obstacle event remains latched while the obstacle is still close.
5. The latch clears only after both sensors exceed their clear thresholds.
6. A later approach can generate a new obstacle event.

This hysteresis prevents repeated stop/status events when the measured distance fluctuates around a threshold.

## FreeRTOS Tasks

| Task | Priority | Stack | Main responsibility |
|---|---|---:|---|
| `run_motor` | AboveNormal | 2048 B | UART command dispatch, 10 ms motion update, wheel control, stop handling |
| `imu` | Normal | 2048 B | MPU6050 DMP update and relative heading calculation at 10 ms period |
| `ultrasonic` | Low | 1536 B | Alternating HC-SR04 measurement and obstacle-state logic |
| `uart_debug` | Low | 1536 B | Final motion debug output |

UART5 task-level output is protected by the mutex in `debug_console.c`.

## Current Motion-Control Flow

```text
UART4 command
    |
    v
communication queue / parser
    |
    v
chassis_tasks
    |
    v
motion_control
    |
    +-- directions 1..4
    |      encoder distance -> completion
    |
    +-- directions 5..6
    |      IMU healthy?
    |        |
    |        +-- yes -> reset relative heading
    |        |          use IMU heading magnitude
    |        |
    |        +-- no  -> encoder angle fallback
    |
    +-- direction 7
           immediate stop

Ultrasonic task
    |
    +-- obstacle -> motor-task stop request -> 2,battery
```

## Peripheral Summary

| Peripheral | Pins / channels | Current use |
|---|---|---|
| TIM1 | CH1-CH4 | Four motor PWM outputs |
| TIM2/TIM3/TIM4/TIM5 | Encoder inputs | Four wheel encoders |
| I2C1 | PB6/PB7 | Initialized; application use not currently defined |
| I2C2 | PB10/PB11 | MPU6050 DMP |
| UART4 | PC10/PC11 | ESP32 command/status interface |
| UART5 | PC12/PD2 | Debug console |
| ADC1 | PA4 | Battery monitoring |
| TIM8 CH1 | PC6 | Ultrasonic sensor 1 echo |
| TIM8 CH2 | PC7 | Ultrasonic sensor 2 echo |
| TIM8 CH3 | PC8 | Configured but not used by current application logic |
| TIM8 CH4 | PC9 | Configured but not used by current application logic |

## Project Structure

```text
graduation-topic-move-base/
|-- Core/
|   |-- Inc/
|   |-- Src/
|   `-- Startup/
|-- user_lib/
|   |-- chassis_config.h
|   |-- chassis_tasks.c/.h
|   |-- communication.c/.h
|   |-- debug_console.c/.h
|   |-- encoder.c/.h
|   |-- imu.c/.h
|   |-- mecanum_kinematics.c/.h
|   |-- motion_control.c/.h
|   |-- motor.c/.h
|   |-- mpu6050_dmp.c/.h
|   |-- power_monitor.c/.h
|   |-- ultrasonic.c/.h
|   `-- wheel_control.c/.h
|-- Drivers/
|-- Middlewares/
|-- main.ioc
|-- STM32F103ZETX_FLASH.ld
`-- README.md
```

## Build

1. Open STM32CubeIDE.
2. Import the repository as an existing project.
3. Select Debug or Release.
4. Build the firmware.
5. Program the STM32 using ST-LINK/SWD.

The linker configuration enables floating-point `printf`/`scanf` support.

## Basic Functional Test

After programming, first keep the chassis stationary while the IMU performs startup calibration.

Expected UART4 ready response after calibration:

```text
0,<battery>
```

Then verify:

```text
(1,30)
(2,30)
(3,30)
(4,30)
(5,90)
(6,90)
(7,0)
```

Check that:

- Directions 1 through 4 stop at the requested distance.
- Directions 5 and 6 stop at the requested angle using IMU heading when healthy.
- Rotation falls back to encoder estimation if IMU data becomes unavailable.
- Direction 7 immediately stops the chassis.
- Normal completion returns `1,<battery>`.
- Sensor 1 below 10 cm or sensor 2 below 5 cm stops a moving chassis and returns `2,<battery>` once.
- Moving back into the clear region resets the ultrasonic latch so a later obstacle can trigger again.
- UART5 continues to provide debug output.

## Known Limitations

- Movement speed is fixed at 30 RPM through the current command interface.
- Target wheel speed changes immediately; the trapezoidal motion-profile experiment was reverted from `main`.
- No braking-distance or final-settling model is implemented.
- Continuous mecanum `vx`, `vy`, and `wz` commands are not implemented.
- Wheel odometry is not implemented.
- IMU heading is relative and may drift over long periods.
- No magnetometer-based absolute heading correction is currently used.
- IMU heading is used for rotation completion, not straight-line heading correction.
- Ultrasonic obstacle logic currently evaluates the obstacle state only when both sensor reads are successful.
- UART framing has no CRC, sequence number, or protocol-level timeout.

## Version History

### `v0.5.0-ultrasonic`

- Added dual HC-SR04 measurement.
- Replaced GPIO polling with TIM8 input-capture interrupts.
- Added asymmetric obstacle thresholds and clear thresholds.
- Added obstacle latch/hysteresis.
- Added obstacle-triggered stop and UART4 `2,<battery>` response.

### `v0.4.0-imu`

- Integrated MPU6050 DMP.
- Added startup gyro-X bias calibration.
- Added relative chassis heading calculation around the sensor X axis.
- Added IMU health monitoring.
- Added IMU-primary rotation completion with encoder fallback.
- Added UART4 ready response after IMU calibration.

### Battery Monitoring

- Added ADC1 battery-voltage measurement.
- Added 27 kOhm / 10 kOhm divider conversion.
- Added 11.6 V to 12.6 V battery-percentage mapping.
- Added battery percentage to UART4 status responses.

### `v0.3.0-motion-profile`

- Experimental trapezoidal motion-profile implementation.
- The feature was later reverted from `main`; the current firmware uses immediate fixed-RPM targets.

### `v0.2.0-refactored`

- Split communication, motion control, wheel control, mecanum mapping, task management, configuration, and debug responsibilities into separate modules.
- Preserved the original verified chassis behavior.

### `v0.1.0-baseline`

- Initial verified chassis firmware before architecture refactoring.

## Planned Development

Potential future work:

- Continuous `vx`, `vy`, and `wz` commands
- ROS 2 / micro-ROS velocity-command integration
- Wheel odometry
- Optional absolute-heading source or magnetometer fusion
- Straight-line IMU heading correction
- Improved braking and settling criteria
- More robust ultrasonic fault handling
- UART framing with sequence numbers and CRC
