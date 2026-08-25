# Changelog

All notable changes to this project are documented in this file.

The project currently uses descriptive Git tags for verified development milestones.

## Unreleased

### Added

- Added trapezoidal and triangular motion-speed profiles.
- Added acceleration and deceleration limits for chassis movement.
- Added braking-distance-based automatic deceleration.
- Added wheel RPM and linear-speed conversion helpers.
- Added wheel-distance conversion for rotation commands.

### Changed

- Simplified motor and mecanum control using lookup tables.
- Invalid motor device numbers are now ignored without changing existing motor outputs.
- Normal chassis command behavior remains unchanged.
- Translation and rotation commands now accelerate and decelerate progressively instead of using an immediate fixed-speed step.
- Encoder timer input filtering is set to reduce noise-induced count errors.
- Direction 7 remains an immediate-stop command and bypasses the motion profile.

### Documentation

- Updated `README.md` to describe the modular chassis architecture.
- Added `docs/ARCHITECTURE.md`.
- Added this changelog.
- Documented module responsibilities, dependency direction, command flow, and current limitations.

## v0.2.0-refactored

### Added

- Added `user_lib/communication.c` and `user_lib/communication.h`.
  - UART4 interrupt reception
  - Command-frame assembly
  - Internal RTOS message queue
  - `(direction,distance)` command parsing
  - UART4 completion-response transmission
  - Optional debug logging callback

- Added `user_lib/debug_console.c` and `user_lib/debug_console.h`.
  - UART5 boot and runtime debug output
  - RTOS mutex protection
  - Text and binary transmission interfaces

- Added `user_lib/mecanum_kinematics.c` and `user_lib/mecanum_kinematics.h`.
  - Direction commands 1 through 6
  - Four signed wheel-target calculations
  - Separation of chassis direction mapping from PID and motor hardware

- Added `user_lib/wheel_control.c` and `user_lib/wheel_control.h`.
  - Four-wheel target RPM storage
  - Four-wheel speed PID execution
  - Coordinated motor stop
  - Wheel-target clearing

- Added `user_lib/motion_control.c` and `user_lib/motion_control.h`.
  - Motion-state management
  - Target distance and target angle storage
  - Encoder reset when a new movement starts
  - Translation-distance progress calculation
  - Rotation-angle progress calculation
  - Completion detection
  - Immediate-stop handling

- Added `user_lib/chassis_tasks.c` and `user_lib/chassis_tasks.h`.
  - Motor control RTOS task
  - Completion debug RTOS task
  - Periodic 10 ms control scheduling
  - Coordination between communication and motion-control modules

- Added `user_lib/chassis_config.h`.
  - UART command queue depth
  - Control-loop period
  - PID time step
  - Default movement RPM
  - RTOS task stack sizes
  - Debug-message buffer size

### Changed

- Reduced `Core/Src/main.c` to:
  - HAL and peripheral initialization
  - Module initialization
  - CMSIS-RTOS V2 kernel startup
  - Chassis task creation
  - Scheduler startup
  - Fatal error handling

- Moved UART4 frame reception and parsing out of `main.c`.
- Moved four-wheel direction mapping out of `main.c`.
- Moved wheel target RPM storage and PID scheduling out of `main.c`.
- Moved distance and rotation completion logic out of `main.c`.
- Moved UART5 mutex and debug output handling out of `main.c`.
- Moved RTOS task definitions and message queue creation out of `main.c`.
- Moved UART4 message queue ownership into the communication module.
- Moved UART4 completion transmission into the communication module.
- Centralized shared chassis-control parameters.

### Preserved

The refactoring intentionally preserved the existing external behavior:

- UART command format remains `(direction,distance)`.
- Directions 1 through 7 retain their original behavior.
- Default movement speed remains 30 RPM.
- Control-loop period remains 10 ms.
- Translation completion remains based on average absolute wheel distance.
- Rotation completion remains based on estimated chassis angle.
- Completed movements still return `1\r\n` through UART4.
- UART5 still reports boot status, errors, and final encoder distances.
- A new command can replace a movement already in progress.
- Direction 7 remains an immediate-stop command.

### Verified

- Project builds successfully in STM32CubeIDE.
- UART4 command reception was verified.
- UART5 debug output was verified.
- Directions 1 through 6 were verified on the physical mecanum chassis.
- Translation-distance stopping was verified.
- Rotation-angle stopping was verified.
- Immediate stop was verified.
- Completion response was verified.
- Final encoder debug output was verified.

## v0.1.0-baseline

### Added

- Initial verified STM32F103ZET6 mecanum chassis firmware.
- Four TIM1 PWM motor outputs.
- Four quadrature encoder timer inputs.
- Four independent wheel PI/PID controllers.
- Fixed-distance translation commands.
- Fixed-angle rotation commands.
- UART4 ASCII command reception.
- UART4 completion response.
- UART5 debug output.
- FreeRTOS with CMSIS-RTOS V2 scheduling.

### Architecture

The baseline version concentrated most application-level responsibilities in `Core/Src/main.c`, including:

- UART4 frame handling
- Command parsing
- Direction-to-wheel mapping
- Motion-state storage
- Distance and angle completion checks
- Four-wheel PID scheduling
- Completion reporting
- UART5 debug handling

This version is retained as the known-good pre-refactoring reference.
