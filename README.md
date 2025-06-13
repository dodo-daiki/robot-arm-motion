# Robot Arm Library (Redesigned)

This is a redesigned modular library for robot arm kinematics.

## Structure

- `RobotArm` : Main interface class
- `ArmModel` : Stores physical parameters (link lengths)
- `IKSolver` : Inverse kinematics
- `FKSolver` : Forward kinematics
- `Pose`     : Pose type

## Usage

See `examples/basic_example/main.cpp`.

## Dependencies

- Eigen (header-only library)

