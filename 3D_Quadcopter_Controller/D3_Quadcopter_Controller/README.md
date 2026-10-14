# 6-DOF Quadcopter Control — LQR

## 1. Project Overview
    This project implements a **6-DOF quadcopter control system** using Linear Quadratic Regulator (LQR) control.

    The objective is to regulate the drone's position and orientation in three-dimensional space by controlling its translational and rotational dynamics.

    The LQR controller uses a linearized state-space model to calculate control inputs that minimize a quadratic cost function involving state error and control effort.


## 2. LQR Control Strategy
    The LQR controller uses the state-space representation:
    A — State matrix
    B — Input matrix

    The optimal feedback gain matrix K is obtained by minimizing the quadratic cost function:
    Where:
    Q— State-error weighting matrix
    R— Control-effort weighting matrix
    K— Optimal state-feedback gain matrix

### LQR Design Parameters
    - State matrix \(A\): Defined according to the selected linearized model.
    - Input matrix \(B\): Defines how control inputs affect the system states.
    - State weighting matrix \(Q\): ______
    - Control weighting matrix \(R\): ______
    - Optimal gain matrix \(K\): Computed using the LQR solution.

## 3. Thrust, Torque and Motor Mixing
    The control commands are converted into the required total thrust and body torques.

    The motor mixer distributes these commands among the four motors to generate the required translational and rotational motion.

    Motor commands are constrained by the configured actuator limits. Since standard LQR feedback does not inherently enforce actuator saturation, additional limiting and suitable handling of actuator constraints may be required.

## 4. Simulation and Visualization
    The simulation evaluates the drone's position, attitude over time.
    Graphical analysis and animation can be used to evaluate:
    - 3D trajectory tracking.
    - Target versus actual position.
    - Roll, pitch, and yaw responses.

    These results help assess tracking accuracy, stability, settling time, and control performance.

## 7. Project Goal
    To implement and analyze a **6-DOF quadcopter control system using LQR**, covering state-space modeling, linearization, optimal feedback gain calculation, position and attitude regulation, motor mixing, and three-dimensional simulation.