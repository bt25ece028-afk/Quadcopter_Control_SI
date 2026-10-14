# Quadcopter Control

## Objective
The primary objective of this repository is to implement and compare autonomous control systems for a quadcopter across **1D, 2D, and 3D space**. The framework focuses on precise trajectory tracking and position following while providing a benchmark for handling external disturbances, unmodeled dynamics, and stability requirements.

---

## Making the Plant

### Aim
The plant dynamics model calculates the next state vector of the quadcopter given its current state and individual **motor angular speeds** ($\omega_1, \omega_2, \omega_3, \omega_4$).

$$\dot{x} = f(x, \omega)$$

* **State Vector ($x$):** Positions ($x, y, z$), translational velocities ($\dot{x}, \dot{y}, \dot{z}$), Euler angles ($\phi, \theta, \psi$), and body angular velocities ($p, q, r$).
* **Control Input Vector ($\omega$):** Motor rotational speeds $\omega = [\omega_1, \omega_2, \omega_3, \omega_4]^T$.

> **Constant Parameters & System Equations**
> 
> ![Quadcopter Parameters & Formulas](path/to/12_parameters_and_formulas.png)
> 
> *Figure 1: Physical parameters (mass, moments of inertia, arm lengths, thrust/drag coefficients) and equations of motion used in the plant model.*

### Simulation Demos
![Plant Simulation GIF](path/to/plant_simulation.gif)

---

## Making the Controller

### 1. PID Controller (Proportional-Integral-Derivative)

#### Theory
The PID controller uses feedback error $e(t) = r(t) - y(t)$ to generate the required control input without requiring an explicit physical model of the quadcopter:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(t') \, dt' + K_d \frac{de(t)}{dt}$$

* **Proportional ($K_p$):** Produces a control output proportional to the current error. Increases system responsiveness and reduces rise time, but high values can cause overshoots and instability.
* **Integral ($K_i$):** Accumulates past errors over time to eliminate **steady-state error**.
* **Derivative ($K_d$):** Responds to the rate of error change. Acts as a damper to reduce overshoot, improve transient response, and optimize **settling time**.

> **Block Diagram**
> 
> ![PID Controller Block Diagram](path/to/pid_block_diagram.png)
> 
> *Figure 2: Closed-loop PID control architecture for quadcopter position and attitude tracking.*

#### Results & Visualizations

##### 1D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 1D Plot 1](path/to/pid_1d_plot1.png) | ![PID 1D Simulation GIF 1](path/to/pid_1d_simulation1.gif) |
| ![PID 1D Plot 2](path/to/pid_1d_plot2.png) | ![PID 1D Simulation GIF 2](path/to/pid_1d_simulation2.gif) |

##### 2D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 2D Plot 1](path/to/pid_2d_plot1.png) | ![PID 2D Simulation GIF 1](path/to/pid_2d_simulation1.gif) |
| ![PID 2D Plot 2](path/to/pid_2d_plot2.png) | ![PID 2D Simulation GIF 2](path/to/pid_2d_simulation2.gif) |

##### 3D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 3D Plot 1](path/to/pid_3d_plot1.png) | ![PID 3D Simulation GIF 1](path/to/pid_3d_simulation1.gif) |
| ![PID 3D Plot 2](path/to/pid_3d_plot2.png) | ![PID 3D Simulation GIF 2](path/to/pid_3d_simulation2.gif) |

---

### 2. LQR Controller (Linear Quadratic Regulator)

#### Theory
LQR is an optimal control strategy designed for state-space representations ($\dot{x} = Ax + Bu$). It computes an optimal state-feedback gain matrix $K$ that minimizes a continuous-time quadratic cost function:

$$J = \int_{0}^{\infty} \left( x^T Q x + u^T R u \right) dt$$

Where:
* $Q \ge 0$ is the positive semi-definite state weighting matrix (penalizes tracking errors).
* $R > 0$ is the positive definite control effort weighting matrix (penalizes excessive actuator input).

The optimal control law is given by:

$$u = -Kx$$

The state-feedback gain matrix $K$ is computed as:

$$K = R^{-1} B^T P$$

Where $P$ is the unique positive-definite solution to the **Continuous Algebraic Riccati Equation (CARE)**:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

> **Block Diagram**
> 
> ![LQR Controller Block Diagram](path/to/lqr_block_diagram.png)
> 
> *Figure 3: Optimal state-feedback LQR control architecture.*

#### Results & Visualizations

##### 1D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![LQR 1D Plot 1](path/to/lqr_1d_plot1.png) | ![LQR 1D Simulation GIF 1](path/to/lqr_1d_simulation1.gif) |
| ![LQR 1D Plot 2](path/to/lqr_1d_plot2.png) | ![LQR 1D Simulation GIF 2](path/to/lqr_1d_simulation2.gif) |

##### 2D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![LQR 2D Plot 1](path/to/lqr_2d_plot1.png) | ![LQR 2D Simulation GIF 1](path/to/lqr_2d_simulation1.gif) |
| ![LQR 2D Plot 2](path/to/lqr_2d_plot2.png) | ![LQR 2D Simulation GIF 2](path/to/lqr_2d_simulation2.gif) |

##### 3D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![LQR 3D Plot 1](path/to/lqr_3d_plot1.png) | ![LQR 3D Simulation GIF 1](path/to/lqr_3d_simulation1.gif) |
| ![LQR 3D Plot 2](path/to/lqr_3d_plot2.png) | ![LQR 3D Simulation GIF 2](path/to/lqr_3d_simulation2.gif) |

---

## Making Motor Mixer

### Objective
The Motor Mixer serves as the interface between high-level control commands (desired total thrust $F$ and moment/torque vector $\tau = [\tau_x, \tau_y, \tau_z]^T$) and the low-level physical actuators, determining individual motor speeds ($\omega_1^2, \omega_2^2, \omega_3^2, \omega_4^2$).

The mapping from squared motor speeds to wrench inputs is given by:

$$\begin{bmatrix} F \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = M \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

To compute the required motor speeds from high-level outputs, the inverse relationship is evaluated. For over-determined or sensitive control conditions, the **Normal Equation** (least squares approach) is applied to minimize error and actuator output variance:

$$\omega^2 = (M^T M)^{-1} M^T \begin{bmatrix} F \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}$$

> **Motor Mixer Equations**
> 
> ![Motor Mixer Formulas](path/to/motor_mixer_formulas.png)
> 
> *Figure 4: Allocation matrix and least-error normal equations for quadcopter motor speed computation.*

---

## Comparison: PID vs LQR

| Feature | PID Controller | LQR Controller |
| :--- | :--- | :--- |
| **Model Dependency** | Model-free; does not require physical dynamics equations. | Model-based; requires linear state-space matrices ($A, B$). |
| **Architecture** | Single-Input Single-Output (SISO) loops tuned independently. | Multi-Input Multi-Output (MIMO) optimal full-state feedback. |
| **Efficiency** | Tuned heuristically or manually (risk of overshoot/actuator saturation). | Systematically optimizes performance vs. control effort via $Q$ and $R$ matrices. |
| **Coupling Handling** | Can struggle with strong dynamic coupling in 3D motions. | Inherently handles cross-coupling across translational and rotational axes. |

### Comparative Results
![PID vs LQR Comparison Plot](path/to/comparison_plot.png)

![PID vs LQR Comparison Simulation GIF](path/to/comparison_simulation.gif)
