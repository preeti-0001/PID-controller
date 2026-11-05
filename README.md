# Quadcopter Altitude, Attitude, and Position Control Using PID/PD Controllers

A Python-based simulation of a quadcopter using cascaded PID and PD controllers for altitude, attitude, and horizontal position control.

## Overview

This project implements a cascaded control architecture to stabilize and control a quadcopter in a simulated environment. The controller regulates altitude, roll, pitch, yaw, and horizontal position using a mathematical model of quadcopter dynamics.

The implementation is based on Section IV (PID Controller and Simulation Results) of the research paper *Quadrotor Design Procedure and PID Control for Outdoor Free Flight* by Ismat Meslouli et al.

The simulation allows users to specify desired altitude, horizontal position, and yaw angle. The system then computes the required control inputs and visualizes the resulting response using Matplotlib.

## Features

- Altitude control using a PID controller.
- Roll, pitch, and yaw control using PD controllers.
- Horizontal position control using outer-loop PD controllers.
- Cascaded control architecture.
- Nonlinear altitude dynamics and simplified horizontal dynamics.
- Actuator saturation and integral anti-windup.
- Real-time numerical simulation using Euler integration.
- Six plots for analyzing altitude, attitude, and position responses.

## Controller Architecture

The control system consists of two cascaded layers.

| Control Loop | Controller | Purpose |
|---|---|---|
| Altitude | PID | Regulates vertical position |
| Horizontal position (X-Y) | PD | Generates desired roll and pitch angles |
| Roll | PD | Controls rotation about the X-axis |
| Pitch | PD | Controls rotation about the Y-axis |
| Yaw | PD | Controls rotation about the Z-axis |

The horizontal position controllers generate desired accelerations, which are converted into desired roll and pitch angles using the small-angle approximation.

## Mathematical Model

### 1. Altitude Dynamics

The vertical acceleration of the quadcopter is modeled as:

\[
\ddot{z}=\frac{U_1\cos\phi\cos\theta}{m}-g
\]

where:

- \(U_1\): Total thrust
- \(m\): Mass of the quadcopter
- \(g\): Gravitational acceleration
- \(\phi\): Roll angle
- \(\theta\): Pitch angle

### 2. Rotational Dynamics

The rotational dynamics are simplified as:

\[
\ddot{\phi}=\frac{U_2}{I_{xx}}
\]

\[
\ddot{\theta}=\frac{U_3}{I_{yy}}
\]

\[
\ddot{\psi}=\frac{U_4}{I_{zz}}
\]

where \(U_2\), \(U_3\), and \(U_4\) are the roll, pitch, and yaw control moments.

### 3. Horizontal Position Dynamics

The horizontal motion is modeled using a simplified small-angle approximation with linear damping:

\[
\ddot{x}=g\theta-k_{drag}\dot{x}
\]

\[
\ddot{y}=-g\phi-k_{drag}\dot{y}
\]

The desired accelerations are calculated using PD control:

\[
a_{x,des}=K_{p,x}(x_{ref}-x)-K_{d,x}\dot{x}
\]

\[
a_{y,des}=K_{p,y}(y_{ref}-y)-K_{d,y}\dot{y}
\]

The desired roll and pitch angles are then calculated as:

\[
\theta_{ref}\approx\frac{a_{x,des}}{g}
\]

\[
\phi_{ref}\approx-\frac{a_{y,des}}{g}
\]

### 4. PID Control Law

The general PID control law is:

\[
U(t)=K_pe(t)+K_i\int e(t)\,dt+K_d\frac{de(t)}{dt}
\]

For the roll, pitch, and yaw controllers, \(K_i=0\), making them PD controllers.

## Simulation Parameters

The following parameters are used in the simulation:

| Parameter | Value |
|---|---|
| Mass | 1.62 kg |
| Gravity | 9.81 m/s² |
| Arm length | 0.25 m |
| Moment of inertia \(I_{xx}\) | 3.3 × 10⁻³ kg·m² |
| Moment of inertia \(I_{yy}\) | 3.3 × 10⁻³ kg·m² |
| Moment of inertia \(I_{zz}\) | 3.3 × 10⁻³ kg·m² |
| Simulation time | 120 seconds |
| Time step | 0.01 seconds |
| Integration method | Forward Euler |

## Requirements

- Python 3.8 or later
- NumPy
- Matplotlib

## Installation

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/quadcopter-pid-control.git
cd quadcopter-pid-control
```

Install the required dependencies:

```bash
pip install numpy matplotlib
```

## Running the Simulation

Run the Python script:

```bash
python quadcopter_pid.py
```

Enter the desired reference values when prompted:

```text
Enter desired altitude: 5
Enter desired x position (m): 2
Enter desired y position (m): 3
Enter desired yaw (rad): 0.5
```

The simulation will run and generate six plots showing the response of the quadcopter.

## Results

The simulation generates the following plots:

1. Altitude response and desired altitude.
2. Roll angle response.
3. Pitch angle response.
4. Yaw angle response.
5. X-position response.
6. Y-position response.

These plots can be used to analyze the tracking performance and stability of the cascaded control system.

## Numerical Stability

During initial simulations, numerical overflow occurred because of excessive controller outputs and accumulation of the altitude integral term.

The following measures were implemented to improve numerical stability:

- **Actuator saturation:** Limits the total thrust and controller outputs to prevent unrealistic control signals.
- **Integral anti-windup:** Restricts the accumulated integral error to prevent excessive controller action.
- **Reference angle limits:** Constrains the desired roll and pitch angles.
- **State angle limits:** Prevents the simulated attitude angles from growing beyond predefined bounds.

## Limitations

- The model assumes simplified rotational dynamics without gyroscopic coupling.
- Horizontal dynamics use a small-angle approximation and linear damping.
- The simulation uses idealized control moments rather than a detailed motor and propeller model.
- The Euler integration method is used instead of more advanced numerical integration techniques.
- The simulation is intended for academic study and does not represent a flight-tested control system.

## References

Meslouli, I., Mesli, R., Kahouadji, M., Choukchou-Braham, A., & Cherki, B. (2017). *Quadrotor Design Procedure and PID Control for Outdoor Free Flight.*

## Author

**Preeti**

M.Tech – Automation and Robotics  
Defence Institute of Advanced Technology (DIAT), Pune