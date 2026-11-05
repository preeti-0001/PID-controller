import numpy as np
import matplotlib.pyplot as plt

# 1. QUADCOPTER PARAMETERS (Table I from the paper)
m = 1.62
g = 9.81
l = 0.25
b=15.65e-4
d=2.08e-5

Ixx = 3.3e-3
Iyy = 3.3e-3
Izz = 3.3e-3


# 2. PID CONTROLLER CLASS
class PID:
    def __init__(self, kp, ki, kd, integral_limit=10.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.prev_error = 0.0

        self.integral_limit = integral_limit
        self.output_limit = output_limit

    def update(self, error, dt):
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(
            self.integral, -self.integral_limit, self.integral_limit
        )

        # Derivative term
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID equation
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Saturate controller output
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        return output


def calculate_omegas(U1, U2, U3, U4):

    # Placeholder for omega calculation logic
    A = U1 / b
    B = U2 / (l * b)
    C = U3 / (l * b)
    D = U4 / d

    # Solve for squared speeds
    w1_sq = (A - 2*C - D) / 4
    w2_sq = (A - 2*B + D) / 4
    w3_sq = (A + 2*C - D) / 4
    w4_sq = (A + 2*B + D) / 4

    # Check for negative values
    if min(w1_sq, w2_sq, w3_sq, w4_sq) < 0:
        raise ValueError(
            "One or more computed squared speeds are negative. "
            "Check control inputs and parameters."
        )

    # Compute actual speeds
    w1 = np.sqrt(w1_sq)
    w2 = np.sqrt(w2_sq)
    w3 = np.sqrt(w3_sq)
    w4 = np.sqrt(w4_sq)

    return [w1, w2, w3, w4]

# 3. PID GAINS

# Attitude controllers
pid_z = PID(kp=3.0, ki=0.5, kd=2.0, integral_limit=20, output_limit=10)

pid_roll = PID(kp=0.05, ki=0.0, kd=0.02, integral_limit=1, output_limit=0.02)

pid_pitch = PID(kp=0.05, ki=0.0, kd=0.02, integral_limit=1, output_limit=0.02)

# Altitude controller
pid_yaw = PID(kp=0.02, ki=0.0, kd=0.01, integral_limit=1, output_limit=0.01)

pid_x = PID(kp=3.0, ki=0.0, kd=1.5, integral_limit=10, output_limit=1.0)

pid_y = PID(kp=3.0, ki=0.0, kd=1.5, integral_limit=10, output_limit=1.0)
# 4. SIMULATION SETTINGS
dt = 0.001
T = 40
N = int(T / dt)
time = np.linspace(0, T, N)

# 5. STATE VARIABLES
X = np.zeros(6)  # [x, y, z, phi, theta, psi]
X_dot = np.zeros(6)  # [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
X_ddot = np.zeros(6)  # [x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot]

# 6. DESIRED REFERENCES
z_ref = float(input("Enter desired altitude: "))
x_ref = float(input("Enter desired x position (m): "))
y_ref = float(input("Enter desired y position (m): "))

psi_ref = float(input("Enter desired yaw (deg): "))
psi_ref = np.deg2rad(psi_ref)
psi_ref = np.clip(psi_ref, -np.pi, np.pi)

# 7. DATA STORAGE FOR PLOTS

X_hist = np.zeros((N, 6))

AG_hist = np.zeros((N, 4)) 

# 8. MAIN SIMULATION LOOP
for i in range(N):
    x = X[0]
    y = X[1]
    z = X[2]
    phi = X[3]
    theta = X[4]
    psi = X[5]
    x_dot = X_dot[0]
    y_dot = X_dot[1]
    # POSITION CONTROL (x, y)


    # Position errors
    error_x = x_ref - x
    error_y = y_ref - y

    # Desired accelerations
    ax_des = pid_x.update(error_x, dt)
    ay_des = pid_y.update(error_y, dt)

    # ALTITUDE CONTROLLER
    error_z = z_ref - z
    U1 = pid_z.update(error_z, dt)

    # Hover thrust + correction
    U1 = m * g + U1

    # Thrust saturation
    U1 = np.clip(U1, 0.0, 2.0 * m * g)

    # Convert desired accelerations to roll/pitch
    theta_ref = ax_des / g
    phi_ref = - ay_des / g

    # Limit desired angles
    phi_ref = np.clip(phi_ref, -0.2, 0.2)
    theta_ref = np.clip(theta_ref, -0.2, 0.2)

    # ATTITUDE CONTROLLERS
    error_phi = phi_ref - phi
    error_theta = theta_ref - theta
    error_psi = psi_ref - psi

    U2 = pid_roll.update(error_phi, dt)
    U3 = pid_pitch.update(error_theta, dt)
    U4 = pid_yaw.update(error_psi, dt)

    # ROTATIONAL DYNAMICS
    phi_ddot = U2 / Ixx
    theta_ddot = U3 / Iyy
    psi_ddot = U4 / Izz

    # ALTITUDE DYNAMICS
    # Prevent NaNs if angles become invalid
    if np.isfinite(phi) and np.isfinite(theta):
        z_ddot = np.cos(phi) * np.cos(theta) * U1 / m - g
    else:
        z_ddot = 0.0

    
    # ANGLE LIMITS
    phi = np.clip(phi, -np.pi, np.pi)
    theta = np.clip(theta, -np.pi / 2, np.pi / 2)
    psi = np.clip(psi, -np.pi, np.pi)

    # Simple translational model with damping
    k_drag = 0.8

    x_ddot = g * theta - k_drag * x_dot
    y_ddot = -g * phi - k_drag * y_dot

    X_ddot = np.array([x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot])
    X_dot += X_ddot * dt
    X += X_dot * dt


    # SAVE DATA
    X_hist[i, :] = X
    AG_hist[i, :] = calculate_omegas(U1, U2, U3, U4)

# 9. PLOTTING RESULTS
plt.figure(figsize=(12, 8))

# Altitude
plt.subplot(3, 2, 1)
plt.plot(time, X_hist[:, 2], label="z")
plt.axhline(z_ref, color="r", linestyle="--", label="z_ref")
plt.title("Altitude Response")
plt.xlabel("Time (s)")
plt.ylabel("z (m)")
plt.grid()
plt.legend()

# Roll
plt.subplot(3, 2, 2)
plt.plot(time, X_hist[:, 3], label="phi")
plt.axhline(phi_ref, color="r", linestyle="--")
plt.title("Roll Response")
plt.xlabel("Time (s)")
plt.ylabel("phi (rad)")
plt.grid()

# Pitch
plt.subplot(3, 2, 3)
plt.plot(time, X_hist[:, 4], label="theta")
plt.axhline(theta_ref, color="r", linestyle="--")
plt.title("Pitch Response")
plt.xlabel("Time (s)")
plt.ylabel("theta (rad)")
plt.grid()

# Yaw
plt.subplot(3, 2, 4)
plt.plot(time, X_hist[:, 5], label="psi")
plt.axhline(psi_ref, color="r", linestyle="--")
plt.title("Yaw Response")
plt.xlabel("Time (s)")
plt.ylabel("psi (rad)")
plt.grid()

# Position x
plt.subplot(3, 2, 5)
plt.plot(time, X_hist[:, 0], label="x")
plt.axhline(x_ref, color="r", linestyle="--")
plt.title("Position x Response")
plt.xlabel("Time (s)")
plt.ylabel("x (m)")
plt.grid()

# Position y
plt.subplot(3, 2, 6)
plt.plot(time, X_hist[:, 1], label="y")
plt.axhline(y_ref, color="r", linestyle="--")
plt.title("Position y Response")
plt.xlabel("Time (s)")
plt.ylabel("y (m)")
plt.grid()

plt.tight_layout()
plt.show()


plt.figure(figsize=(12, 4))
plt.subplot(2, 2, 1)
plt.plot(time, AG_hist[:, 0], label="W1 (Thrust)")
plt.title("Motor 1 Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("W1 (rad/s)")
plt.grid()
plt.subplot(2, 2, 2)
plt.plot(time, AG_hist[:, 1], label="W2 (Roll)")
plt.title("Mototr 2 Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("W2 (rad/s)")
plt.grid()
plt.subplot(2, 2, 3)
plt.plot(time, AG_hist[:, 2], label="W3 (Pitch)")
plt.title("Motor 3 Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("W3 (rad/s)")
plt.grid()
plt.subplot(2, 2, 4)
plt.plot(time, AG_hist[:, 3], label="W4 (Yaw)")
plt.title("Motor 4 Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("W4 (rad/s)")
plt.grid()

plt.tight_layout()
plt.show()
