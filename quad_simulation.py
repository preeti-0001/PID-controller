import numpy as np
import matplotlib.pyplot as plt

# 1. QUADCOPTER PARAMETERS (Table I from the paper)
m = 1.62
g = 9.81
l = 0.25

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

# 3. PID GAINS
# Altitude controller
pid_yaw = PID(kp=0.02, ki=0.0, kd=0.01, integral_limit=1, output_limit=0.01)

# Attitude controllers
pid_z = PID(kp=3.0, ki=0.5, kd=2.0, integral_limit=20, output_limit=10)

pid_roll = PID(kp=0.05, ki=0.0, kd=0.02, integral_limit=1, output_limit=0.02)

pid_pitch = PID(kp=0.05, ki=0.0, kd=0.02, integral_limit=1, output_limit=0.02)

# Outer-loop position controllers
pid_x = PID(
    kp=0.8,
    ki=0.0,
    kd=0.4,
    integral_limit=10,
    output_limit=3.0,  # max acceleration (m/s^2)
)

pid_y = PID(kp=0.8, ki=0.0, kd=0.4, integral_limit=10, output_limit=3.0)
# 4. SIMULATION SETTINGS
dt = 0.01
T = 120
N = int(T / dt)
time = np.linspace(0, T, N)

# 5. STATE VARIABLES
# Altitude states
z = 0.0  # Position (m)
z_dot = 0.0  # Velocity (m/s)

# Attitude states
phi = 0.0  # Roll angle (rad)
theta = 0.0  # Pitch angle (rad)
psi = 0.0  # Yaw angle (rad)

# Horizontal positions
x = 0.0
y = 0.0

# Horizontal velocities
x_dot = 0.0
y_dot = 0.0

# Angular velocities
phi_dot = 0.0
theta_dot = 0.0
psi_dot = 0.0

# 6. DESIRED REFERENCES
z_ref = float(input("Enter desired altitude: "))
x_ref = float(input("Enter desired x position (m): "))
y_ref = float(input("Enter desired y position (m): "))

psi_ref = float(input("Enter desired yaw (rad): "))

# Safety limits on references
psi_ref = np.clip(psi_ref, -np.pi, np.pi)

# 7. DATA STORAGE FOR PLOTS
z_hist = []
phi_hist = []
theta_hist = []
psi_hist = []
x_hist = []
y_hist = []

# 8. MAIN SIMULATION LOOP
for t in time:
    # POSITION CONTROL (x, y)
    
    # Position errors
    error_x = x_ref - x
    error_y = y_ref - y
    
    # PD gains for position control
    Kp_pos = 0.08      # position gain
    Kd_pos = 0.60      # velocity damping gain
    
    # Desired accelerations
    ax_des = Kp_pos * error_x - Kd_pos * x_dot
    ay_des = Kp_pos * error_y - Kd_pos * y_dot
    
    # Limit desired accelerations
    ax_des = np.clip(ax_des, -1.0, 1.0)
    ay_des = np.clip(ay_des, -1.0, 1.0)
    
    # Convert desired accelerations to roll/pitch
    # Small-angle approximation:
    # theta ≈ ax/g
    # phi   ≈ -ay/g
    theta_ref = ax_des / g
    phi_ref   = -ay_des / g
    
    # Limit desired angles
    phi_ref = np.clip(phi_ref, -0.2, 0.2)
    theta_ref = np.clip(theta_ref, -0.2, 0.2)

    #.....
    # ALTITUDE CONTROLLER
    error_z = z_ref - z
    u_z = pid_z.update(error_z, dt)

    # Hover thrust + correction
    U1 = m * g + u_z

    # Thrust saturation
    U1 = np.clip(U1, 0.0, 2.0 * m * g)

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

    # INTEGRATION
    z_dot += z_ddot * dt
    z += z_dot * dt

    phi_dot += phi_ddot * dt
    phi += phi_dot * dt

    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    psi_dot += psi_ddot * dt
    psi += psi_dot * dt

    # ANGLE LIMITS
    phi = np.clip(phi, -np.pi, np.pi)
    theta = np.clip(theta, -np.pi / 2, np.pi / 2)
    psi = np.clip(psi, -np.pi, np.pi)

    # Simple translational model with damping
    k_drag = 0.8

    x_ddot = g * theta - k_drag * x_dot
    y_ddot = -g * phi - k_drag * y_dot

    # Integrate
    x_dot += x_ddot * dt
    y_dot += y_ddot * dt

    x += x_dot * dt
    y += y_dot * dt

    # SAVE DATA
    z_hist.append(z)
    phi_hist.append(phi)
    theta_hist.append(theta)
    psi_hist.append(psi)
    x_hist.append(x)
    y_hist.append(y)

# 9. PLOTTING RESULTS
plt.figure(figsize=(12, 8))

# Altitude
plt.subplot(3, 2, 1)
plt.plot(time, z_hist, label="z")
plt.axhline(z_ref, color="r", linestyle="--", label="z_ref")
plt.title("Altitude Response")
plt.xlabel("Time (s)")
plt.ylabel("z (m)")
plt.grid()
plt.legend()

# Roll
plt.subplot(3, 2, 2)
plt.plot(time, phi_hist, label="phi")
plt.axhline(phi_ref, color="r", linestyle="--")
plt.title("Roll Response")
plt.xlabel("Time (s)")
plt.ylabel("phi (rad)")
plt.grid()

# Pitch
plt.subplot(3, 2, 3)
plt.plot(time, theta_hist, label="theta")
plt.axhline(theta_ref, color="r", linestyle="--")
plt.title("Pitch Response")
plt.xlabel("Time (s)")
plt.ylabel("theta (rad)")
plt.grid()

# Yaw
plt.subplot(3, 2, 4)
plt.plot(time, psi_hist, label="psi")
plt.axhline(psi_ref, color="r", linestyle="--")
plt.title("Yaw Response")
plt.xlabel("Time (s)")
plt.ylabel("psi (rad)")
plt.grid()

# Position x
plt.subplot(3, 2, 5)
plt.plot(time, x_hist, label="x")
plt.axhline(x_ref, color="r", linestyle="--")
plt.title("Position x Response")
plt.xlabel("Time (s)")
plt.ylabel("x (m)")
plt.grid()

# Position y
plt.subplot(3, 2, 6)
plt.plot(time, y_hist, label="y")
plt.axhline(y_ref, color="r", linestyle="--")
plt.title("Position y Response")
plt.xlabel("Time (s)")
plt.ylabel("y (m)")
plt.grid()

plt.tight_layout()
plt.show()
