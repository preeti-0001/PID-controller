
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# PID CONTROLLER
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_output, self.max_output = output_limits

    def update(self, error, dt):
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        candidate_integral = self.integral + error * dt

        unsat = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )

        output = unsat
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        # Anti-windup: integrate only when not saturated
        if abs(output - unsat) < 1e-9:
            self.integral = candidate_integral

        self.prev_error = error
        return output


# ============================================================
# QUADCOPTER PARAMETERS (from paper)
# ============================================================
m = 2.0
G = 9.81
l = 0.2

Ix = 1.25
Iy = 1.25
Iz = 2.5

K1 = K2 = K3 = 0.01
K4 = K5 = K6 = 0.012

# Limits
OUTER_MAX_TILT = np.deg2rad(5)   # small tilt -> stable x,y response
INNER_MAX_TILT = np.deg2rad(8)


# ============================================================
# SIMULATION SETTINGS
# ============================================================
dt = 0.01
T = 130.0
N = int(T / dt)
time = np.arange(0, T, dt)


# ============================================================
# USER INPUTS
# ============================================================
z_des = float(input("Enter desired altitude (m): "))
x_des = float(input("Enter desired x position (m): "))
y_des = float(input("Enter desired y position (m): "))
psi_ref = float(input("Enter desired yaw (deg): "))
psi_des = np.deg2rad(psi_ref)


# ============================================================
# PID CONTROLLERS
# ============================================================
# Outer position loop (very conservative gains)
pid_x = PID(0.01, 0.0, 0.02, (-OUTER_MAX_TILT, OUTER_MAX_TILT))
pid_y = PID(0.01, 0.0, 0.02, (-OUTER_MAX_TILT, OUTER_MAX_TILT))

# Altitude loop
pid_z = PID(4.0, 0.4, 3.0, (-5.0, 5.0))

# Inner attitude loops
pid_phi = PID(5.0, 0.0, 3.0, (-10.0, 10.0))
pid_theta = PID(5.0, 0.0, 3.0, (-10.0, 10.0))
pid_psi = PID(4.0, 0.05, 1.5, (-5.0, 5.0))


# ============================================================
# INITIAL STATES
# ============================================================
x = y = z = 0.0
x_dot = y_dot = z_dot = 0.0

phi = theta = psi = 0.0
phi_dot = theta_dot = psi_dot = 0.0


# ============================================================
# DATA STORAGE
# ============================================================
x_hist, y_hist, z_hist = [], [], []
phi_hist, theta_hist, psi_hist = [], [], []
phi_des_hist, theta_des_hist = [], []


# ============================================================
# MAIN SIMULATION LOOP
# ============================================================
for k in range(N):

    # --------------------------------------------------------
    # OUTER POSITION LOOP
    # --------------------------------------------------------
    ex = x_des - x
    ey = y_des - y

    # Desired pitch controls x, desired roll controls y
    theta_des = pid_x.update(ex, dt)
    phi_des = -pid_y.update(ey, dt)

    # Additional hard limit
    theta_des = np.clip(theta_des, -INNER_MAX_TILT, INNER_MAX_TILT)
    phi_des = np.clip(phi_des, -INNER_MAX_TILT, INNER_MAX_TILT)

    # --------------------------------------------------------
    # ATTITUDE CONTROLLERS
    # --------------------------------------------------------
    u2 = pid_phi.update(phi_des - phi, dt)
    u3 = pid_theta.update(theta_des - theta, dt)
    u4 = pid_psi.update(psi_des - psi, dt)

    # --------------------------------------------------------
    # ALTITUDE CONTROLLER
    # Compensate loss of vertical thrust when tilted
    # --------------------------------------------------------
    ez = z_des - z
    thrust_correction = pid_z.update(ez, dt)

    tilt_comp = max(np.cos(phi) * np.cos(theta), 0.5)
    u1 = (m * G + thrust_correction) / tilt_comp
    u1 = np.clip(u1, 0.0, 40.0)

    # --------------------------------------------------------
    # DYNAMICS
    # --------------------------------------------------------
    x_ddot = (u1 / m) * (np.cos(phi) * np.sin(theta)) - (K1 / m) * x_dot
    y_ddot = (u1 / m) * (-np.sin(phi)) - (K2 / m) * y_dot
    z_ddot = (u1 / m) * (np.cos(phi) * np.cos(theta)) - G - (K3 / m) * z_dot

    phi_ddot = l * u2 / Ix - (K4 / Ix) * phi_dot
    theta_ddot = l * u3 / Iy - (K5 / Iy) * theta_dot
    psi_ddot = u4 / Iz - (K6 / Iz) * psi_dot

    # --------------------------------------------------------
    # EULER INTEGRATION
    # --------------------------------------------------------
    x_dot += x_ddot * dt
    y_dot += y_ddot * dt
    z_dot += z_ddot * dt

    x += x_dot * dt
    y += y_dot * dt
    z += z_dot * dt

    phi_dot += phi_ddot * dt
    theta_dot += theta_ddot * dt
    psi_dot += psi_ddot * dt

    phi += phi_dot * dt
    theta += theta_dot * dt
    psi += psi_dot * dt

    # --------------------------------------------------------
    # STORE DATA
    # --------------------------------------------------------
    x_hist.append(x)
    y_hist.append(y)
    z_hist.append(z)

    phi_hist.append(np.rad2deg(phi))
    theta_hist.append(np.rad2deg(theta))
    psi_hist.append(np.rad2deg(psi))

    phi_des_hist.append(np.rad2deg(phi_des))
    theta_des_hist.append(np.rad2deg(theta_des))


# ============================================================
# PLOTS
# ============================================================
fig, axs = plt.subplots(3, 2, figsize=(14, 10))

axs[0, 0].plot(time, x_hist)
axs[0, 0].axhline(x_des, linestyle='--')
axs[0, 0].set_title('X Position')
axs[0, 0].grid(True)

axs[0, 1].plot(time, y_hist)
axs[0, 1].axhline(y_des, linestyle='--')
axs[0, 1].set_title('Y Position')
axs[0, 1].grid(True)

axs[1, 0].plot(time, z_hist)
axs[1, 0].axhline(z_des, linestyle='--')
axs[1, 0].set_title('Altitude')
axs[1, 0].grid(True)

axs[1, 1].plot(time, phi_hist, label='phi')
axs[1, 1].plot(time, phi_des_hist, '--', label='phi_des')
axs[1, 1].set_title('Roll')
axs[1, 1].legend()
axs[1, 1].grid(True)

axs[2, 0].plot(time, theta_hist, label='theta')
axs[2, 0].plot(time, theta_des_hist, '--', label='theta_des')
axs[2, 0].set_title('Pitch')
axs[2, 0].legend()
axs[2, 0].grid(True)

axs[2, 1].plot(time, psi_hist)
axs[2, 1].axhline(np.rad2deg(psi_des), linestyle='--')
axs[2, 1].set_title('Yaw')
axs[2, 1].grid(True)

for ax in axs.flat:
    ax.set_xlabel('Time (s)')

plt.tight_layout()
plt.show()