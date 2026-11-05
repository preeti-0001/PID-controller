
%% QUADCOPTER ALTITUDE, ATTITUDE AND POSITION CONTROL
% Cascaded PID/PD Controller Simulation
% Based on: Quadrotor Design Procedure and PID Control
% for Outdoor Free Flight (Meslouli et al., 2017)

clear;
clc;
close all;

%% 1. QUADCOPTER PARAMETERS

m = 1.62;       % Mass (kg)
g = 9.81;       % Gravitational acceleration (m/s^2)
l = 0.25;       % Arm length (m)

Ixx = 3.3e-3;   % Moment of inertia about X-axis
Iyy = 3.3e-3;   % Moment of inertia about Y-axis
Izz = 3.3e-3;   % Moment of inertia about Z-axis

%% 2. PID CONTROLLER INITIALIZATION

% Altitude PID controller
pid_z.kp = 3.0;
pid_z.ki = 0.5;
pid_z.kd = 2.0;
pid_z.integral = 0;
pid_z.prev_error = 0;
pid_z.integral_limit = 20;
pid_z.output_limit = 10;

% Roll PD controller
pid_roll.kp = 0.05;
pid_roll.ki = 0;
pid_roll.kd = 0.02;
pid_roll.integral = 0;
pid_roll.prev_error = 0;
pid_roll.integral_limit = 1;
pid_roll.output_limit = 0.02;

% Pitch PD controller
pid_pitch.kp = 0.05;
pid_pitch.ki = 0;
pid_pitch.kd = 0.02;
pid_pitch.integral = 0;
pid_pitch.prev_error = 0;
pid_pitch.integral_limit = 1;
pid_pitch.output_limit = 0.02;

% Yaw PD controller
pid_yaw.kp = 0.02;
pid_yaw.ki = 0;
pid_yaw.kd = 0.01;
pid_yaw.integral = 0;
pid_yaw.prev_error = 0;
pid_yaw.integral_limit = 1;
pid_yaw.output_limit = 0.01;

%% 3. SIMULATION SETTINGS

dt = 0.01;
T = 120;

N = round(T / dt);
time = (0:N-1) * dt;

%% 4. INITIAL STATE VARIABLES

% Altitude
z = 0;
z_dot = 0;

% Attitude
phi = 0;
theta = 0;
psi = 0;

% Horizontal position
x = 0;
y = 0;

% Horizontal velocities
x_dot = 0;
y_dot = 0;

% Angular velocities
phi_dot = 0;
theta_dot = 0;
psi_dot = 0;

%% 5. DESIRED REFERENCES

z_ref = input('Enter desired altitude (m): ');
x_ref = input('Enter desired x position (m): ');
y_ref = input('Enter desired y position (m): ');
psi_ref = input('Enter desired yaw (rad): ');

% Limit yaw reference
psi_ref = max(-pi, min(pi, psi_ref));

%% 6. DATA STORAGE

z_hist = zeros(1, N);
phi_hist = zeros(1, N);
theta_hist = zeros(1, N);
psi_hist = zeros(1, N);

x_hist = zeros(1, N);
y_hist = zeros(1, N);

phi_ref_hist = zeros(1, N);
theta_ref_hist = zeros(1, N);

U1_hist = zeros(1, N);

%% 7. MAIN SIMULATION LOOP

for k = 1:N

    %% POSITION CONTROL (X-Y)

    error_x = x_ref - x;
    error_y = y_ref - y;

    % Position controller gains
    Kp_pos = 0.08;
    Kd_pos = 0.60;

    % Desired horizontal accelerations
    ax_des = Kp_pos * error_x - Kd_pos * x_dot;
    ay_des = Kp_pos * error_y - Kd_pos * y_dot;

    % Acceleration saturation
    ax_des = max(-1.0, min(1.0, ax_des));
    ay_des = max(-1.0, min(1.0, ay_des));

    % Convert acceleration to desired pitch and roll
    theta_ref = ax_des / g;
    phi_ref = -ay_des / g;

    % Desired attitude limits
    phi_ref = max(-0.2, min(0.2, phi_ref));
    theta_ref = max(-0.2, min(0.2, theta_ref));

    %% ALTITUDE PID CONTROLLER

    error_z = z_ref - z;

    [u_z, pid_z] = pid_update(error_z, dt, pid_z);

    % Hover thrust + altitude correction
    U1 = m * g + u_z;

    % Thrust saturation
    U1 = max(0, min(2 * m * g, U1));

    %% ATTITUDE CONTROLLERS

    error_phi = phi_ref - phi;
    error_theta = theta_ref - theta;
    error_psi = psi_ref - psi;

    [U2, pid_roll] = pid_update(error_phi, dt, pid_roll);
    [U3, pid_pitch] = pid_update(error_theta, dt, pid_pitch);
    [U4, pid_yaw] = pid_update(error_psi, dt, pid_yaw);

    %% ROTATIONAL DYNAMICS

    phi_ddot = U2 / Ixx;
    theta_ddot = U3 / Iyy;
    psi_ddot = U4 / Izz;

    %% ALTITUDE DYNAMICS

    if isfinite(phi) && isfinite(theta)
        z_ddot = cos(phi) * cos(theta) * U1 / m - g;
    else
        z_ddot = 0;
    end

    %% STATE INTEGRATION

    % Altitude
    z_dot = z_dot + z_ddot * dt;
    z = z + z_dot * dt;

    % Roll
    phi_dot = phi_dot + phi_ddot * dt;
    phi = phi + phi_dot * dt;

    % Pitch
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;

    % Yaw
    psi_dot = psi_dot + psi_ddot * dt;
    psi = psi + psi_dot * dt;

    %% ANGLE LIMITS

    phi = max(-pi, min(pi, phi));
    theta = max(-pi/2, min(pi/2, theta));
    psi = max(-pi, min(pi, psi));

    %% HORIZONTAL TRANSLATIONAL DYNAMICS

    k_drag = 0.8;

    x_ddot = g * theta - k_drag * x_dot;
    y_ddot = -g * phi - k_drag * y_dot;

    % Integrate horizontal velocities
    x_dot = x_dot + x_ddot * dt;
    y_dot = y_dot + y_ddot * dt;

    % Integrate horizontal positions
    x = x + x_dot * dt;
    y = y + y_dot * dt;

    %% SAVE SIMULATION DATA

    z_hist(k) = z;

    phi_hist(k) = phi;
    theta_hist(k) = theta;
    psi_hist(k) = psi;

    x_hist(k) = x;
    y_hist(k) = y;

    phi_ref_hist(k) = phi_ref;
    theta_ref_hist(k) = theta_ref;

    U1_hist(k) = U1;

end

%% 8. PLOTTING RESULTS

figure('Name', 'Quadcopter Control Responses', ...
       'NumberTitle', 'off', ...
       'Position', [100 100 1200 800]);

% Altitude response
subplot(3, 2, 1);

plot(time, z_hist, 'b', 'LineWidth', 1.5);
hold on;
yline(z_ref, 'r--', 'LineWidth', 1.2);

title('Altitude Response');
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('z', 'z_{ref}');
grid on;

% Roll response
subplot(3, 2, 2);

plot(time, phi_hist, 'b', 'LineWidth', 1.5);
hold on;
plot(time, phi_ref_hist, 'r--', 'LineWidth', 1.2);

title('Roll Response');
xlabel('Time (s)');
ylabel('Roll Angle (rad)');
legend('\phi', '\phi_{ref}');
grid on;

% Pitch response
subplot(3, 2, 3);

plot(time, theta_hist, 'b', 'LineWidth', 1.5);
hold on;
plot(time, theta_ref_hist, 'r--', 'LineWidth', 1.2);

title('Pitch Response');
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
legend('\theta', '\theta_{ref}');
grid on;

% Yaw response
subplot(3, 2, 4);

plot(time, psi_hist, 'b', 'LineWidth', 1.5);
hold on;
yline(psi_ref, 'r--', 'LineWidth', 1.2);

title('Yaw Response');
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');
legend('\psi', '\psi_{ref}');
grid on;

% X-position response
subplot(3, 2, 5);

plot(time, x_hist, 'b', 'LineWidth', 1.5);
hold on;
yline(x_ref, 'r--', 'LineWidth', 1.2);

title('X-Position Response');
xlabel('Time (s)');
ylabel('X Position (m)');
legend('x', 'x_{ref}');
grid on;

% Y-position response
subplot(3, 2, 6);

plot(time, y_hist, 'b', 'LineWidth', 1.5);
hold on;
yline(y_ref, 'r--', 'LineWidth', 1.2);

title('Y-Position Response');
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('y', 'y_{ref}');
grid on;

sgtitle('Quadcopter Altitude, Attitude and Position Control');

%% 9. PID CONTROLLER FUNCTION

function [output, controller] = pid_update(error, dt, controller)

    % Integral term with anti-windup
    controller.integral = controller.integral + error * dt;

    controller.integral = max( ...
        -controller.integral_limit, ...
        min(controller.integral_limit, controller.integral));

    % Derivative term
    derivative = (error - controller.prev_error) / dt;

    controller.prev_error = error;

    % PID control equation
    output = controller.kp * error ...
           + controller.ki * controller.integral ...
           + controller.kd * derivative;

    % Output saturation
    if ~isempty(controller.output_limit)

        output = max( ...
            -controller.output_limit, ...
            min(controller.output_limit, output));

    end

end
