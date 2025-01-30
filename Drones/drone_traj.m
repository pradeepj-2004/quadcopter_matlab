m = 1.535; %mass of drone
g = 9.81; %gravity

%Inertia values
Ixx= 0.029125;
Iyy= 0.029125;
Izz= 0.055225; 

%Drag constants
kt=0.47;

%PID gains for controller

kp_phi=1.5;
kd_phi=1.2;

kp_theta=1.5;
kd_theta=1.5;

kp_psi=0.09;
kd_psi=0.07;  

kp_z_dot=0.08;
kd_z_dot=0.08;

kp_x_dot=-0.15;
kd_x_dot=-0.25;

kp_y_dot=0.15;
kd_y_dot=0.25;

% Desired Fixed values
phi_dot_d=0;
theta_dot_d=0;
psi_dot_d=0;
x_ddot_d=0;
y_ddot_d=0;
z_ddot_d=0;

psi_d=0;


%velocity desired values
x_dot_d=1;
y_dot_d=3;
z_dot_d=9;

% Time period of controller(50 ms)
dt = 0.1;

% Total simulation time
t_total = 10;

tspan = 0:dt:t_total; % Full time range

y0 = zeros(12, 1); % Initial state

u = [0; 0.0; 0.0; 0.0]; % Initial control input

y_all = []; % To store all states
t_all = []; % To store all time

current_state = y0; % Start with the initial state
T=0;

%Motor Thrust
T1=0;
T2=0;
T3=0;
T4=0;
TM=[T1;T2;T3;T4];

%Control Allocation Matrix
c=0.1;
l=0.1;
CM = [1 1 1 1; 0 -l 0 l;-l 0 l 0; c -c c -c];

%Control inputs;
roll=0;
pitch=0;
yaw=0;

for i = 1:length(tspan)-1

    t_current = [tspan(i), tspan(i+1)];

    % Solve ODE for this time step
    [t, y] = ode45(@(t, y) droneDynamics(t, y, u,m,g,Ixx,Iyy,Izz), t_current, current_state);

    % Store results
    y_all = [y_all; y(1:end-1, :)]; % Exclude the last point to avoid duplication
    t_all = [t_all; t(1:end-1)];
    
    % Update the current state
    current_state = y(end, :)';

    phi=current_state(7);
    phi_dot=current_state(10);

    theta=current_state(8);
    theta_dot=current_state(11);

    psi=current_state(9);
    psi_dot=current_state(12);
    
    x_dot=current_state(4);
    y_dot=current_state(5);
    z_dot=current_state(6);

    x=current_state(1);
    y=current_state(2);
    z=current_state(3);

    acc=[(1/(m))*((cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*T -kt*x_dot);(1/(m))*((cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi))*T -kt*y_dot) ;(1/(m))*((cos(phi)*cos(theta))*T - m*g - kt*z_dot)];
    x_ddot=acc(1);
    y_ddot=acc(2);
    z_ddot=acc(3);

    [phi_d,theta_d]=outerController(x_dot_d,x_dot,x_ddot_d,x_ddot,kp_x_dot,kd_x_dot,y_dot_d,y_dot,y_ddot_d,y_ddot,kp_y_dot,kd_y_dot);

    [e_r,e_p,yaw,e_t] = innerController(phi_d,phi,phi_dot_d,phi_dot,kp_phi,kd_phi,theta_d,theta,theta_dot_d,theta_dot,kp_theta,kd_theta,psi_d,psi,psi_dot_d,psi_dot,kp_psi,kd_psi,z_dot_d,z_dot,z_ddot_d,z_ddot,kp_z_dot,kd_z_dot);
    roll=roll+e_r;
    pitch=pitch+e_p;
    T=T+e_t;
    u=[T;roll;pitch;yaw];

    TM =inv(CM)*u;


end

% Add the final point
y_all = [y_all; current_state'];
t_all = [t_all; tspan(end)];

x_dot=current_state(4)
y_dot=current_state(5)
z_dot=current_state(6)

phi=current_state(7)
theta=current_state(8)
psi=current_state(9)

% Plot Drone Trajectory
figure;
plot3(y_all(:, 1), y_all(:, 2), y_all(:, 3), 'b', 'LineWidth', 1.5);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Drone Trajectory');
legend('Trajectory');

% Plot Position over Time
% figure;
% subplot(3, 1, 1);
% plot(t_all, y_all(:, 1), 'r', 'LineWidth', 1.2);
% grid on;
% xlabel('Time (s)'); ylabel('X (m)');
% title('Position X');
% 
% subplot(3, 1, 2);
% plot(t_all, y_all(:, 2), 'g', 'LineWidth', 1.2);
% grid on;
% xlabel('Time (s)'); ylabel('Y (m)');
% title('Position Y');
% 
% subplot(3, 1, 3);
% plot(t_all, y_all(:, 3), 'b', 'LineWidth', 1.2);
% grid on;
% xlabel('Time (s)'); ylabel('Z (m)');
% title('Position Z');

% Plot Velocities
figure;
subplot(3, 1, 1);
plot(t_all, y_all(:, 4), 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('X Dot (m/s)');
title('Velocity X');

subplot(3, 1, 2);
plot(t_all, y_all(:, 5), 'g', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('Y Dot (m/s)');
title('Velocity Y');

subplot(3, 1, 3);
plot(t_all, y_all(:, 6), 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('Z Dot (m/s)');
title('Velocity Z');

% Plot Orientation Angles
figure;
subplot(3, 1, 1);
plot(t_all, rad2deg(y_all(:, 7)), 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Roll Angle (\phi)');

subplot(3, 1, 2);
plot(t_all, rad2deg(y_all(:, 8)), 'g', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Pitch Angle (\theta)');

subplot(3, 1, 3);
plot(t_all, rad2deg(y_all(:, 9)), 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('\psi (deg)');
title('Yaw Angle (\psi)');



