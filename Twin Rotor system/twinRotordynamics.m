function dydt = twinRotordynamics(t,fx,m,g,l,Izz, u)
    % State variables
    x = fx(1); y = fx(2); % position
    x_dot = fx(3); y_dot = fx(4);% velocity
    theta = fx(5); % orientation
    theta_dot = fx(6);

    T1 = u(1);
    T2 = u(2);
    % Translational dynamics
    acc=[(T1+T2)*cos(theta)-m*g;(T1+T2)*sin(theta)];
    ang_acc = (1/(2*Izz))*(T1-T2)*l;
    % Derivatives
    dydt = zeros(6, 1);
    dydt(1:2) = [x_dot; y_dot]; % position derivatives
    dydt(3:4) = acc;          % velocity derivatives
    dydt(5) = theta_dot;
    dydt(6) = ang_acc;% angular velocity derivatives

end
