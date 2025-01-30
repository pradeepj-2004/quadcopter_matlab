function dydt = droneDynamics(t, y, u,m,g,Ixx,Iyy,Izz,kt,kr)

    % State variables
    x_pos = y(1); y_pos = y(2); z_pos = y(3); % position
    vx = y(4); vy = y(5); vz = y(6); % velocity
    phi = y(7); theta = y(8); psi = y(9); % orientation
    p = y(10); q = y(11); r = y(12); % angular velocity

    T = u(1); % total thrust


    % Translational dynamics
    acc=[(1/(m))*((cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*T -kt*vx);
         (1/(m))*((cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi))*T -kt*vy) ;
         (1/(m))*((cos(phi)*cos(theta))*T - m*g - kt*vz)];
    
    % Rotational Dynamics
    ang_acc =[(1/Ixx)*(-kr*p -q*r*( Izz - Iyy)+u(2)) ;
              (1/Iyy)*(-kr*q-p*r*( Ixx - Izz)+u(3));
              (1/Izz)*(-kr*r-p*q*( Iyy - Ixx)+u(4))];

    % Derivatives
    dydt = zeros(12, 1);
    dydt(1:3) = [vx; vy; vz]; % position derivatives
    dydt(4:6) = acc;          % velocity derivatives
    dydt(7:9) = [p+(q*sin(phi)*tan(theta))+(r*cos(phi)*tan(theta));(q*cos(phi))-(r*sin(phi));(1/cos(theta))*(q*sin(phi)+r*cos(phi))]; % orientation derivatives
    dydt(10:12) = ang_acc; % angular velocity derivatives
end