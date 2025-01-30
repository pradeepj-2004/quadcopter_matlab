function [psi_d,theta_d] = outerController(x_dot_d,x_dot,x_ddot_d,x_ddot,kp_x_dot,kd_x_dot,y_dot_d,y_dot,y_ddot_d,y_ddot,kp_y_dot,kd_y_dot)
    %OUTERCONTROLLER 
    psi_d=kp_x_dot*(x_dot_d-x_dot)+kd_x_dot*(x_ddot_d-x_ddot);
    theta_d=kp_y_dot*(y_dot_d-y_dot)+kd_y_dot*(y_ddot_d-y_ddot);

end