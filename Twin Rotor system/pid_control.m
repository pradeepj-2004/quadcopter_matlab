function [u,error_d,error_in] = pid_control(val,val_d,error_prev,error_i,kp,ki,kd,dt)
    error_d=val_d-val;
    u = kp*(error_d)+(kd*(error_d-error_prev)/(dt))+ki*error_i;
    error_in=error_i+error_d;
end