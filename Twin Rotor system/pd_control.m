function [u,error] = pd_control(val,val_d,error_prev,kp,kd,dt)
    error=val_d-val;
    u = kp*(error)+(kd*(error-error_prev)/(dt));
end