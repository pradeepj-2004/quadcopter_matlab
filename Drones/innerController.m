function [u1,u2,u3,T] = innerController(phi_d,phi,phi_dot_d,phi_dot,kp_phi,kd_phi,theta_d,theta,theta_dot_d,theta_dot,kp_theta,kd_theta,psi_d,psi,psi_dot_d,psi_dot,kp_psi,kd_psi,z_dot_d,z_dot,z_ddot_d,z_ddot,kp_z_dot,kd_z_dot)
    u1 = kp_phi*(phi_d-phi)+kd_phi*(phi_dot_d-phi_dot);
    u2 = kp_theta*(theta_d-theta)+kd_theta*(theta_dot_d-theta_dot);
    u3 = kp_psi*(psi_d-psi)+kd_psi*(psi_dot_d-psi_dot);
    T = kp_z_dot*(z_dot_d-z_dot)+kd_z_dot*(z_ddot_d-z_ddot);
end