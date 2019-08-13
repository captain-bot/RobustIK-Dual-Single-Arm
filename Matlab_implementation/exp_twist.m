function gab = exp_twist(xi, theta)
% Function to compute the exponential of a twist
omega = xi(4:6,1);
v = xi(1:3,1);

if (norm(omega) <= eps)
    omega_hat = eye(3,3);
    p_val = v*theta;
else
    omega_hat = [0 -omega(3) omega(2); ...
                 omega(3) 0 -omega(1); ...
                -omega(2) omega(1) 0];
    omega_hat_theta = eye(3,3) + omega_hat*sin(theta) + omega_hat^2*(1 - cos(theta));
    p_val = (eye(3,3) - omega_hat_theta)*cross(omega, v) + omega*omega'*v*theta;
end
last_row = [zeros(1,3) 1];
gab = [omega_hat_theta p_val];
gab = [gab; last_row];
end
