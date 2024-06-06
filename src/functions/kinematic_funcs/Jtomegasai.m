% rotational Jacobian of single-segment snak
%
function Jt_omega_sai = Jtomegasai(theta_tL , delta_t)

Jt_omega_sai = [-sin(delta_t) , cos(delta_t)*cos(theta_tL);...
    -cos(delta_t) , -sin(delta_t)*cos(theta_tL);...
    0 , -1+sin(theta_tL)];

end

