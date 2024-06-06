% desired twist for 3DOF position and 2DOF(neglect roll) orientation
%
function x_des_dt = desiredTwist5Dof(P_L_cur,R_cur,P_L_des,R_des)

% resolved rate parameters
V_MIN = 0.5/1000;
V_MAX = 60/1000;
EPSILON_P = 0.1/1000;
KISI_MIN = 0.5*pi/180;
KISI_MAX = 30*pi/180;
EPSILON_KISI = pi/180;
LAMBDA = 3;

%
delta_p = norm(P_L_des - P_L_cur); %position error
delta_p_mm = 1000*delta_p,
theta_e = acos(R_cur(:,3)'*R_des(:,3)); % orientation error
if delta_p < EPSILON_P
    v_tilde_norm = 0;%goal reached
    disp('position goal reached');
elseif delta_p > LAMBDA*EPSILON_P
    v_tilde_norm = V_MAX; %approach with max linear velocity
else
    v_tilde_norm = (V_MAX-V_MIN)*(delta_p-EPSILON_P)/(EPSILON_P*(LAMBDA-1)) + V_MIN;
end
nhat = (P_L_des - P_L_cur)/delta_p;
p_des_dt = v_tilde_norm * nhat;
%p_des_dt = 100*delta_p * nhat;

if theta_e < EPSILON_KISI
    kisi_d_norm = 0; %goal reaached
    disp('orientation goal reached');%
elseif theta_e > LAMBDA*EPSILON_KISI
    kisi_d_norm = KISI_MAX; %approach with max angular velocity
else
    kisi_d_norm = (KISI_MAX-KISI_MIN)*(theta_e-EPSILON_KISI)/(EPSILON_KISI*(LAMBDA-1)) + KISI_MIN;
end

m_e = cross(R_cur(:,3),R_des(:,3));
m_e_hat = m_e/(norm(m_e));
kisi_des_dt = kisi_d_norm * m_e_hat;

x_des_dt = [p_des_dt ; kisi_des_dt];

end


