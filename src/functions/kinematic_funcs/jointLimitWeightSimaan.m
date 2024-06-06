%#eml
% Implementing weights based on discussion with Nabil. See
% notes/2016_06_14_whiteboard_joint_limit_avoidance_turbt.pdf
%
function W = jointLimitWeightSimaan(psi,psi_dot,PSI_MAX,PSI_MIN)

%
M_THETA1L = 20; %20
M_THETA2L = 20; %20
M_THETA3L = 20; %20
P_THETA1L = 20; %20
P_THETA2L = 20; %20
P_THETA3L = 20; %20

%
M_DELTA1 = 0; %0
M_DELTA2 = 0; %0
M_DELTA3 = 0; %0
P_DELTA1 = 20; %20
P_DELTA2 = 20; %20
P_DELTA3 = 20; %20

% Joint Limits
THETA_1L_MAX = PSI_MAX(1);
THETA_2L_MAX = PSI_MAX(3);
THETA_3L_MAX = PSI_MAX(5);
THETA_1L_MIN = PSI_MIN(1);
THETA_2L_MIN = PSI_MIN(3);
THETA_3L_MIN = PSI_MIN(5);

% Joint Limits
DELTA_1_MAX = PSI_MAX(2);
DELTA_2_MAX = PSI_MAX(4);
DELTA_3_MAX = PSI_MAX(6);
DELTA_1_MIN = PSI_MIN(2);
DELTA_2_MIN = PSI_MIN(4);
DELTA_3_MIN = PSI_MIN(6);

%
THETA_1L_MID = (THETA_1L_MAX + THETA_1L_MIN)/2;
THETA_1L_WIDTH = (THETA_1L_MAX - THETA_1L_MIN);
THETA_2L_MID = (THETA_2L_MAX + THETA_2L_MIN)/2;
THETA_2L_WIDTH = (THETA_2L_MAX - THETA_2L_MIN);
THETA_3L_MID = (THETA_3L_MAX + THETA_3L_MIN)/2;
THETA_3L_WIDTH = (THETA_3L_MAX - THETA_3L_MIN);

%
DELTA_1_MID = (DELTA_1_MAX + DELTA_1_MIN)/2;
DELTA_1_WIDTH = (DELTA_1_MAX - DELTA_1_MIN);
DELTA_2_MID = (DELTA_2_MAX + DELTA_2_MIN)/2;
DELTA_2_WIDTH = (DELTA_2_MAX - DELTA_2_MIN);
DELTA_3_MID = (DELTA_3_MAX + DELTA_3_MIN)/2;
DELTA_3_WIDTH = (DELTA_3_MAX - DELTA_3_MIN);

% parsing input
theta_1L = psi(1);
delta_1 = psi(2);
theta_2L = psi(3);
delta_2 = psi(4);
theta_3L = psi(5);
delta_3 = psi(6);
theta_1L_dt = psi_dot(1);
delta_1_dt = psi_dot(2);
theta_2L_dt = psi_dot(3);
delta_2_dt = psi_dot(4);
theta_3L_dt = psi_dot(5);
delta_3_dt = psi_dot(6);

%%% theta_1L
if theta_1L_dt*(theta_1L-THETA_1L_MID) >= 0 %moving toward joint limit
    w_theta1L = 30 + M_THETA1L*(2^(2*P_THETA1L))*((theta_1L-THETA_1L_MID)/THETA_1L_WIDTH)^(2*P_THETA1L);
else
    w_theta1L = 30;
end

%%% theta_2L
if theta_2L_dt*(theta_2L-THETA_2L_MID) >= 0 %moving toward joint limit
    w_theta2L = 30 + M_THETA2L*(2^(2*P_THETA2L))*((theta_2L-THETA_2L_MID)/THETA_2L_WIDTH)^(2*P_THETA2L);
else
    w_theta2L = 30;
end

%%% theta_3L
if theta_3L_dt*(theta_3L-THETA_3L_MID) >= 0 %moving toward joint limit
    w_theta3L = 30 + M_THETA3L*(2^(2*P_THETA3L))*((theta_3L-THETA_3L_MID)/THETA_3L_WIDTH)^(2*P_THETA3L);
else
    w_theta3L = 30;
end

%%% delta_1
if delta_1_dt*(delta_1-DELTA_1_MID) >= 0 %moving toward joint limit
    w_delta1 = 30 + M_DELTA1*(2^(2*P_DELTA1))*((delta_1-DELTA_1_MID)/DELTA_1_WIDTH)^(2*P_DELTA1);
else
    w_delta1 = 30;
end

%%% delta_2
if delta_2_dt*(delta_2-DELTA_2_MID) >= 0 %moving toward joint limit
    w_delta2 = 30 + M_DELTA2*(2^(2*P_DELTA2))*((delta_2-DELTA_2_MID)/DELTA_2_WIDTH)^(2*P_DELTA2);
else
    w_delta2 = 30;
end

%%% delta_3
if delta_3_dt*(delta_3-DELTA_3_MID) >= 0 %moving toward joint limit
    w_delta3 = 30 + M_DELTA3*(2^(2*P_DELTA3))*((delta_3-DELTA_3_MID)/DELTA_3_WIDTH)^(2*P_DELTA3);
else
    w_delta3 = 30;
end

% output
W = diag([w_theta1L,w_delta1,w_theta2L,w_delta2,w_theta3L,w_delta3]);
