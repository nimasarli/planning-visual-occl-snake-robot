function Q = config2Joint(psi)
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

% parsing psi
theta1l = psi(1);
delta1 = psi(2);
theta2l = psi(3);
delta2 = psi(4);
theta3l = psi(5);
delta3 = psi(6);
qIns = psi(7);

%
del_theta1 = theta1l - THETA_0;
del_theta2 = theta2l - THETA_0;
del_theta3 = theta3l - THETA_0;

%
delta_12 = delta1 + BETA; % delta_ti = delta_t + (i-1)*beta;
delta_13 = delta1 + 2*BETA;
delta_22 = delta2 + BETA; % delta_ti = delta_t + (i-1)*beta;
delta_23 = delta2 + 2*BETA;
delta_32 = delta3 + BETA; % delta_ti = delta_t + (i-1)*beta;
delta_33 = delta3 + 2*BETA;

%
Delta_11 = R_O_BB*cos(delta1); % Delta_ti = r_b*cos(delta_ti);
Delta_12 = R_O_BB*cos(delta_12);
Delta_13 = R_O_BB*cos(delta_13);
Delta_21 = R_O_BB*cos(delta2); % Delta_ti = r_b*cos(delta_ti);
Delta_22 = R_O_BB*cos(delta_22);
Delta_23 = R_O_BB*cos(delta_23);
Delta_31 = R_O_BB*cos(delta3); % Delta_ti = r_b*cos(delta_ti);
Delta_32 = R_O_BB*cos(delta_32);
Delta_33 = R_O_BB*cos(delta_33);

%
q_11 = Delta_11 * del_theta1; % q_ti = Delta_ti * del_theta_t;
q_12 = Delta_12 * del_theta1;
q_13 = Delta_13 * del_theta1;
q_21 = Delta_21 * del_theta2; % q_ti = Delta_ti * del_theta_t;
q_22 = Delta_22 * del_theta2;
q_23 = Delta_23 * del_theta2;
q_31 = Delta_31 * del_theta3; % q_ti = Delta_ti * del_theta_t;
q_32 = Delta_32 * del_theta3;
q_33 = Delta_33 * del_theta3;

%
Q = [q_11 ; q_12 ; q_13 ; q_21 ; q_22 ; q_23 ; q_31 ; q_32 ; q_33 ; qIns];

end