function J_x_psi = Jxsai(Psi)
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

% parsing psi
theta1l = Psi(1);
delta1 = Psi(2);
theta2l = Psi(3);
delta2 = Psi(4);
theta3l = Psi(5);
delta3 = Psi(6);
% qIns = Psi(7);

% Should J_x_sai be defined seperately when theta_L = pi/2 ?
% Jtvpsi
J1_v_sai = Jtvsai(theta1l , delta1 , L1);
J2_v_sai = Jtvsai(theta2l , delta2 , L2);
J3_v_sai = Jtvsai(theta3l , delta3 , L3);

% Jtomegasai
J1_omega_sai = Jtomegasai(theta1l , delta1);
J2_omega_sai = Jtomegasai(theta2l , delta2);
J3_omega_sai = Jtomegasai(theta3l , delta3);

%
J1 = [J1_v_sai ; J1_omega_sai];
J2 = [J2_v_sai ; J2_omega_sai];
J3 = [J3_v_sai ; J3_omega_sai];
J4 = [0 0 1 0 0 0].';

%
homTran01 = directKin(Psi,0,1,L);
R01 = homTran01(1:3,1:3);
homTran12 = directKin(Psi,1,2,L);
R12 = homTran12(1:3,1:3);
homTran23 = directKin(Psi,2,3,L);
R23 = homTran23(1:3,1:3);
homTran13 = directKin(Psi,1,3,L);
R13 = homTran13(1:3,1:3);
homTran02 = directKin(Psi,0,2,L);
R02 = homTran02(1:3,1:3);
homTran03 = directKin(Psi,0,3,L);
R03 = homTran03(1:3,1:3);

%
homTran23 = directKin(Psi,2,3,L); %consider using Pbt_tl function for faster performance
P_2_2L = homTran23(1:3,4);
homTran34 = directKin(Psi,3,4,L);
P_3_3L = homTran34(1:3,4);

%
vec1 = R12*P_2_2L;
vec2 = R13*P_3_3L;
vec3 = R23*P_3_3L;
cross_prod_mat_vec1 = crossProd_mat(vec1);
cross_prod_mat_vec2 = crossProd_mat(vec2);
cross_prod_mat_vec3 = crossProd_mat(vec3);

%
S1 = [R01 -R01*(cross_prod_mat_vec1+cross_prod_mat_vec2);...
    zeros(3) R01];
S2 = [R02 -R02*cross_prod_mat_vec3;...
    zeros(3) R02];
S3 = [R03 zeros(3);...
    zeros(3) R03];

% Jxpsi
J_x_psi = [S1*J1 , S2*J2 , S3*J3 , J4];

end

