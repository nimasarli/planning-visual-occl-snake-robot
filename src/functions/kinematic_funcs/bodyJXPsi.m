% Jacobian of an arbitrary point on the body of TURBT snake
% s is the arc length on the snake primary backbone with sOverL = 0 the beginnig
% and sOverL = 1 the end of the snake segment
% point_s = [segNum,sOverL]
% 
% Author: Nima Sarli
% Date: 8/8/2016
%
function J_X_Psi_S = bodyJXPsi(Psi,point_s)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

% parsing Inputs
theta1l = Psi(1);
delta1 = Psi(2);
theta2l = Psi(3);
delta2 = Psi(4);
theta3l = Psi(5);
delta3 = Psi(6);
% qIns = Psi(7);
segNum = point_s(1);
sOverL = point_s(2);

% Exception Handling
if ~any(segNum == [1,2,3])
    error('Segment Number cab be 1,2 or 3')
end

if (sOverL < 0) || (sOverL > 1)
    error('This must hold: 0 <= sOverL <= 1');
end

%
if segNum == 1
    % J1s_V_Psi
    J1s_V_Psi = JsVPsi(theta1l,delta1,sOverL,L1);
    % J1s_omega_Psi
    J1s_omega_Psi = JsOmegaPsi(theta1l,delta1,sOverL);
    %
    J1s = [J1s_V_Psi ; J1s_omega_Psi];
    J4 = [0 0 1 0 0 0]';
    %
    homTran01 = directKin(Psi,0,1,L);
    R01 = homTran01(1:3,1:3);
    %
    S1 = [R01 zeros(3);...
        zeros(3) R01];
    % J_X_Psi_S
    J_X_Psi_S = [S1*J1s , zeros(6,2) , zeros(6,2) , J4];  
elseif segNum == 2
    % Jtvpsi
    J1_v_sai = Jtvsai(theta1l,delta1,L1);
    J2s_V_Psi = JsVPsi(theta2l,delta2,sOverL,L2);
    % Jtomegasai
    J1_omega_sai = Jtomegasai(theta1l,delta1);
    J2s_omega_Psi = JsOmegaPsi(theta2l,delta2,sOverL);
    %
    J1 = [J1_v_sai ; J1_omega_sai];
    J2s = [J2s_V_Psi ; J2s_omega_Psi];
    J4 = [0 0 1 0 0 0]';
    %
    homTran01 = directKin(Psi,0,1,L);
    R01 = homTran01(1:3,1:3);
    homTran12 = directKin(Psi,1,2,L);
    R12 = homTran12(1:3,1:3);
    homTran02 = directKin(Psi,0,2,L);
    R02 = homTran02(1:3,1:3);
    %
    s = sOverL*L2;
    thetaS = THETA_0 - sOverL*(THETA_0-theta2l);
    P_2_2s = Pbt_tl(thetaS,delta2,s);
    vec1 = R12*P_2_2s;
    cross_prod_mat_vec1 = crossProd_mat(vec1);
    %
    S1 = [R01 -R01*cross_prod_mat_vec1;...
        zeros(3,3) R01];
    S2 = [R02 zeros(3,3);...
        zeros(3,3) R02];
    % J_X_Psi_S
    J_X_Psi_S = [S1*J1 , S2*J2s , zeros(6,2) , J4]; 
elseif segNum == 3
    % Jtvpsi
    J1_v_sai = Jtvsai(theta1l,delta1,L1);
    J2_v_sai = Jtvsai(theta2l,delta2,L2);
    J3s_V_Psi = JsVPsi(theta3l,delta3,sOverL,L3);
    % Jtomegasai
    J1_omega_sai = Jtomegasai(theta1l,delta1);
    J2_omega_sai = Jtomegasai(theta2l,delta2);
    J3s_omega_Psi = JsOmegaPsi(theta3l,delta3,sOverL);
    %
    J1 = [J1_v_sai ; J1_omega_sai];
    J2 = [J2_v_sai ; J2_omega_sai];
    J3s = [J3s_V_Psi ; J3s_omega_Psi];
    J4 = [0 0 1 0 0 0]';
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
    P_2_2L = Pbt_tl(theta2l , delta2 , L2);
    s = sOverL*L3;
    thetaS = THETA_0 - sOverL*(THETA_0-theta3l);
    P_3_3s = Pbt_tl(thetaS,delta3,s);
    %
    vec1 = R12*P_2_2L;
    vec2 = R13*P_3_3s;
    vec3 = R23*P_3_3s;
    cross_prod_mat_vec1 = crossProd_mat(vec1);
    cross_prod_mat_vec2 = crossProd_mat(vec2);
    cross_prod_mat_vec3 = crossProd_mat(vec3);
    %
    S1 = [R01 -R01*(cross_prod_mat_vec1+cross_prod_mat_vec2);...
        zeros(3,3) R01];
    S2 = [R02 -R02*cross_prod_mat_vec3;...
        zeros(3,3) R02];
    S3 = [R03 zeros(3,3);...
        zeros(3,3) R03];
    % J_X_Psi_S
    J_X_Psi_S = [S1*J1 , S2*J2 , S3*J3s , J4];
end

end