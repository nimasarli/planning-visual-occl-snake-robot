%#eml
% Determines inverse kinemtics of a three-segment snake from configuration space to
% joint space with independant desoupled actuators (No Actuator is mounted
% on another one)
%
function q = invKinThreeSegment(psi)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

% Configuration variables parsing
theta1l = psi(1);
delta1 = psi(2);
theta2l = psi(3);
delta2 = psi(4);
theta3l = psi(5);
delta3 = psi(6);

%
q11 = R_O_BB*cos(delta1)*(theta1l-THETA_0);
q12 = R_O_BB*cos(delta1+BETA)*(theta1l-THETA_0);
q13 = R_O_BB*cos(delta1+2*BETA)*(theta1l-THETA_0);

q21 = R_O_BB*cos(delta2)*(theta2l-THETA_0);
q22 = R_O_BB*cos(delta2+BETA)*(theta2l-THETA_0);
q23 = R_O_BB*cos(delta2+2*BETA)*(theta2l-THETA_0);

q31 = R_O_BB*cos(delta3)*(theta3l-THETA_0);
q32 = R_O_BB*cos(delta3+BETA)*(theta3l-THETA_0);
q33 = R_O_BB*cos(delta3+2*BETA)*(theta3l-THETA_0);

%
q = [q11;q12;q13;q21;q22;q23;q31;q32;q33];