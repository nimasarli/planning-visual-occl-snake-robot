% Gradient of Bending energy of 3-seg snake w/o tools
% refer to actuation_compensation_turbt.pdf in notes folder for details
%
function gradU = getGradEnergy(psi,segmentNum)

% [L1,L2,L3,~,~,~,R_O_BB,~,~,~,OD_S1,ID_S1,OD_S2,ID_S2,...
%     OD_S3,ID_S3,~,~,THETA_0,BETA,...
%     ~,~,~,~,ES,~,...
%     ~,~] = setParam('TURBT constants');

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

IS_1 = (pi/64)*(OD_S1^4-ID_S1^4);
IS_2 = (pi/64)*(OD_S2^4-ID_S2^4);
IS_3 = (pi/64)*(OD_S3^4-ID_S3^4);

% Configuration variables parsing
theta1l = psi(1);
delta1 = psi(2);
theta2l = psi(3);
delta2 = psi(4);
theta3l = psi(5);
delta3 = psi(6);
% qIns = psi(7);

%
Eb_Vec = getEnergy(psi);
Eb1 = Eb_Vec(1);
Eb2 = Eb_Vec(2);
Eb3 = Eb_Vec(3);

% inverse kinematic
q = invKinThreeSegment(psi);
q11 = q(1);
q12 = q(2);
q13 = q(3);
q21 = q(4);
q22 = q(5);
q23 = q(6);
q31 = q(7);
q32 = q(8);
q33 = q(9);

% Secondary BB lengths
L11 = L1 + q11;
L12 = L1 + q12;
L13 = L1 + q13;

L21 = L2 + q21;
L22 = L2 + q22;
L23 = L2 + q23;

L31 = L3 + q31;
L32 = L3 + q32;
L33 = L3 + q33;

%
switch segmentNum
    case 1
        dU1_dTheta1l = (2/(theta1l-THETA_0))*Eb1 - (1/2)*(theta1l-THETA_0)^2*...
            ES*(IS_1+IS_2+IS_3)*(cos(delta1)/L11^2 + cos(delta1+BETA)/L12^2 + cos(delta1+2*BETA)/L13^2);
        dU1_dDelta1_snake_portion = (1/2)*(theta1l-THETA_0)^2*ES*R_O_BB*...
            (IS_1+IS_2+IS_3)*(sin(delta1)/L11^2 + sin(delta1+BETA)/L12^2 + sin(delta1+2*BETA)/L13^2);
        dU1_dDelta1_tool_portion = 0; %no tool for now
        dU1_dDelta1 = dU1_dDelta1_snake_portion + dU1_dDelta1_tool_portion;
        gradU = [dU1_dTheta1l;dU1_dDelta1];
    case 2
        dU2_dTheta2l = (2/(theta2l-THETA_0))*Eb2 - (1/2)*(theta2l-THETA_0)^2*...
            ES*(IS_2+IS_3)*(cos(delta2)/L21^2 + cos(delta2+BETA)/L22^2 + cos(delta2+2*BETA)/L23^2);
        dU2_dDelta2_snake_portion = (1/2)*(theta2l-THETA_0)^2*ES*R_O_BB*...
            (IS_2+IS_3)*(sin(delta2)/L21^2 + sin(delta2+BETA)/L22^2 + sin(delta2+2*BETA)/L23^2);
        dU2_dDelta2_tool_portion = 0; %no tool for now
        dU2_dDelta2 = dU2_dDelta2_snake_portion + dU2_dDelta2_tool_portion;
        gradU = [dU2_dTheta2l;dU2_dDelta2];
    case 3
        dU3_dTheta3l = (2/(theta3l-THETA_0))*Eb3 - (1/2)*(theta3l-THETA_0)^2*...
            ES*IS_3*(cos(delta3)/L31^2 + cos(delta3+BETA)/L32^2 + cos(delta3+2*BETA)/L33^2);
        dU3_dDelta3_snake_portion = (1/2)*(theta3l-THETA_0)^2*ES*R_O_BB*...
            IS_3*(sin(delta3)/L31^2 + sin(delta3+BETA)/L32^2 + sin(delta3+2*BETA)/L33^2);
        dU3_dDelta3_tool_portion = 0; %no tool for now
        dU3_dDelta3 = dU3_dDelta3_snake_portion + dU3_dDelta3_tool_portion;
        gradU = [dU3_dTheta3l;dU3_dDelta3];
        
    otherwise %exception handling
        gradU = [];
        error('wrong segment number');
end

end
