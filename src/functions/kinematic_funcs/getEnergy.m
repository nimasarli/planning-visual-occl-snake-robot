% Bending energy of each segment of 3-seg snake w/o tools
%
function Eb_Vec = getEnergy(psi)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
IP = (pi/64)*DP^4;
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

% Secondary BB lengths
L11 = L1 + R_O_BB*cos(delta1);
L12 = L1 + R_O_BB*cos(delta1+BETA);
L13 = L1 + R_O_BB*cos(delta1+2*BETA);

L21 = L2 + R_O_BB*cos(delta2);
L22 = L2 + R_O_BB*cos(delta2+BETA);
L23 = L2 + R_O_BB*cos(delta2+2*BETA);

L31 = L3 + R_O_BB*cos(delta3);
L32 = L3 + R_O_BB*cos(delta3+BETA);
L33 = L3 + R_O_BB*cos(delta3+2*BETA);

% Weighted Bending Energy Computation
Eb_1_prim = (theta1l-THETA_0)^2 * (EP*IP) /(2*L1);%bending energy of the first segment: primary backbone
Eb_1_secon = (1/2) * (theta1l-THETA_0)^2 * ES *(IS_1+IS_2+IS_3)...
    *(1/L11 + 1/L12 + 1/L13); %bending energy of the first segment: secondary backbones

Eb_2_prim = (theta2l-THETA_0)^2 * (EP*IP) /(2*L2);%bending energy of the 2nd segment: primary backbone
Eb_2_secon = (1/2) * (theta2l-THETA_0)^2 ...
    * ES *(IS_2+IS_3) * (1/L21 + 1/L22 + 1/L23);%bending energy of the 2nd segment: secondary backbones

Eb_3_prim = (theta3l-THETA_0)^2 * (EP*IP) /(2*L3);%bending energy of the 3rd segment: primary backbone
Eb_3_secon = (1/2) * (theta3l-THETA_0)^2 * ...
    ES *IS_3 * (1/L31 + 1/L32 + 1/L33);%bending energy of the 3rd segment: secondary backbones

Eb1 = Eb_1_prim + Eb_1_secon; %bending energy of 1st segment
Eb2 = Eb_2_prim + Eb_2_secon; %bending energy of 2nd segment
Eb3 = Eb_3_prim + Eb_3_secon; %bending energy of 3rd segment

Eb_Vec = [Eb1 ; Eb2 ; Eb3];

end
