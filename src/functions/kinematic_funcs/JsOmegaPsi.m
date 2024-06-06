% rotational Jacobian of arbitrary point on signle-segment snake 
%
function Js_Omega_Psi = JsOmegaPsi(theta_tL,delta_t,sOverL)
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

%
del_theta_t = THETA_0 - theta_tL;
thetaS = THETA_0 - sOverL*del_theta_t;

%
Js_Omega_Psi = [-sin(delta_t) , cos(delta_t)*cos(thetaS);...
    -cos(delta_t) , -sin(delta_t)*cos(thetaS);...
    0 , -1+sin(thetaS)];

end

