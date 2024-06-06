%successive rotaiom matrix from segment t (b_t) to segment t+1 (gt or bt+1)
function rotMat = R_bi_gi(theta_tL , delta_t)
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

%
rotMat = rotZ(-delta_t)*rotY(THETA_0-theta_tL)*rotZ(delta_t);
end

