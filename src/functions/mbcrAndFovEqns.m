% Equations of points on MBCR and cone of FOV to solve by fzero that gives the
% points of intersection.
% 
function  [d,r] = mbcrAndFovEqns(s,segNum,phi)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

% tool constants
[EI_FIBERSCOPE,EI_GRIPPER,EI_LASER,EP_MICROSNAKE,...
    ES_MICROSNAKE,OD_BB_MICROSNAKE,ID_BB_MICROSNAKE,L_MICROSNAKE, L_S_MICROSNAKE,...
    R_O_BB_MICROSNAKE, R_DISK_MICROSNAKE,...
    DP_MICROSNAKE, D_HEIGHT_ED_MICROSNAKE, D_HEIGHT_SD_MICROSNAKE,...
    N_DISKS_MICROSNAKE,D_O,LENS_FOV_DEGREE,LENS_ANGLE_DEGREE] = ...
    setParam('tool parameters');

%%
psi = phi(1:7);
if segNum == 3
    point_s = [segNum,s/L3];
elseif segNum == 2
    point_s = [segNum,s/L2];
elseif segNum == 1
    point_s = [segNum,s/L1];
end

homTran_0_s = bodyDirectKin(psi,0,point_s,L);
p_From_World_Frame_To_S = homTran_0_s(1:3,4);

lens_Rotation_Angle = phi(8);
[r,d] = conicalParameters(p_From_World_Frame_To_S,lens_Rotation_Angle);

end

