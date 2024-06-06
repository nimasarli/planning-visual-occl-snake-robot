% gives the line of sight where 
%   p0_ls: a point on the line (camera origin)
%   u_ls: direction unit vector of line of sight
%
function [p0_ls,u_ls] = lineOfSight(psi)

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

T04 = directKin(psi,0,4,L);
P04 = T04(1:3,4);
lens_rotation_angle_degree = 0; %Any number
p0_ls = positionInWorldFrame(zeros(3,1),D_O,LENS_ANGLE_DEGREE,lens_rotation_angle_degree); %lens coordinate in world
u_ls = (P04-p0_ls)/norm(P04-p0_ls); % dir unit vec of line of sight

end

