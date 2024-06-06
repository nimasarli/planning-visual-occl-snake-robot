function d_Dc_d_Gamma = d_Dc_d_Gamma_Func(psi,lens_Rotation_Angle)

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

BETA = (pi/180)*LENS_FOV_DEGREE/2;
LENS_ANGLE = LENS_ANGLE_DEGREE*(pi/180);

% 
homTran04 = directKin(psi,0,4,L); %direct kinematic homogeneous
% transformation form frame 0 to 4 (base to end-effector frame)
point_Position = homTran04(1:3,4);

%
lens_Rotation_Angle_Degree = lens_Rotation_Angle*(180/pi);
point_Position_In_Lens_Frame = positionInLensFrame(point_Position,D_O,LENS_ANGLE_DEGREE,lens_Rotation_Angle_Degree);
dPsp_In_Lens_dGamma = [-cos(LENS_ANGLE)*sin(lens_Rotation_Angle),cos(LENS_ANGLE)*cos(lens_Rotation_Angle),0;...
    -cos(lens_Rotation_Angle),-sin(lens_Rotation_Angle),0;...
    sin(LENS_ANGLE)*sin(lens_Rotation_Angle),-sin(LENS_ANGLE)*cos(lens_Rotation_Angle),0]*point_Position;

%
d_dc_to_d_psp_in_Lens = d_dc_d_psp_In_Lens_Func(point_Position_In_Lens_Frame,BETA);
d_Dc_d_Gamma = d_dc_to_d_psp_in_Lens*dPsp_In_Lens_dGamma;



