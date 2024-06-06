function [r,d] = conicalParameters(position_In_World_Frame,lens_Rotation_Angle)

% tool constants
[EI_FIBERSCOPE,EI_GRIPPER,EI_LASER,EP_MICROSNAKE,...
    ES_MICROSNAKE,OD_BB_MICROSNAKE,ID_BB_MICROSNAKE,L_MICROSNAKE, L_S_MICROSNAKE,...
    R_O_BB_MICROSNAKE, R_DISK_MICROSNAKE,...
    DP_MICROSNAKE, D_HEIGHT_ED_MICROSNAKE, D_HEIGHT_SD_MICROSNAKE,...
    N_DISKS_MICROSNAKE,D_O,LENS_FOV_DEGREE,LENS_ANGLE_DEGREE] = ...
    setParam('tool parameters');
BETA = (pi/180)*LENS_FOV_DEGREE/2;

%
lens_Rotation_Angle_Degree = lens_Rotation_Angle*(180/pi);
point_Position_In_Lens_Frame = positionInLensFrame(position_In_World_Frame,D_O,LENS_ANGLE_DEGREE,lens_Rotation_Angle_Degree);
x_0 = point_Position_In_Lens_Frame(1);
y_0 = point_Position_In_Lens_Frame(2);
z_0 = point_Position_In_Lens_Frame(3);

%
r = sin(BETA)*(x_0^2+y_0^2)^0.5 + cos(BETA)*z_0;
d = cos(BETA)*(x_0^2+y_0^2)^0.5 - sin(BETA)*z_0;

