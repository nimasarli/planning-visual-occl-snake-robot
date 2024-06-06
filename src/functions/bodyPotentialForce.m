% Outputs:
% body_potential_Force
%                  The repulsive potential force on an arbitraty [x0,y0,z0] point in space
%                  (described in {lens} frame) due to a conical shape at an arbitrary orientation
% dForce_dGamma
%                  The gradient of repulsive potential force on an arbitraty [x0,y0,z0] point in space
%                  (described in {lens} frame) due to a conical shape at an arbitrary orientation
%
% MEDIT borescope specs are used
% STORZ endoscope specs: model 27020 AA diamater 2.9 mm length 300 mm
%
% point_Position_In_Lens_Frame is in {lens} frame
%
function [body_potential_Force,dForce_dGamma] = bodyPotentialForce(point_Position_In_Lens_Frame,lens_Rotation_Angle)

% tool constants
[EI_FIBERSCOPE,EI_GRIPPER,EI_LASER,EP_MICROSNAKE,...
    ES_MICROSNAKE,OD_BB_MICROSNAKE,ID_BB_MICROSNAKE,L_MICROSNAKE, L_S_MICROSNAKE,...
    R_O_BB_MICROSNAKE, R_DISK_MICROSNAKE,...
    DP_MICROSNAKE, D_HEIGHT_ED_MICROSNAKE, D_HEIGHT_SD_MICROSNAKE,...
    N_DISKS_MICROSNAKE,D_O,LENS_FOV_DEGREE,LENS_ANGLE_DEGREE] = ...
    setParam('tool parameters');
BETA = (pi/180)*LENS_FOV_DEGREE/2;

% potetntial method constants
[D_INFL,R_STAR,F_STAR,P,Q,CONE_HEIGHT] = ...
    setParam('potential field constants');
%CONE_BASE_RADIUS = CONE_HEIGHT*tan(0.5*LENS_FOV_DEGREE*(pi/180));

% Parsing input
x_0 = point_Position_In_Lens_Frame(1);
y_0 = point_Position_In_Lens_Frame(2);
z_0 = point_Position_In_Lens_Frame(3);

%
d = cos(BETA)*(x_0^2+y_0^2)^0.5 - sin(BETA)*z_0;
r = sin(BETA)*(x_0^2+y_0^2)^0.5 + cos(BETA)*z_0;
LENS_ANGLE = LENS_ANGLE_DEGREE*(pi/180);
rot_W_Lens = rotationWorldLens(LENS_ANGLE,lens_Rotation_Angle);
f_hat_in_Lens = [cos(BETA)*x_0/(x_0^2+y_0^2)^0.5;...
    cos(BETA)*y_0/(x_0^2+y_0^2)^0.5;...
    -sin(BETA)];
f_hat = rot_W_Lens*f_hat_in_Lens;

%
lens_Rotation_Angle_Degree = lens_Rotation_Angle*(180/pi);
point_Position = positionInWorldFrame(point_Position_In_Lens_Frame,D_O,LENS_ANGLE_DEGREE,lens_Rotation_Angle_Degree);
dPsp_In_Lens_dGamma = [-cos(LENS_ANGLE)*sin(lens_Rotation_Angle),cos(LENS_ANGLE)*cos(lens_Rotation_Angle),0;...
    -cos(lens_Rotation_Angle),-sin(lens_Rotation_Angle),0;...
    sin(LENS_ANGLE)*sin(lens_Rotation_Angle),-sin(LENS_ANGLE)*cos(lens_Rotation_Angle),0]*point_Position;

% output
if (d <= D_INFL) && (r >= 0)
    body_potential_Force = (F_STAR*R_STAR^Q/r^Q) * (d/D_INFL-1)^(2*P) * f_hat;
    dForce_dPsp_in_Lens = df_dPsp_in_Lens_Func(point_Position_In_Lens_Frame,BETA,R_STAR,F_STAR,Q,P,D_INFL,lens_Rotation_Angle,LENS_ANGLE);
    dForce_dGamma = dForce_dPsp_in_Lens*dPsp_In_Lens_dGamma;
else %outside distance of influence (D_INFL)
    body_potential_Force = zeros(3,1);
    dForce_dGamma = zeros(3,1);
end

