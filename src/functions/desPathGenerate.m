% generates task
%
function [P_L_des,R_des] = desPathGenerate(time,modeString)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

% tool constants
[EI_FIBERSCOPE,EI_GRIPPER,EI_LASER,EP_MICROSNAKE,...
    ES_MICROSNAKE,OD_BB_MICROSNAKE,ID_BB_MICROSNAKE,L_MICROSNAKE, L_S_MICROSNAKE,...
    R_O_BB_MICROSNAKE, R_DISK_MICROSNAKE,...
    DP_MICROSNAKE, D_HEIGHT_ED_MICROSNAKE, D_HEIGHT_SD_MICROSNAKE,...
    N_DISKS_MICROSNAKE,D_O,LENS_FOV_DEGREE,LENS_ANGLE_DEGREE] = ...
    setParam('tool parameters');
LENS_ANGLE = LENS_ANGLE_DEGREE*(pi/180);

% potetntial method constants
[D_INFL,R_STAR,F_STAR,P,Q,CONE_HEIGHT] = ...
    setParam('potential field constants');
CONE_BASE_RADIUS = CONE_HEIGHT*tan(0.5*LENS_FOV_DEGREE*(pi/180));

switch modeString
    case 'circle of R10 facing up'
        r = 10/1000;
        omega = pi/4;
        R = [1 0 0;...
            0 1 0;...
            0 0 1];
        Porg = [0;D_O;45/1000];
        theta = omega*time;
        xPath = r*cos(theta);
        yPath = r*sin(theta);
        zPath = 0;
        
        P_L_des = R*[xPath;yPath;zPath] + Porg;
        R_des = [1,0,0;...
            0,1,0;...
            0,0,1]; %keep the snake facing toward z0
    case 'circle of R10 facing inside plane of view'
        r = 10/1000;
        omega = pi/2;
        R = [1 0 0;...
            0 0 -1;...
            0 1 0];
        Porg = [0;D_O;45/1000];
        theta = omega*time;
        xPath = r*cos(theta);
        yPath = r*sin(theta);
        zPath = 0;
        
        P_L_des = R*[xPath;yPath;zPath] + Porg;
        R_des = [0,1,0;...
            0,0,1;...
            -1,0,0]; %keep the snake facing toward y0
    case 'Circle - Angled Cone'
        r = 10/1000;
        cone_Vertex = [0;D_O;0];
        rot_W_Lens = rotationWorldLens(LENS_ANGLE,0);
        Porg = cone_Vertex + 0.75*CONE_HEIGHT*rot_W_Lens(:,3);
        omega = pi/2;
        theta = omega*time;
        xPath = r*cos(theta);
        yPath = r*sin(theta);
        zPath = 0;
        
        P_L_des = rot_W_Lens*[xPath;yPath;zPath] + Porg;
        R_des = rot_W_Lens;
    case 'Keep the Tip fixed'
        P_L_des = [0;D_O;45/1000];
        R_des = [1,0,0;...
            0,1,0;...
            0,0,1]; %keep the snake facing toward z0
    case 'Keep the Tip fixed - cone face'
        P_L_des = [0;D_O+tan(0.5*55*(pi/180))*(45/1000);45/1000];
        R_des = [1,0,0;...
            0,1,0;...
            0,0,1]; %keep the snake facing toward z0
    case 'Constant pose for angled endoscope'
        position_In_Lens_Frame = [0,0,45/1000]';
        P_L_des = positionInWorldFrame(position_In_Lens_Frame,D_O,LENS_ANGLE_DEGREE,0);
        R_des = rotationWorldLens(LENS_ANGLE_DEGREE*(pi/180),0);
    case 'Trace an offset line'
        theta = 0*(pi/180); %15*(pi/180)
        z_des = 45/1000;
        x_des = z_des*tan(theta);
        v_y_des = -3/1000;
        y_des_0 = 7/1000;
        y_des = v_y_des*time + y_des_0;
        P_L_Des_In_Cone_Frame = [x_des;y_des;z_des];
        
        cone_Vertex = [0;D_O;0];
        Porg = cone_Vertex;
        rot_W_Lens = rotationWorldLens(LENS_ANGLE,0);
        
        P_L_des = rot_W_Lens*P_L_Des_In_Cone_Frame + Porg;
        R_des = rot_W_Lens;
        %         R_des = [1,0,0;...
        %             0,1,0;...
        %             0,0,1]; %keep the snake facing toward z0
    otherwise
        P_L_des = [];
        R_des = [];
        error('Wrong choice of mode in desPathGenerate(time,mode)!');
        
        
end
