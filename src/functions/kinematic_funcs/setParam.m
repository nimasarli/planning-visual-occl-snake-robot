% parameters of the simulation
% Last Edited 08/05/2016
%
function varargout = setParam(string)
switch string
    case 'TURBT constants'
        L1 = 26/1000; %first segment length 5/5/2016
        L2 = 20/1000; %2nd segment length 5/5/2016
        L3 = 19/1000; %3rd segment length 6/2/2016
        Ls1_1 = 780/1000; %secondary backbone length from A/U to 1st segment EE
        Ls1_2 = Ls1_1+L2+40/1000; %secondary backbone length from A/U to 2nd segment EE
        Ls1_3 = Ls1_1+L2+L3+80/1000; %secondary backbone length from A/U to 3rd segment EE
        R_O_BB = 1.725/1000; %kinematic radius
        R_O_CH = 1.2/1000;
        R_DISK = 2.5/1000;
        DP = 0;
        OD_S1 = 0.6604/1000; %first segment secondary backbone OD
        ID_S1 = 0.4826/1000; %first segment secondary backbone ID
        OD_S2 = 0.4318/1000; %second segment secondary backbone OD
        ID_S2 = 0.2794/1000; %second segment secondary backbone ID
        OD_S3 = 0.254/1000; %third segment secondary backbone OD
        ID_S3 = 0;          %third segment secondary backbone ID
        D_HEIGHT_ED = 3.08/1000;
        D_HEIGHT_SD = 3.08/1000;
        THETA_0 = pi/2;
        BETA = 120*(pi/180);
        % D_PRIMARY = 0.3/1000;
        N_DISKS_SEG1 = ceil(0.5*round(L1/D_HEIGHT_SD)); %spacer + end disk
        N_DISKS_SEG2 = ceil(0.5*round(L2/D_HEIGHT_SD));
        N_DISKS_SEG3 = ceil(0.5*round(L3/D_HEIGHT_SD));
        EP = 0.5e9; %primary elastic modulus (PTFE)
        ES = 42e9; %secondary elastic modulus (NiTi)
        MAX_STRAIN = 0.05;
        varargout = {L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
            OD_S1,ID_S1,OD_S2,ID_S2,...
            OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
            N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN};
    case 'optimization parameters'
        wAngle = 10; %1
        wDistance  = 5000; %500
        wManip = 15; %15
        epsSigma = 0.0001; % to render condition number nonsingular
        wb1 = 10; %1
        wb2 = 10;
        wb3 = 10;
        W_THETA_MAX = 10; %for tubular constraint
        s = 300; %300 % soft step coefficient for tubular constraint
        L4 = 0; %distance b/w the P4 and P_tiss
        varargout = {wAngle,wDistance,wManip,epsSigma,wb1,wb2,wb3,...
            W_THETA_MAX,s,L4};
    case 'tool parameters'
        EI_FIBERSCOPE = 1e-4; %measured by experiments
        EI_GRIPPER = 5.1e-4; %measured by experiments
        EI_LASER = 0; %Assumed
        EP_MICROSNAKE = 0;
        ES_MICROSNAKE = 42e9;
        OD_BB_MICROSNAKE = 0.254/1000; %microsnake secondary backbone OD
        ID_BB_MICROSNAKE = 0; %microsnake secondary backbone ID
        L_MICROSNAKE = 10.5/1000; %length
        L_S_MICROSNAKE = 1010/1000; %secondary backbone length from A/U to 1st segment EE of microsnake
        R_O_BB_MICROSNAKE = 0.65/1000;
        R_DISK_MICROSNAKE = 0.84/1000;
        DP_MICROSNAKE = 0;
        D_HEIGHT_ED_MICROSNAKE = 2.3/1000; %end disk height
        D_HEIGHT_SD_MICROSNAKE = 1.5/1000; %spacer disk height
        N_DISKS_MICROSNAKE = 4; %spacer + end disk
        D_O = 4.47/1000; %robot and endoscope channel center-to-center distance
        LENS_FOV_DEGREE = 55; %endoscope field of view (model 27020 AA from STORZ)
        LENS_ANGLE_DEGREE = 30; % endoscope angle (0 degree is forward-looking)(MEDIT borescope)
        %LENS_ANGLE_DEGREE = 0; %endoscope angle (0 degree is forward-looking)(model 27020 AA from STORZ)
        varargout = {EI_FIBERSCOPE,EI_GRIPPER,EI_LASER,EP_MICROSNAKE,...
            ES_MICROSNAKE,OD_BB_MICROSNAKE,ID_BB_MICROSNAKE,L_MICROSNAKE, L_S_MICROSNAKE,...
            R_O_BB_MICROSNAKE, R_DISK_MICROSNAKE,...
            DP_MICROSNAKE, D_HEIGHT_ED_MICROSNAKE, D_HEIGHT_SD_MICROSNAKE,...
            N_DISKS_MICROSNAKE,D_O,LENS_FOV_DEGREE,LENS_ANGLE_DEGREE};
    case 'joint limits'
        THETA_1L_MIN = 25 *(pi/180);
        THETA_1L_MAX = pi - THETA_1L_MIN;
        DELTA_1_MIN = -inf *(pi/180);
        DELTA_1_MAX = inf *(pi/180);
        THETA_2L_MIN = 55 *(pi/180);
        THETA_2L_MAX = pi - THETA_2L_MIN;
        DELTA_2_MIN = -inf *(pi/180);
        DELTA_2_MAX = inf *(pi/180);
        THETA_3L_MIN = 55 *(pi/180);
        THETA_3L_MAX = pi - THETA_3L_MIN;
        DELTA_3_MIN = -inf *(pi/180);
        DELTA_3_MAX = inf *(pi/180);
        QINS_MIN = -44/1000;
        QINS_MAX = 44/1000;
        varargout = {THETA_1L_MIN,...
            THETA_1L_MAX ,...
            DELTA_1_MIN,...
            DELTA_1_MAX,...
            THETA_2L_MIN,...
            THETA_2L_MAX,...
            DELTA_2_MIN,...
            DELTA_2_MAX,...
            THETA_3L_MIN,...
            THETA_3L_MAX,...
            DELTA_3_MIN,...
            DELTA_3_MAX,...
            QINS_MIN,...
            QINS_MAX};
    case 'actuation compensation'
        ETA1 = 1; %actuation compensation correction factor for segment1
        ETA2 = 1;
        ETA3 = 1;
        varargout = {ETA1,ETA2,ETA3};
    case 'potential field constants'
        D_INFL = 5/1000; %5/1000 %potetial influence distance
        R_STAR = 60/1000; %60/1000
        F_STAR = 5; %5 %repulsive force at d=0 & r=R_STAR
        P = 1;
        Q = 2;
        CONE_HEIGHT = 60/1000;
        varargout = {D_INFL,R_STAR,F_STAR,P,Q,CONE_HEIGHT};
    otherwise
        varargout = {};
end
end %end function