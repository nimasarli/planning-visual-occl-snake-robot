% Description: Redundancy resolution for a 3-segment + translation TURBT robot
% assuming a 5-DOF task. redundancy resolution is implemented to minimize endoscopic image
% occlusion.
% Method: Weighted LS 
%
% Task: Find Psi corresonding with a Pdes and Rdes
%
% Created : 
%                   02/14/2017
% Modified :   
%
% Created By:
%                   Nima Sarli
%
%%
%
close all;clear all;clc;
addpath(genpath('draw'));
addpath(genpath('functions'));

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
LENS_ANGLE = LENS_ANGLE_DEGREE*(pi/180);

% joint limits
[THETA_1L_MIN,...
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
    QINS_MAX] = setParam('joint limits');
PSI_MAX = [THETA_1L_MAX;DELTA_1_MAX;...
    THETA_2L_MAX;DELTA_2_MAX;...
    THETA_3L_MAX;DELTA_3_MAX];
PSI_MIN = [THETA_1L_MIN;DELTA_1_MIN;...
    THETA_2L_MIN;DELTA_2_MIN;...
    THETA_3L_MIN;DELTA_3_MIN];

% potetntial method constants
[D_INFL,R_STAR,F_STAR,P,Q,CONE_HEIGHT] = ...
    setParam('potential field constants');
CONE_BASE_RADIUS = CONE_HEIGHT*tan(0.5*LENS_FOV_DEGREE*(pi/180));

%% set up redundancy resolution ODE
% ODE parameters
%psi_Init = [pi/2.1,0,pi/2.1,0,pi/2.1,0,0]';
%psi_Init = [0,0,pi/2.1,0,pi/2.1,0,0]';
%psi_Init = [1.7639,1.2896,1.3662,1.0268,1.6069,-0.2925,0.0153-35/1000]'; %corresponding to
% position of [0,D_O,45/1000] and z_4_hat facing up
% psi_Init = [0.4467,-1.5500,0.6985,-4.6482,1.3140,1.4109,-0.0096]';%corresponding to position of P_L_des
% = [0;D_O+tan(0.5*55*(pi/180))*(45/1000);45/1000] and z_4_hat facing up
% psi_Init = [0.4927,-1.9808,0.7591,-5.0562,1.2903,0.9530,-0.0102]';%corresponding to position of P_L_des = [-tan(0.5*55*(pi/180))*(45/1000)*sin(theta);...
% D_O+tan(0.5*55*(pi/180))*(45/1000)*cos(theta)45/1000] and z_4_hat facing
psi_Init = [0.4927,-1.9808,0.7591,-5.0562,1.2903,0.9530,-0.0102]';%corresponding to position of P_L_des = [-tan(0.5*55*(pi/180))*(45/1000)*sin(theta);...
% D_O+tan(0.5*55*(pi/180))*(45/1000)*cos(theta)45/1000] and z_4_hat facing
% up (theta = 30 degrees)
TIME_STEP = 0.01;
T_F = 3; %4;
psi_Prev = psi_Init; %initialize
psi_Mat = psi_Init; %solution matrix initialization
psidt_Prev = zeros(7,1); %initialize
counter = 0;

seg1_Psp_Index_Vec = [50,75,100]; %a vec whose elements are can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 1
seg2_Psp_Index_Vec = [25,50,75,100]; %a vec whose elements are can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 2
seg3_Psp_Index_Vec = [25,50,75]; %a vec whose elements are can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 3
pspNum = size(seg1_Psp_Index_Vec,2)+...
    size(seg2_Psp_Index_Vec,2)+...
    size(seg3_Psp_Index_Vec,2);

%% Solve using Euler Method
while(true)
    
    %%%--------- break condition
    time = TIME_STEP*counter;
    if time > T_F
        disp('Done!');
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        break;
    end
    
    %
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    counter = counter + 1,
    time,
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Compute Pi
    energy_Prev_Vec = getEnergy(psi_Prev); % Energy
    energy_Prev = sum(energy_Prev_Vec);
    if (counter == 1) || (counter == 2) % need two iterations to get estimate of dPi/dGamma at prev time step (I have this
        % though I don't have because I used mode code as a starting point)
        potential_Work_Prev = 0;
        Pi_Prev = energy_Prev - potential_Work_Prev, % Pi = E - W
    else
        potential_Work_PPrev_To_Prev = 0; %initialize
        for m = 1:pspNum
            % pprev
            pspDataStrcut_PPrev = historyStruct(counter-2).Data;
            psp_HomTran_0_s_PPrev = pspDataStrcut_PPrev(m).homTran_0_s;
            psp_Position_PPrev = psp_HomTran_0_s_PPrev(1:3,4);
            potential_Force_at_Psp_PPrev = pspDataStrcut_PPrev(m).potential_Force;
            
            % prev
            pspDataStrcut_Prev = historyStruct(counter-1).Data;
            psp_HomTran_0_s_Prev = pspDataStrcut_Prev(m).homTran_0_s;
            psp_Position_Prev = psp_HomTran_0_s_Prev(1:3,4);
            
            %
            potential_Work_PPrev_To_Prev = potential_Work_PPrev_To_Prev + ...
                (psp_Position_Prev-psp_Position_PPrev)'*potential_Force_at_Psp_PPrev;
        end
        potential_Work_Prev = potential_Work_PPrev + potential_Work_PPrev_To_Prev;
        Pi_Prev = energy_Prev - potential_Work_Prev, % Pi = E - W
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Update psi
    psi_Cur = psidt_Prev*TIME_STEP + psi_Prev,
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Calculate psidt_Des for next iteration
    % current pose
    homTran04 = directKin(psi_Cur,0,4,L); %direct kinematic homogeneous
    % transformation form frame 0 to 4 (base to end-effector frame)
    P_L_cur = homTran04(1:3,4);
    R_cur = homTran04(1:3,1:3);
    
    % desired pose
    R_des = [1,0,0;...
        0,1,0;...
        0,0,1]; %keep the snake facing toward z0
    cone_Vertex = [0;D_O;0];
    rot_W_Lens = rotationWorldLens(LENS_ANGLE,0);
    P_L_des = cone_Vertex + 0.75*CONE_HEIGHT*rot_W_Lens(:,3);
    
    % desired end-effector velocity (for resolved rate algorithm)
    x_dt_Des = desiredTwist5Dof(P_L_cur,R_cur,P_L_des,R_des);
    
    % redundancy resolution
    J_X_Psi = Jxsai(psi_Cur);
    % rank_Mat(:,counter+1) = rank(J_X_Psi);
    %W_joint = jointLimitWeightSimaan(psi_Prev,psidt_Prev,PSI_MAX,PSI_MIN);
    W_joint = diag([1,1,1,1,1,1,1]);
    %EPSILON = 0.0001;
    EPSILON = 0;
    W_task = diag(ones(1,6));
    %J_X_Psi_pinv =(J_X_Psi'*W_task*J_X_Psi+EPSILON*W_joint)\J_X_Psi'*W_task; %SRI
    J_X_Psi_pinv = (W_joint\J_X_Psi')/(J_X_Psi*(W_joint\J_X_Psi')+EPSILON*eye(6)); %regularized WLN
    [grad_Pi,histStructFromGradPiInPsi] = gradPiInPsi(psi_Cur,...
        0,...
        seg1_Psp_Index_Vec,...
        seg2_Psp_Index_Vec,...
        seg3_Psp_Index_Vec);
    
    %Psidt_Des = J_X_Psi_pinv*x_dt_Des;
    eta = 15;
    Psidt_Des = J_X_Psi_pinv*x_dt_Des - eta*(eye(7)-J_X_Psi_pinv*J_X_Psi)*grad_Pi;
    %Psidt_Des = J_X_Psi_pinv*x_dt_Des;
    
    % sanity checks
    % I_MUST_BE_ZERO = J_X_Psi*(eye(7)-J_X_Psi_pinv*J_X_Psi)*psidt_zero, %must be zero
    
    % calculate, console out and save min singular value of J_X_Psi
    J_X_Psi_Translation = J_X_Psi(1:3,:);
    J_X_Psi_Rotation = J_X_Psi(4:6,:);
    sigma_J_X_Psi_Translation = svd(J_X_Psi_Translation);
    %fprintf('sigma_J_X_Psi_Translation = %0.6f,%0.6f,%0.6f\n',sigma_J_X_Psi_Translation);
    sigma_J_X_Psi_Rotation = svd(J_X_Psi_Rotation);
    sigma_min_J_X_Psi_Translation = sigma_J_X_Psi_Translation(3);
    %fprintf('sigma_min_J_X_Psi_Translation = %0.6f\n',sigma_min_J_X_Psi_Translation);
    sigma_min_J_X_Psi_Rotation = sigma_J_X_Psi_Rotation(3);
    sigma_min_Translation_vec(counter) = sigma_min_J_X_Psi_Translation;
    sigma_min_Rotation_vec(counter) = sigma_min_J_X_Psi_Rotation;
    
    % Save Solutions
    psi_Mat(:,counter) = psi_Cur;
    psidt_Des_Mat (:,counter) = Psidt_Des;
    historyStruct(counter).psi = psi_Cur;
    historyStruct(counter).gamma = 0; %rotation angle of endoscope
    historyStruct(counter).time = time;
    historyStruct(counter).Data = histStructFromGradPiInPsi;
    if counter ~= 1
        historyStruct(counter-1).Pi = Pi_Prev;
        Pi_Mat(counter-1) = Pi_Prev;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Set up for next iteration
    psi_Prev = psi_Cur;
    psidt_Prev = Psidt_Des;
    potential_Work_PPrev = potential_Work_Prev;
    Pi_PPrev = Pi_Prev;
    
end

psi_Mat_degree_and_mm = [psi_Mat(1:6,:)*(180/pi);psi_Mat(7,:)*1000];

