% Description: Redundancy resolution for a 3segment + translation TURBT robot
% assuming a 5-DOF task. redundancy resolution is implemented to minimize endoscopic image
% occlusion
% There are two modes of operation:
%
% Mode I:
%          The endoscope is stationary;
% Mode II:
%          Endoscope rotation DOF is utilized to minimize end effector-FOV center-to-center distance.
%
% This file implements mode II for task of tracking a circle.
%
% Created By Nima Sarli
%
% Created :         09/02/2016
%
% Last Modified : 2/20/2017
%
% To-do
% 1- tune potential force with min norm solution (eps = 0) --ALMOST DONE
% 2- make sure about the direction of tau_psi bring back null space
% projector (eps = 0) --DONE
% 2- add saturation to force
% 3- in the end, bring in joint limit (for experiments)
% 4- add more psp's --DONE
% 5- add cam view
% 7- change back BETA (half-angle of cone) --DONE
% 9- use simple pseudoinverse for null-space term versus weighted term
% 10- do position tracking only to have more redundancy if you want to get
% more convincing results
% 11- epsilon term is ruining stuff, what you will do with that?
%
% Conclusions
% 1- both exp(Pi) and Pi minimization lead to almost the same Pi with the
% given coefficients, but Pi minimization pushed the snake more outside.
% This tells me the problem is in ?
% 2- More psp points probably means more chances of mbcr going crazy
% because it will impend more motion at the config level. 

%%
close all;clear all;clc;
addpath(genpath('draw'));
addpath(genpath('functions'));

%% parameters that are changed more often
TIME_STEP = 0.001;
T_F = 4.67; %2.5%4.67 gives full rotation for gamma
EPSILON = 1e-6;
eta_2 = 30; %for endoscope rotation gradient descent alg

% redundancyMethodStr = 'Min-Potential-and-LS';
% c_Pi = 1e3; %1e3 if c_LS is 0 and delta_t = 0.001
% c_LS = 0; %1e5 if c_Pi is 0 and delta_t = 0.001
% annotation_Text_Str = ['Track a circle, ',redundancyMethodStr,' Solution, Mode II, Step = ',num2str(TIME_STEP)];
% video_Name_Str = 'mode_II_min_pot_and_ls_track_circle';
% draw_potetial_force_Str = 'y';

redundancyMethodStr = 'Min-Potential';
eta = 60;
annotation_Text_Str = ['Track a circle, ',redundancyMethodStr,' Solution, Mode II, Step = ',num2str(TIME_STEP)];
video_Name_Str = 'mode_II_min_pot_track_circle';
draw_potetial_force_Str = 'y';

% redundancyMethodStr = 'Min-Norm';
% annotation_Text_Str = ['Track a circle, ',redundancyMethodStr,' Solution, Mode II, Step = ',num2str(TIME_STEP)];
% video_Name_Str = 'mode_II_min_norm_track_circle';
% draw_potetial_force_Str = 'n';

%%
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
psi_Init = [1.5394
    -3.8988
    2.9112
    -6.0515
    0.1959
    0.2020
    -0.0151];

gamma_Init = 0;
psi_Prev = psi_Init; %initialize
gamma_Prev = gamma_Init;
psi_Mat = psi_Init; %solution matrix initialization
gamma_Vec = gamma_Init;
psidt_Prev = zeros(7,1); %initialize
gammadt_Prev = 0;

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

seg1_LS_Point_Index_Vec = [0,50,100]; %a vec whose elements can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 1
seg2_LS_Point_Index_Vec = [50,100]; %a vec whose elements can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 2
seg3_LS_Point_Index_Vec = 50; %a vec whose elements can be integer 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 3

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
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Update psi nad gamma
    psi_Cur = psidt_Prev*TIME_STEP + psi_Prev,
    gamma_Cur = gammadt_Prev*TIME_STEP + gamma_Prev,
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Calculate psidt_Des for next iteration
    % current pose
    homTran04 = directKin(psi_Cur,0,4,L); %direct kinematic homogeneous
    % transformation form frame 0 to 4 (base to end-effector frame)
    P_L_cur = homTran04(1:3,4);
    R_cur = homTran04(1:3,1:3);
    
    % desired pose
    modeString = 'Circle - Angled Cone';
    [P_L_des,R_des] = desPathGenerate(time,modeString);
    
    % desired end-effector velocity (for resolved rate algorithm)
    x_dt_Des = desiredTwist5Dof(P_L_cur,R_cur,P_L_des,R_des);
    
    % redundancy resolution
    J_X_Psi = Jxsai(psi_Cur);
    % rank_Mat(:,counter+1) = rank(J_X_Psi);
    %W_joint = jointLimitWeightSimaan(psi_Prev,psidt_Prev,PSI_MAX,PSI_MIN);
    W_joint = diag([1,1,1,1,1,1,1]);
    %EPSILON = 0.0001;
    W_task = diag(ones(1,6));
    %J_X_Psi_pinv =(J_X_Psi'*W_task*J_X_Psi+EPSILON*W_joint)\J_X_Psi'*W_task; %SRI
    J_X_Psi_pinv = (W_joint\J_X_Psi')/(J_X_Psi*(W_joint\J_X_Psi')+EPSILON*eye(6)); %regularized WLN
    [grad_Pi,histStructFromGradPiInPsi] = gradPiInPsi(psi_Cur,...
        gamma_Cur,...
        seg1_Psp_Index_Vec,...
        seg2_Psp_Index_Vec,...
        seg3_Psp_Index_Vec);
    [weighted_lineOfSight_Distance,lineOfSight_Distance,~] = ...
        rmsLineSightDistance(psi_Cur,...
        seg1_LS_Point_Index_Vec,...
        seg2_LS_Point_Index_Vec,...
        seg3_LS_Point_Index_Vec), % weighted root mean square distance to line of sight
    
    %
    if strcmp(redundancyMethodStr,'Min-Norm')
        Psidt_Des = J_X_Psi_pinv*x_dt_Des;
        gammadt_Des = -eta_2*d_Dc_d_Gamma_Func(psi_Cur,gamma_Cur);
    elseif strcmp(redundancyMethodStr,'Min-Potential')
        Psidt_Des = J_X_Psi_pinv*x_dt_Des - eta*(eye(7)-J_X_Psi_pinv*J_X_Psi)*grad_Pi;
        gammadt_Des = -eta_2*d_Dc_d_Gamma_Func(psi_Cur,gamma_Cur);
    elseif strcmp(redundancyMethodStr,'Min-Potential-and-LS')
        grad_LineSightDistance = numericalGradient(@rmsLineSightDistance,psi_Cur,...
            seg1_LS_Point_Index_Vec,...
            seg2_LS_Point_Index_Vec,...
            seg3_LS_Point_Index_Vec);
        Psidt_Des = J_X_Psi_pinv*x_dt_Des - (eye(7)-J_X_Psi_pinv*J_X_Psi)*...
            (c_Pi*exp(Pi_Prev)*grad_Pi-c_LS*exp(-weighted_lineOfSight_Distance)*grad_LineSightDistance);
        gammadt_Des = -eta_2*d_Dc_d_Gamma_Func(psi_Cur,gamma_Cur);
    else
        error('Wrong redundancyMethodStr!');
    end
    
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
    gamma_Vec(:,counter) = gamma_Cur;
    psidt_Des_Mat (:,counter) = Psidt_Des;
    gammadt_Des_Vec (:,counter) = gammadt_Des;
    lineOfSight_Distance_Vec(counter,1) = lineOfSight_Distance;
    historyStruct(counter).psi = psi_Cur;
    historyStruct(counter).gamma = gamma_Cur; %rotation angle of endoscope
    historyStruct(counter).time = time;
    historyStruct(counter).Data = histStructFromGradPiInPsi;
    if counter ~= 1
        historyStruct(counter-1).Pi = Pi_Prev;
        Pi_Mat(counter-1) = Pi_Prev;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%--------- Set up for next iteration
    psi_Prev = psi_Cur;
    gamma_Prev = gamma_Cur;
    psidt_Prev = Psidt_Des;
    gammadt_Prev = gammadt_Des;
    potential_Work_PPrev = potential_Work_Prev;
    Pi_PPrev = Pi_Prev;
end

psi_Mat_degree_and_mm = [psi_Mat(1:6,:)*(180/pi);psi_Mat(7,:)*1000];

%% draw plots
% main_drawplots;

% plot Pi
figure();
h10 = plot(Pi_Mat);
set(h10,'LineWidth',2);
set(gca,'FontSize',20);
grid on;
xlabel('x');
ylabel('y');
saveas(gcf,'figures/mode2_track_circle_pi_plot.epsc'); %for psfrag

xlabel('Solution Index','interpreter','LaTex','FontSize',20);
ylabel('$\Pi$','interpreter','LaTex','FontSize',20);
saveas(gcf,'figures/mode2_track_circle_pi_plot.fig');
saveas(gcf,'figures/mode2_track_circle_pi_plot.png');

% plot gamma
figure();
h11 = plot(gamma_Vec*(180/pi));
set(h11,'LineWidth',2);
set(gca,'FontSize',20);
grid on;
xlabel('x');
ylabel('y');
saveas(gcf,'figures/mode2_track_circle_gamma_plot.epsc'); %for psfrag

xlabel('Solution Index','interpreter','LaTex','FontSize',20);
ylabel('$\gamma(degree)$','interpreter','LaTex','FontSize',20);
saveas(gcf,'figures/mode2_track_circle_gamma_plot.fig');
saveas(gcf,'figures/mode2_track_circle_gamma_plot.png');

% plot lineOfSight_Distance
figure();
h12 = plot(1000*lineOfSight_Distance_Vec);
set(h12,'LineWidth',2);
set(gca,'FontSize',20);
grid on;
axis tight;
xlabel('x');
ylabel('y');
saveas(gcf,'figures/mode2_track_circle_dls_plot','eps'); %for psfrag

xlabel('Solution Index','interpreter','LaTex','FontSize',20);
ylabel('$\overline{d}_{LS}(mm)$','interpreter','LaTex','FontSize',20);
saveas(gcf,'figures/mode2_track_circle_dls_plot','fig');
saveas(gcf,'figures/mode2_track_circle_dls_plot','png');

%% draw and capture

% draw a circle
path = [];
for j = linspace(0,T_F,100)
    [path_coordinate , ~] = desPathGenerate(j,modeString);
    path = [path , path_coordinate];
end
figure;
targetPointPlotHandle = plot3(1000*path(1,:),1000*path(2,:),1000*path(3,:),...
    'Marker','.','MarkerSize',5,'LineStyle','none','Color','k');
set(targetPointPlotHandle,'HandleVisibility','off'); %so that the circle plot
% wouldn't be cleared later when cla is issued
axis auto;
hold on;

set(gcf,'Renderer','opengl')
p = drawAndCapture(historyStruct,...
    'capture','y',...
    'name',video_Name_Str,...
    'draw_potetial_cone','y',...
    'draw_potetial_force',draw_potetial_force_Str,...
    'camera_view','external',...
    'annotation_text',annotation_Text_Str,...
    'view',[-53 8]);
