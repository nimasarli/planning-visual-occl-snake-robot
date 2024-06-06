% seg1_Psp_Index_Vec is a vec whose elements are integers 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 1
% seg2_Psp_Index_Vec is a vec whose elements are integers 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 2
% seg3_Psp_Index_Vec is a vec whose elements are integers 0-100 with 0 denoting the beginning
% point and 100 denoting the end point of segment 3
% historyStruct is an array structure whose elemnts are each snake point's
% address, hom xfrom, translational Jacobian and applied potential force
%
function [grad_Pi_In_Psi,historyStruct] = gradPiInPsi(psi,...
    lens_Rotation_Angle,...
    seg1_Psp_Index_Vec,...
    seg2_Psp_Index_Vec,...
    seg3_Psp_Index_Vec)

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

% seg1
i = 0;
for seg1_Psp_Index = seg1_Psp_Index_Vec
    i = i + 1;
    segNum = 1;
    sOverL = seg1_Psp_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% seg2
for seg2_Psp_Index = seg2_Psp_Index_Vec
    i = i + 1;
    segNum = 2;
    sOverL = seg2_Psp_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% seg3
for seg3_Psp_Index = seg3_Psp_Index_Vec
    i = i + 1;
    segNum = 3;
    sOverL = seg3_Psp_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% All repulsive PSP's on all segments
lens_Rotation_Angle_Degree = lens_Rotation_Angle*(180/pi);
for j = 1:i %i is now the number of PSP's
    J_X_Psi_S_j = bodyJXPsi(psi,historyStruct(j).point_s);
    historyStruct(j).J_V_Psi_S = J_X_Psi_S_j(1:3,:); %translational Jacobian
    homTran_0_s = bodyDirectKin(psi,0,historyStruct(j).point_s,L); %pose of point s in world frame
    historyStruct(j).homTran_0_s = homTran_0_s;
    point_s_Position = homTran_0_s(1:3,4);
    point_s_Position_In_Lens_Frame = positionInLensFrame(point_s_Position,D_O,LENS_ANGLE_DEGREE,lens_Rotation_Angle_Degree);
    [body_Potential_Force,dForce_dGamma] = bodyPotentialForce(point_s_Position_In_Lens_Frame,lens_Rotation_Angle);
    historyStruct(j).potential_Force = body_Potential_Force;
    historyStruct(j).dForce_dGamma = dForce_dGamma;
end

% gradient of bending energy
gradU1 = getGradEnergy(psi,1);
gradU2 = getGradEnergy(psi,2);
gradU3 = getGradEnergy(psi,3);
gradU = [gradU1;gradU2;gradU3;0];

% potetnial force in Psi space
total_Potential_Force_In_Psi_Space = zeros(7,1);
for j = 1:i
    J_V_Psi_S_j = historyStruct(j).J_V_Psi_S; %translational Jacobian
    potential_Force_j = historyStruct(j).potential_Force;
    f_In_Psi_Space_j = J_V_Psi_S_j'*potential_Force_j;
    total_Potential_Force_In_Psi_Space = total_Potential_Force_In_Psi_Space + f_In_Psi_Space_j;
end

% Output
grad_Pi_In_Psi = gradU - total_Potential_Force_In_Psi_Space; %body psp
% grad_Pi_In_Psi = -total_Potential_Force_In_Psi_Space; %w/o energy gradient

end

