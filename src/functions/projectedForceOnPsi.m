function projected_Force_On_Psi = projectedForceOnPsi(Psi,...
    lens_rotation_angle_degree,...
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
    pointsStruct(i).point_s = point_s;
end

% seg2
for seg2_Psp_Index = seg2_Psp_Index_Vec
    i = i + 1;
    segNum = 2;
    sOverL = seg2_Psp_Index/100;
    point_s = [segNum,sOverL];
    pointsStruct(i).point_s = point_s;
end

% seg3
for seg3_Psp_Index = seg3_Psp_Index_Vec
    i = i + 1;
    segNum = 3;
    sOverL = seg3_Psp_Index/100;
    point_s = [segNum,sOverL];
    pointsStruct(i).point_s = point_s;
end

% All points on all segments
projected_Force_On_Psi = zeros(7,1);
for j = 1:i %i is now the number of PSP's
    J_X_Psi_S_j = bodyJXPsi(Psi,pointsStruct(j).point_s);
    J_V_Psi_S_j = J_X_Psi_S_j(1:3,:); %translational Jacobian
    homTran_0_s = bodyDirectKin(Psi,0,pointsStruct(j).point_s,L); %pose of point s in world frame
    point_s_Position = homTran_0_s(1:3,4);
    point_s_Position_In_Lens_Frame = homTranWorldInLens(point_s_Position,D_O,LENS_ANGLE_DEGREE,lens_rotation_angle_degree);
    potential_Force_j = potentialForce(point_s_Position_In_Lens_Frame);
    f_In_Psi_Space_j = J_V_Psi_S_j'*potential_Force_j;
    projected_Force_On_Psi = projected_Force_On_Psi + f_In_Psi_Space_j;
end

end

