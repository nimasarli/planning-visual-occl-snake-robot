% Computes the wighted and non-weighted rms distance bw points on MBCR body and the line of
% sight. Weightd version is used for optimization, non-weihgted is used for
% plottig ans assessment.
%
function [weighted_Rms_LineOfSight_Distance,rms_LineOfSight_Distance,historyStruct] = ...
    rmsLineSightDistance(psi,...
    seg1_LS_Point_Index_Vec,...
    seg2_LS_Point_Index_Vec,...
    seg3_LS_Point_Index_Vec)

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
for seg1_Index = seg1_LS_Point_Index_Vec
    i = i + 1;
    segNum = 1;
    sOverL = seg1_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% seg2
for seg2_Index = seg2_LS_Point_Index_Vec
    i = i + 1;
    segNum = 2;
    sOverL = seg2_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% seg3
for seg3_Index = seg3_LS_Point_Index_Vec
    i = i + 1;
    segNum = 3;
    sOverL = seg3_Index/100;
    point_s = [segNum,sOverL];
    historyStruct(i).point_s = point_s;
end

% line of sight
[p0_ls,u_ls] = lineOfSight(psi);

% compute weighted root mean square
sum_Square = 0;
weighted_Sum_Square = 0;
seg1_Ls_Num = length(seg1_LS_Point_Index_Vec);
seg2_Ls_Num = length(seg2_LS_Point_Index_Vec);
seg3_Ls_Num = length(seg3_LS_Point_Index_Vec);

for j = 1:i %i is now the number of line of sight points
    homTran_0_s = bodyDirectKin(psi,0,historyStruct(j).point_s,L); %pose of point s in world frame
    historyStruct(j).homTran_0_s = homTran_0_s;
    point_s_Position = homTran_0_s(1:3,4);
    point_LS_Distance = pointLineDistance(u_ls,p0_ls,point_s_Position);
    historyStruct(j).point_LS_Distance = point_LS_Distance;
    
    sum_Square = sum_Square + point_LS_Distance^2;
    if j <= seg1_Ls_Num
        weighted_Sum_Square = weighted_Sum_Square + 10*point_LS_Distance^2;
    elseif (j <= seg1_Ls_Num+seg2_Ls_Num) && (j > seg1_Ls_Num)
        weighted_Sum_Square = weighted_Sum_Square + 5*point_LS_Distance^2;
    elseif (j > seg1_Ls_Num+seg2_Ls_Num)
        weighted_Sum_Square = weighted_Sum_Square + 1*point_LS_Distance^2;
    end
end

% root mean square
rms_LineOfSight_Distance = sqrt(sum_Square/i);

% weighted root mean square
sum_Weights = 10*seg1_Ls_Num+5*seg2_Ls_Num+1*seg3_Ls_Num;
weighted_Rms_LineOfSight_Distance = sqrt(weighted_Sum_Square/sum_Weights);

end
