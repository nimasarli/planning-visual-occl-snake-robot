% Generate points in the format of a matrix whose rows are
% point_s_i = [segment Number, s/L(segNum)] that
% are on the MBCR and inside the cone of FOV.
%
function [s_Zeros,r_Zeros,pointsForVoiMat] = genPointsForVoi(phi)
point_Interval = 0.002; %every 2mm
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

%
[s_Zeros,r_Zeros] = fZerosMbcrAndFov(phi);
pointsForVoiMat = [];
if all(isnan(s_Zeros))
    return
end

%
for segNum = 1:3 % three segments
    % check if the segment is in/out of cone
    homTran_0_s = bodyDirectKin(phi(1:7),0,[segNum,0],L);
    position_0_s = homTran_0_s(1:3,4);
    [~,d_point_s] = conicalParameters(position_0_s,phi(8));
    
    if isnan(s_Zeros(segNum))
        %         % check if the segment is in/out of cone
        %         homTran_0_s = bodyDirectKin(phi(1:7),0,[segNum,0],L);
        %         position_0_s = homTran_0_s(1:3,4);
        %         [~,d_point_s] = conicalParameters(position_0_s,phi(8));
        
        if d_point_s <= 0 %inside cone
            sOverL_Vec = [0:point_Interval:L(segNum)]'*(1/L(segNum));
            numPoints = length(sOverL_Vec);
            point_s_Vec = [segNum*ones(numPoints,1),sOverL_Vec];
            pointsForVoiMat = [pointsForVoiMat;point_s_Vec];
        end
        
    else
        if d_point_s <= 0 %first portion inside cone
            sOverL_Vec = [0:point_Interval:s_Zeros(segNum)]'*(1/L(segNum));
            numPoints = length(sOverL_Vec);
            point_s_Vec = [segNum*ones(numPoints,1),sOverL_Vec];
            pointsForVoiMat = [pointsForVoiMat;point_s_Vec];
        end
        
        % check if the second portion of segment is in/out of cone
        homTran_0_s = bodyDirectKin(phi(1:7),0,[segNum,(s_Zeros(segNum)+L(segNum))/(2*L(segNum))],L);
        position_0_s = homTran_0_s(1:3,4);
        [~,d_point_s] = conicalParameters(position_0_s,phi(8));
        
        if d_point_s <= 0 %inside cone
            sOverL_Vec = [s_Zeros(segNum):point_Interval:L(segNum)]'*(1/L(segNum));
            numPoints = length(sOverL_Vec);
            point_s_Vec = [segNum*ones(numPoints,1),sOverL_Vec];
            pointsForVoiMat = [pointsForVoiMat;point_s_Vec];
        end
    end
end

end

