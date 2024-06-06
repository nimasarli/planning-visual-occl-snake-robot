% This function calculates direct kinematic homogenous
% transformation from frame i (0 is wcs, 4 is the tip) to point_s where
% point_s = [segNum,sOverL]
%
function homTran_i_s = bodyDirectKin(Psi,frame_i,point_s,L)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');

% parsing
theta1l = Psi(1);
delta1 = Psi(2);
theta2l = Psi(3);
delta2 = Psi(4);
theta3l = Psi(5);
delta3 = Psi(6);
% qIns = Psi(7);
L1 = L(1);
L2 = L(2);
L3 = L(3);
segNum = point_s(1);
sOverL = point_s(2);

% Exception Handling
if ~any(frame_i == [0,1,2,3])
    error('The second argument can be 0,1,2 or 3')
end

if ~any(segNum == [1,2,3])
    error('Segment Number cab be 1,2 or 3')
end

% if (sOverL < 0) || (sOverL > 1)
%     error('This must hold: 0 <= sOverL <= 1');
% end

if segNum < frame_i
    error('This must hold: segNum >= frame_i');
end

%
if segNum == 3
    del_theta_t = THETA_0 - theta3l;
    thetaS = THETA_0 - sOverL*del_theta_t;
    s = sOverL*L3;
    P_from_base_frame_to_s = Pbt_tl(thetaS , delta3 , s);
    R_from_base_frame_to_s = R_bi_gi(thetaS , delta3);
elseif segNum == 2
    del_theta_t = THETA_0 - theta2l;
    thetaS = THETA_0 - sOverL*del_theta_t;
    s = sOverL*L2;
    P_from_base_frame_to_s = Pbt_tl(thetaS , delta2 , s);
    R_from_base_frame_to_s = R_bi_gi(thetaS , delta2);
elseif segNum == 1
    del_theta_t = THETA_0 - theta1l;
    thetaS = THETA_0 - sOverL*del_theta_t;
    s = sOverL*L1;
    P_from_base_frame_to_s = Pbt_tl(thetaS , delta1 , s);
    R_from_base_frame_to_s = R_bi_gi(thetaS , delta1);
end
T_from_base_frame_to_s = [R_from_base_frame_to_s,P_from_base_frame_to_s;...
    zeros(1,3),1];
homTran_i_s = directKin(Psi,frame_i,segNum,L)*T_from_base_frame_to_s;

end


