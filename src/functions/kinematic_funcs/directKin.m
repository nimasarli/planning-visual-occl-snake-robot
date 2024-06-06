% This function calculates direct kinematic homogenous transformation from
% frame i to frame j recursively where wcs is frame 0 and last frame is 4,
% the gripper frame of the 3rd segment
% 
% Author: Nima Sarli
% Date: 8/8/2016
%
function homTranij = directKin(Psi,frame_i,frame_j,L)

% parsing
theta1l = Psi(1);
delta1 = Psi(2);
theta2l = Psi(3);
delta2 = Psi(4);
theta3l = Psi(5);
delta3 = Psi(6);
qIns = Psi(7);
L1 = L(1);
L2 = L(2);
L3 = L(3);

%Exception Handling
if frame_j < frame_i
    error('j has to be greater than or equal to i');
end

if ~any(frame_i == [0,1,2,3,4])
    error('frame i should be 0,1,2,3 or 4');
end

if ~any(frame_j == [0,1,2,3,4])
    error('frame j should be 0,1,2,3 or 4');
end

%
if frame_i == frame_j
    homTranij = eye(4,4);
    return;  
elseif frame_i == frame_j-1
    if frame_i == 3
        P34 = Pbt_tl(theta3l , delta3 , L3);
        R34 = R_bi_gi(theta3l , delta3);
        T34 = [R34,P34;zeros(1,3),1];
        homTranij = T34;
        return;
    elseif frame_i == 2
        P23 = Pbt_tl(theta2l , delta2 , L2);
        R23 = R_bi_gi(theta2l , delta2);
        T23 = [R23,P23;zeros(1,3),1];
        homTranij = T23;
        return;
    elseif frame_i == 1
        P12 = Pbt_tl(theta1l , delta1 , L1);
        R12 = R_bi_gi(theta1l , delta1);
        T12 = [R12,P12;zeros(1,3),1];
        homTranij = T12;
        return;
    elseif frame_i == 0
        P01 = [0 0 qIns]';
        R01 = eye(3,3);
        T01 = [R01,P01;zeros(1,3),1];
        homTranij = T01;
        return;
    end   
else
    homTranij = directKin(Psi,frame_i,frame_j-1,L)*directKin(Psi,frame_j-1,frame_j,L);
end

end