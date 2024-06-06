%
% Caveat: finds only one solution per segment!
%
function [s_Zeros,r_Zeros] = fZerosMbcrAndFov(phi)

% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

s_Zeros = zeros(3,1); %preallocation
r_Zeros = zeros(3,1); %preallocation
for segNum = 1:3
    init_Guess = L(segNum)/2;
    s_Zero = fzero(@(s)mbcrAndFovEqns(s,segNum,phi),init_Guess);
    [~,r_Zero] = mbcrAndFovEqns(s_Zero,segNum,phi);
    if (s_Zero >= 0) && (s_Zero <= L(segNum)) && (r_Zero >= 0) %Valid solution
        s_Zeros(segNum) = s_Zero;
    else
        s_Zeros(segNum) = NaN;
    end
    
    r_Zeros(segNum) = r_Zero;
end

end

