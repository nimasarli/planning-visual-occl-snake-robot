% Copmutes VOI index (See IROS2017) given phi = [psi;gamma]
%
function voi = computeVoi(phi)
%------------- constants ---------------------------
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
BETA_ANGLE = LENS_FOV_DEGREE/2*(pi/180);

%-- calculate mbcr-cone zeros and descretize parts of mbcr inside cone ---
[s_Zeros,r_Zeros,pointsForVoiMat] = genPointsForVoi(phi);
% s_Zeros,r_Zeros,pointsForVoiMat
if all(isnan(s_Zeros))
    voi = 0;
    fprintf('No part of MBCR is inside field of view!\n');
    return
end
%------------- Finding nu -----------------------------
% first point of entry into the cone
homTran_0_s_Zero = bodyDirectKin(phi(1:7),0,pointsForVoiMat(1,:),L);
p_s_zero_In_World = homTran_0_s_Zero(1:3,4);
p_s_zero_In_Lens = positionInLensFrame(p_s_zero_In_World,D_O,LENS_ANGLE_DEGREE,phi(8)*(180/pi));
[r_entry,~] = conicalParameters(p_s_zero_In_World,phi(8)); %r_entry

% other points inside the cone
alpha_Degree_Max = 0;
for i = 1:size(pointsForVoiMat,1) %for all points inside cone
    homTran_0_s = bodyDirectKin(phi(1:7),0,pointsForVoiMat(i,:),L);
    p_s_In_World = homTran_0_s(1:3,4);
    p_s_In_Lens = positionInLensFrame(p_s_In_World,D_O,LENS_ANGLE_DEGREE,phi(8)*(180/pi));
    alpha_Degree = real(acosd(p_s_zero_In_Lens'*p_s_In_Lens/...
        (norm(p_s_zero_In_Lens)*norm(p_s_In_Lens))));
    if alpha_Degree_Max < alpha_Degree
        alpha_Degree_Max = alpha_Degree;
    end
    
end

% alpha_Degree_Max

if alpha_Degree_Max*(pi/180) <= 2*BETA_ANGLE
    nu = alpha_Degree_Max*(pi/180);
else
    nu = 2*BETA_ANGLE;
end

%-------------Finding r_max (or r_ee) ---------------------
homTran04 = directKin(phi(1:7),0,4,L);
p_ee = homTran04(1:3,4);
[r_ee,~] = conicalParameters(p_ee,phi(8));
% r_ee =  45/1000/cos(BETA_ANGLE);


%------------ VOI -----------------------------------------
voi = (tan(nu)^2*(1-r_entry^3/r_ee^3)*(cot(BETA_ANGLE)^2 + 1)^2)/(4*(cot(BETA_ANGLE) + tan(nu))^2);

end
