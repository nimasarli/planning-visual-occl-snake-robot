% Translational Jacobian of control points c1,c2 each located at the
% origins of {2},{3} respectively. see notes/J_c.pdf
% J_c_psi is 3*7
%
function J_c_psi = Jc(Psi,control_point_index)
% TURBT robot constants
[L1,L2,L3,Ls1_1,Ls1_2,Ls1_3,R_O_BB,R_O_CH,R_DISK,DP,...
    OD_S1,ID_S1,OD_S2,ID_S2,...
    OD_S3,ID_S3,D_HEIGHT_ED,D_HEIGHT_SD,THETA_0,BETA,...
    N_DISKS_SEG1,N_DISKS_SEG2,N_DISKS_SEG3,EP,ES,MAX_STRAIN] = ...
    setParam('TURBT constants');
L = [L1;L2;L3];

% parsing psi
theta1l = Psi(1);
delta1 = Psi(2);
theta2l = Psi(3);
delta2 = Psi(4);
% theta3l = Psi(5);
% delta3 = Psi(6);
% qIns = Psi(7);

% Jtvpsi
J1_v_sai = Jtvsai(theta1l , delta1 , L1);
J2_v_sai = Jtvsai(theta2l , delta2 , L2);

% Jtomegasai
J1_omega_sai = Jtomegasai(theta1l , delta1);
J2_omega_sai = Jtomegasai(theta2l , delta2);

%
J1 = [J1_v_sai ; J1_omega_sai];
J2 = [J2_v_sai ; J2_omega_sai];
J4 = [0 0 1 0 0 0].';

%
homTran01 = directKin(Psi,0,1,L);
R01 = homTran01(1:3,1:3);
homTran12 = directKin(Psi,1,2,L);
R12 = homTran12(1:3,1:3);
homTran02 = directKin(Psi,0,2,L);
R02 = homTran02(1:3,1:3);

%
homTran23 = directKin(Psi,2,3,L); %consider using Pbt_tl function for faster performance
P_2_2L = homTran23(1:3,4);

% selection matrices
vec1 = R12*P_2_2L;
cross_prod_mat_vec1 = crossProd_mat(vec1);
Sc1_4 = [R01, zeros(3,3)];
Sc2_4 = [R01, zeros(3,3)];
Sc1_1 = [R01, zeros(3,3)];
Sc2_1 = [R01, -R01*cross_prod_mat_vec1];
Sc2_2 = [R02, zeros(3,3)];

%
switch control_point_index
    case 1 %c1
        J_c_psi = [Sc1_1*J1,zeros(3,2),zeros(3,2),Sc1_4*J4]; %J_c1
    case 2 %c2
        J_c_psi = [Sc2_1*J1,Sc2_2*J2,zeros(3,2),Sc2_4*J4]; %J_c2
    otherwise
        J_c_psi = [];
end

end

