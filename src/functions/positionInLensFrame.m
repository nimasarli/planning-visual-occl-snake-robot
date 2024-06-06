% position_In_World_Frame: 
%                                 postion of point in world frame
% lens_angle_degree (in degrees):
%                                 e.g. 0 degree for front-viewing endoscope
% lens_rotation_angle_degree (in degree): 
%                                 the angle of rotation of endoscope in resectoscope
% center_To_Center_Dis:           
%                                 robot and endoscope channel center-to-center distance
%
function position_In_Lens_Frame = positionInLensFrame(position_In_World_Frame,center_To_Center_Dis,lens_angle_degree,lens_rotation_angle_degree)

% rotation sequence from endoscope lens frame to world frame ^{lens}^R_{w}
rot_W_Lens = rotZ(lens_rotation_angle_degree*(pi/180)) * rotY(-lens_angle_degree*(pi/180));
rot_Lens_W = rot_W_Lens';

%
position_In_Lens_Frame = rot_Lens_W*(position_In_World_Frame+[0;-center_To_Center_Dis;0]);

