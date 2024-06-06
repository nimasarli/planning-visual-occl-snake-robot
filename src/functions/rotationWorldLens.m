% rotation sequence from endoscope lens frame to world frame ^{lens}^R_{w}
%
% lens_angle (in radians):
%                                 e.g. 0 degree for front-viewing endoscope
% lens_rotation_angle (in radians): 
%                                 the angle of rotation of endoscope in resectoscope
%
function rot_W_Lens = rotationWorldLens(lens_angle,lens_rotation_angle)

rot_W_Lens = rotZ(lens_rotation_angle) * rotY(-lens_angle);


