%this is test to generate 2D point with 3D points

points_3d = [1,2,3
    4,8,5
    12,9,6
    6,12,9
    23,45,76
    32,24,23
    52,15,32
    34,15,54,
    34,12,23
    43,12,45
    12,45,32
    65,23,51
    54,23,13]';

points_3d_homo = [points_3d; ones(1,size(points_3d,2))]




K = [482.096858 0 456.548301
    0.000000, 482.307115, 364.185589
    0.000000, 0.000000, 1.000000];


%%% Cam pose 1

% no rotation
% at the origin point of the world frame

camera1_rotation_in_world_frame = [0, 0, 0]; % rpy: roll, pitch, yaw
camera1_position_in_world_frame = [0, 0, 0];


R1 = eye(3);

P = K*R1*[eye(3) , -camera1_position_in_world_frame'];

point_2d_homogenous = P*points_3d_homo;
point_2d_v1 = point_2d_homogenous;

for(i  = 1:size(point_2d_v1,2))
    point_2d_v1(:,i) = point_2d_v1(:,i) / point_2d_v1(3,i);
end

point_2d_v1 = point_2d_v1(1:2,:);




%%% Cam pose 2

% with rotation
% not at the origin point of the world frame

camera2_rotation_in_world_frame = [0.1,0.2,0.2]; % rpy: roll, pitch, yaw
camera2_position_in_world_frame = [0.4,0.5,0.7];

R_rpy = camera2_rotation_in_world_frame;
roll = R_rpy(1);
pitch = R_rpy(2);
yaw = R_rpy(3);



%convert rodrigues vector to rotation matrix
R_roll = [cos(roll) -sin(roll) 0
    sin(roll) cos(roll) 0
    0 0 1];

R_pitch = [cos(pitch) 0 sin(pitch)
    0 1 0
    -sin(pitch) 0 cos(pitch)]

R_yaw = [cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw) 0
    0 0 1]

R2 = R_roll * R_pitch * R_yaw



% PROJECTION
% Multiple view geometry in computer vision 2nd version
% x = P*X

% form1: P = K*R*[ I | -C ], with C as the camera position in world from
% form2: P = K*[ R | t ], with t as the negative of the translation of the
% camera from world origin to its position. So t = -R * C. So the trasform
% T is actually [ R | t ] or [ R | -R*C ].

P_form1 = K*R2*[eye(3) , -camera2_position_in_world_frame'];
P_form2 = K*[R2 , -R2*camera2_position_in_world_frame'];


% P_form1 and P_form2 are the same, here take P_form1

P = P_form1;

point_2d_homogenous = P*points_3d_homo;



point_2d_v2 = point_2d_homogenous;

for(i  = 1:size(point_2d_v2,2))
    point_2d_v2(:,i) = point_2d_v2(:,i) / point_2d_v2(3,i);
end

point_2d_v2 = point_2d_v2(1:2,:);


