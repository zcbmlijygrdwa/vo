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


%%% Transform 1

%R1 = I
%t1 = 0;
R = eye(3);
t = [0,0,0];
T = [R t']

point_2d_homogenous = K*T*points_3d_homo;
point_2d_v1 = point_2d_homogenous;

for(i  = 1:size(point_2d_v1,2))
    point_2d_v1(:,i) = point_2d_v1(:,i) / point_2d_v1(3,i);
end

point_2d_v1 = point_2d_v1(1:2,:);




%%% Transform 2

R_rpy = [0.1,0.2,0.2];
roll = R_ryp(1);
pitch = R_ryp(2);
yaw = R_ryp(3);



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

R = R_roll * R_pitch * R_yaw



t = [0.4,0.5,0.7];

T = [R t']

point_2d_homogenous = K*T*points_3d_homo;
point_2d_v2 = point_2d_homogenous;

for(i  = 1:size(point_2d_v2,2))
    point_2d_v2(:,i) = point_2d_v2(:,i) / point_2d_v2(3,i);
end

point_2d_v2 = point_2d_v2(1:2,:);


