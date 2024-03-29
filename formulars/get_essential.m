%this is test to get essential matrix from 8 2D points

% A = [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     [u1*u2 u1*v2 u1 v1*u2 v1 u2 v2 1]
%     ...


A = [];

for(i = 1:size(point_2d_v1,2))
    u1 = point_2d_v1(1,i);
    v1 = point_2d_v1(2,i);
    
    u2 = point_2d_v2(1,i);
    v2 = point_2d_v2(2,i);
    
    
    
    A = [A; u1*u2 u1*v2 u1 v1*u2 v1*v2 v1 u2 v2 1];
    
end

% solve A*e = 0 for e
% use SVD to get e
[U,S,V] = svd(A);
f = V(:,end)
f = reshape(f,3,3)

e = K'*f*K





% A*[e1]
%   [e2]
%   [e3]
%   [e4]
%   [e5]    =   0
%   [e6]
%   [e7]
%   [e8]
%   [e9]


% A*E = 0;




%https://www.cnblogs.com/houkai/p/6665506.html

%recover R t
[U,S,V] = svd(e);

yaw = pi/2;
R_z = [cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw) 0
    0 0 1];

R1 = U*R_z*V';

Z = R_z;
Z(3,3) = 0;
S1 = U*Z*U';

%recover translation from skew symmetric matrix
t1 = [S1(3,2), S1(1,3), S1(2,1)]';

%here t is unified so it losses scale factor

yaw = -pi/2;
R_z = [cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw) 0
    0 0 1];

R2 = U*R_z*V';

Z = R_z;
Z(3,3) = 0;


S2 = U*Z*U';
%recover translation from skew symmetric matrix
t2 = [S2(3,2), S2(1,3), S2(2,1)]';

%here t is unified so it losses scale factor


%two rotations and two translations form 4 solutions of [R | t].

T1 = [R1,t1];
T2 = [R1,t2];
T3 = [R2,t1];
T4 = [R2,t2];




% to verify x' * F * x = 0; (epilopar constrain)
point_2d_v1_homo = [point_2d_v1 ; ones(1,size(point_2d_v1,2))];
point_2d_v2_homo = [point_2d_v2 ; ones(1,size(point_2d_v2,2))];
fundamental_mat_constrain_test_result = point_2d_v1_homo'*f*point_2d_v2_homo;
fundamental_mat_constrain_test_result = det(fundamental_mat_constrain_test_result)



% use triangulation to choose correct solution
% triangulate 2d point pairs into 3D, testing
% with a single point to determine if it is in front of both cameras is sufficient to decide
% between the four different solutions for the camera matrix P
% Multiple view geometry in computer vision, 2nd version, p259

%try T1
x_3d_recover = [];
isTrueSolution = true;
for i = 1:size(point_2d_v1,2)
    
    x1 = point_2d_v1(1,i);
    y1 = point_2d_v1(2,i);
    
    x2 = point_2d_v2(1,i);
    y2 = point_2d_v2(2,i);
    
    x_3d_temp = my_triangulation(x1,y1,x2,y2,K,T1);
    x_3d_recover = [x_3d_recover, x_3d_temp];
    
    
    depth_in_cam_1 = point_depth_from_camera(K,R1,camera1_position_in_world_frame,x_3d_temp(1:3,:))
    depth_in_cam_2 = point_depth_from_camera(K,R2,camera2_position_in_world_frame,x_3d_temp(1:3,:))
    
    if(depth_in_cam_1<0 || depth_in_cam_2<0)
        disp("point found behind a camera, wrong solution.");
        isTrueSolution = false;
        break;
    end
end


if(isTrueSolution)
   disp("all points produce positive distance to cameras, solution found as T1.") 
end


%try T2
x_3d_recover = [];
isTrueSolution = true;
for i = 1:size(point_2d_v1,2)
    
    x1 = point_2d_v1(1,i);
    y1 = point_2d_v1(2,i);
    
    x2 = point_2d_v2(1,i);
    y2 = point_2d_v2(2,i);
    
    x_3d_temp = my_triangulation(x1,y1,x2,y2,K,T2);
    x_3d_recover = [x_3d_recover, x_3d_temp];
    
    
    depth_in_cam_1 = point_depth_from_camera(K,R1,camera1_position_in_world_frame,x_3d_temp(1:3,:));
    depth_in_cam_2 = point_depth_from_camera(K,R2,camera2_position_in_world_frame,x_3d_temp(1:3,:));
    
    if(depth_in_cam_1<0 || depth_in_cam_2<0)
        disp("point found behind a camera, T2 is wrong solution.");
        isTrueSolution = false;
        break;
    end
end

if(isTrueSolution)
   disp("all points produce positive distance to cameras, solution found as T2.") 
end


%try T3
x_3d_recover = [];
isTrueSolution = true;
for i = 1:size(point_2d_v1,2)
    
    x1 = point_2d_v1(1,i);
    y1 = point_2d_v1(2,i);
    
    x2 = point_2d_v2(1,i);
    y2 = point_2d_v2(2,i);
    
    x_3d_temp = my_triangulation(x1,y1,x2,y2,K,T3);
    x_3d_recover = [x_3d_recover, x_3d_temp];
    
    
    depth_in_cam_1 = point_depth_from_camera(K,R1,camera1_position_in_world_frame,x_3d_temp(1:3,:));
    depth_in_cam_2 = point_depth_from_camera(K,R2,camera2_position_in_world_frame,x_3d_temp(1:3,:));
    
    if(depth_in_cam_1<0 || depth_in_cam_2<0)
        disp("point found behind a camera, T3 is wrong solution.");
        isTrueSolution = false;
        break;
    end
end

if(isTrueSolution)
   disp("all points produce positive distance to cameras, solution found as T3.") 
end


%try T4
x_3d_recover = [];
isTrueSolution = true;
for i = 1:size(point_2d_v1,2)
    
    x1 = point_2d_v1(1,i);
    y1 = point_2d_v1(2,i);
    
    x2 = point_2d_v2(1,i);
    y2 = point_2d_v2(2,i);
    
    x_3d_temp = my_triangulation(x1,y1,x2,y2,K,T4);
    x_3d_recover = [x_3d_recover, x_3d_temp];
    
    
    depth_in_cam_1 = point_depth_from_camera(K,R1,camera1_position_in_world_frame,x_3d_temp(1:3,:));
    depth_in_cam_2 = point_depth_from_camera(K,R2,camera2_position_in_world_frame,x_3d_temp(1:3,:));
    
    if(depth_in_cam_1<0 || depth_in_cam_2<0)
        disp("point found behind a camera, T4 is wrong solution.");
        isTrueSolution = false;
        break;
    end
end

if(isTrueSolution)
   disp("all points produce positive distance to cameras, solution found as T4.") 
end


