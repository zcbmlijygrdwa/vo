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

%use SVD to get e
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




%recover R t
[U,S,V] = svd(e);

yaw = pi/2;
R_z = [cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw) 0
    0 0 1];

R1 = U*R_z*V'

Z = R_z;
Z(3,3) = 0;
t1 = U*Z*U'


yaw = -pi/2;
R_z = [cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw) 0
    0 0 1];

R2 = U*R_z*V'

Z = R_z;
Z(3,3) = 0;


t2 = U*Z*U'


