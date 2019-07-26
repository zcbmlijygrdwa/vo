% use triangulation to choose correct solution
% triangulate 2d point pairs into 3D, the solution that produce most
% positive depth can be the correct solution.
%%% https://www.cnblogs.com/houkai/p/6665506.html 


%projection matrix 1

P1 = K* ([eye(3) zeros(3,1)]);

%projection matrix 2

P2 = K* (T4);

x1 = point_2d_v1(1,1);
y1 = point_2d_v1(2,1);

x2 = point_2d_v2(1,1);
y2 = point_2d_v2(2,1);

p1_r1 = P1(1,:);
p1_r2 = P1(2,:);
p1_r3 = P1(3,:);

p2_r1 = P2(1,:);
p2_r2 = P2(2,:);
p2_r3 = P2(3,:);


test1 = x1*(p1_r3*X) - p1_r1*X
test2 = y1*(p1_r3*X) - p1_r2*X
test3 = x1*(p1_r2*X) - y1*(p1_r1*X)



test4 = x2*(p2_r3*X) - p2_r1*X
test5 = y2*(p2_r3*X) - p2_r2*X
test6 = x2*(p2_r2*X) - y2*(p2_r1*X)




A = [x1*p1_r3 - p1_r1
    y1*p1_r3 - p1_r2
    x2*p2_r3 - p2_r1
    y2*p2_r3 - p2_r2];

[U,S,V] = svd(A);


x_3d = V(:,end); %here x_3d is normalized, |x_3d| = 1;

x_3d = x_3d/(x_3d(4));