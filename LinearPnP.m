function [C, R] = LinearPnP(X, x, K)
% X - size (N x 3) matrix of 3D points
% x - size (N x 2) matrix of 2D points whose rows correspond with X
% K - size (3 x 3) camera calibration (intrinsics) matrix
% C - size (3 x 1) pose transation
% R - size (3 x 1) pose rotation

num = size(X,1); % number of points
X = [X, ones(num,1)];
x = [x, ones(num,1)];

a_matrix = [zeros(num,4)          -X    x(:,2).*X;
                      X  zeros(num,4)  -x(:,1).*X;
             -x(:,2).*X    x(:,1).*X  zeros(num,4)];

[~, ~, V] = svd(a_matrix);

p_matrix = V(:,end);
p_matrix = reshape(p_matrix,3,4);

R = K \ p_matrix(:,1:3);

[u, s, v] = svd(R);
R = u * v'; % rotation matrix clean up
T = (K \ p_matrix(:,4))/s(1,1); % rotation matrix recover
C = -R' * T;

end