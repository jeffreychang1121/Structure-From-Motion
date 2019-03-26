function [X] = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
% C1 and R1: the first camera pose
% C2 and R2: the second camera pose
% x1 and x2: n by 2 matrices represent correspondences between two images
% X: n by 3 matrix represents 3D triangulated point

% number of points
num = size(x1,1);
X = zeros(num,3);

% construct P1 and P2 matrices: 3 by 4 = K * [R | t]
P1 = K * [R1, -R1 * C1];
P2 = K * [R2, -R2 * C2];

for i = 1 : num
% construct antisymmetric matrices for x1 and x2
X1 = [0 -1 x1(i,2); 1 0 -x1(i,1); -x1(i,2) x1(i,1) 0];
X2 = [0 -1 x2(i,2); 1 0 -x2(i,1); -x2(i,2) x2(i,1) 0];

% point triangulation
A_matrix = [X1 * P1; X2 * P2];
[~, ~, V] = svd(A_matrix);
x = V(:,end)/V(end,end);
X(i,:) = x(1:3)';
end

end