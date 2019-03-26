function [F] = EstimateFundamentalMatrix(x1,x2)
% given more than 8 correspondences between two images x1, x2
% estimate the fundamental matrix F
% x1 = n by 2 matrix (pixel coordinate)
% x2 = n by 2 matrix (pixel coordinate)

% n points given
n = size(x1,1);
x1 = [x1,ones(n,1)];
x2 = [x2,ones(n,1)];

A_matrix = [x1(:,1).*x2 x1(:,2).*x2 x2];
[~, ~, V] = svd(A_matrix);
% approximate F
X = V(:,9);
F = reshape(X,3,3);
% make sure rank(F) = 2
[u, s, v] = svd(F);
m = size(s,1);
s = [s(:,1:2),zeros(m,1)];
% SVD clean up
F = u * s * v';

end