function [C, R, idx] = PnPRANSAC(X, x, K)
% X and x: Nx3 and Nx2 matrices represents 
% correspondences between 3D and 2D points
% K: intrinsic parameter
% C and R: camera pose (translation, rotation matrix)

% number of points
num = size(X,1);
X = [X,ones(num,1)];
x = [x,ones(num,1)];

% parameters for RANSAC
iteration = 2000;
threshold_error = 0.005;
threshold_inlier_lower = 0.85 * num;

i = 0;
max_inlier = 0;
error = zeros(num,1);
while i < iteration
sample_index = randperm(num);
% randomly pick 6 correspondences
sample_index = sample_index(1:6)';
sample_X = X(sample_index,1:2);
sample_x = x(sample_index,1:2);
% calculate C and R based on sample points
[C_tmp, R_tmp] = LinearPnP(sample_X, sample_x, K);
% calculate P based on C and R
P_tmp = K * R_tmp * [eye(3), -C_tmp];

% estimate error based reprojection error
for j = 1 : num
error(j) = (x(j,1) - (P_tmp(1,:) * X(j,:)')/(P_tmp(3,:) * X(j,:)'))^2 ...
         + (x(j,2) - (P_tmp(2,:) * X(j,:)')/(P_tmp(3,:) * X(j,:)'))^2;
end

% calculate inliers
threshold = abs(error) < threshold_error;
inlier = nnz(threshold);
% break loop if get enough inliers
if inlier > threshold_inlier_lower
    Y = X(threshold,1:2);
    y = x(threshold,1:2);
    idx = [1:num]';
    idx = idx(threshold);
    break;
end
% store the best result so far
max_inlier = max(inlier, max_inlier);
% update the optimize inliers                
if inlier == max_inlier
    Y = X(threshold,1:2);
    y = x(threshold,1:2);
    idx = [1:num]';
    idx = idx(threshold);
end

i = i + 1;
end

% update optimize C and R
[C, R] = LinearPnP(Y, y, K);

end