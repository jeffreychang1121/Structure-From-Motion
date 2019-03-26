function [y1, y2, idx] = GetInliersRANSAC(x1, x2)
% given more than 8 correspondences between two images x1, x2
% estimate the inlier correspondence using fundamental matrix
% x1 = n by 2 matrix (pixel coordinate)
% x2 = n by 2 matrix (pixel coordinate)
% y1 = n by 2 matrix inlier correspondence
% y2 = n by 2 matrix inlier correspondence
% idx = index of inlier y1

% number of points
num = size(x1,1);
x1 = [x1,ones(num,1)];
x2 = [x2,ones(num,1)];

% parameters for RANSAC
iteration = 2000;
threshold_error = 0.005;
threshold_inlier_lower = 0.85 * num;

i = 0;
max_inlier = 0;
while i < iteration
sample_index = randperm(num);
% randomly pick 8 correspondences
sample_index = sample_index(1:8)';
sample_x1 = x1(sample_index,1:2);
sample_x2 = x2(sample_index,1:2);
% calculate F based on sample points
F = EstimateFundamentalMatrix(sample_x1,sample_x2);
% estimate error based on F 
error = x2 * F * x1';
error = diag(error);
% calculate inliers
threshold = abs(error) < threshold_error;
inlier = nnz(threshold);
% break loop if get enough inliers
if inlier > threshold_inlier_lower
    y1 = x1(threshold,1:2);
    y2 = x2(threshold,1:2);
    idx = [1:num]';
    idx = idx(threshold);
    break;
end
% store the best result so far
max_inlier = max(inlier, max_inlier);
% update the optimize inliers                
if inlier == max_inlier
    y1 = x1(threshold,1:2);
    y2 = x2(threshold,1:2);
    idx = [1:num]';
    idx = idx(threshold);
end

i = i + 1;
end

end