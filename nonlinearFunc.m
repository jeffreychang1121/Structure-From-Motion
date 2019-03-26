function [refine_var] = nonlinearFunc(....)
% A pseudo code for nonlinear optimization
% Input - arguments you need
% Output - refine_var to represent the refine variables

%% Pre-process if necessary 
.......

%% Define an optimizer 
% feel free to change parameters or use other optimizers 
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, ...
    'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');


%% Construct objective function (e.g. reprojection error) for minimizing
% e.g. 
% Triangulation: the variable is 3D point cloud estimated from Linear Triangulation 
% below, iterate all 3D points one by one

% number of triangulated points
% XO initial triangulated points
N = size(XO,1);
optimize_3D = zeros(N,3);

for i = 1 : N
  initial_3D = XO(i,:); % initial estimate

  % reprojection error computation, x is the variable you want to optimize
  error_i = @(x)error_reprojection(x, .......);  

  optimize_3D = lsqnonlin(error_i, initial_3D, [], [], opts); % optimize result
end


%% Post-process if necessary
.......


refine_var = .......; 
end


%% compute reprojection error 
function [err] = error_reprojection(X, C, R)
% X: triangulated points
% C and R: camera pose
% err: an array including all reprojection error 
% in all observed frames (e.g. 1, 2, 3, ..., n)

.......

err = [err_1, err_2, ....., err_n];
end