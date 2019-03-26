function [Cset_refine, Rset_refine, Xset_refine] = BundleAdjustmentWithNumericalJacobian(........)
%% construct parameters for optimization function
% e.g. for BA, the parameter should be a vector contains camera and 3D points information
% training parameter's size should be (7 x number of cameras) + (4 x number of 3D points)

%% Encode the input arguments into a parameter vector for the optimization
params0 = zeros(1, 7 * num_camera + 3 * num_points);

........


%% Construct Jacobian Matrix
[Jaco] = createJacobianBA(........);


%% Define an optimizer (feel free to change below parameters)
opts = optimoptions(@fsolve, 'Display', 'iter', 'JacobPattern', Jaco, 'Algorithm', ...
            'trust-region-reflective', 'PrecondBandWidth', 1 , 'MaxIter', 60);

%% Objective function (e.g. reprojection error)
f = @(params)err_reproject_BA(params, ........);   % the params should have the same format as params0 you defined

%% Solver
params_opt = fsolve(f, params0, opts);

%% Decode parameters into rotation, translation and 3D point cloud so on
........

Cset_refine = ...;
Rset_refine = ...;
Xset_refine = ...;

end


%% function to compute reprojection error
function [err] = err_reproject_BA(params, ........)
........

err = [];

for i = 1 : num_3Dpoint
  .......

  for j = 1 : num_cameras
    .......

    err = [err, err_ij_u, err_ij_v, .....];
  end

end

end


%% Jacobian construction
function [J] = createJacobianBA(........)
% use the sparse matrix to build the Jacobian
% reference link: https://www.mathworks.com/help/matlab/ref/sparse.html

.......

end
