function [C, R, XO, idx] = DisambiguateCameraPose(Cset, Rset, Xset)
% Cset and Rset: four configurations of camera centers and rotations
% Xset: four sets of triangulated points from four camera pose
% C and R: the correct camera pose (points that are in front of camera)
% XO: the triangulated points from the camera pose

% number of points
num = size(Xset{1},1);

% position in front of the second camera 
Z = zeros(num,4);
% position in front of the first camera
X = zeros(num,4);

for i = 1 : 4
r3 = Rset{i}(3,:);
X(:,i) = Xset{i}(:,3);
Z(:,i) = (r3 * (Xset{i} - Cset{i}')')';
end

% number of points that are in front of the camera
nx = [nnz(X(:,1) > 0),nnz(X(:,2) > 0),nnz(X(:,3) > 0),nnz(X(:,4) > 0)];
nz = [nnz(Z(:,1) > 0),nnz(Z(:,2) > 0),nnz(Z(:,3) > 0),nnz(Z(:,4) > 0)];
% total number
n = nx + nz;
[~, index] = max(n);

C = Cset{index};
R = Rset{index};
XO = Xset{index};

% take out points that are behind the camera
idx = find(XO(:,3) > 0 & (R(3,:) * (XO - C')')' > 0);
XO = XO(idx,:);
   
end