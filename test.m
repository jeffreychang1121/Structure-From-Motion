%% main function
close all
folder = 'data';
% extract data from txt file
[Mu, Mv, V, RGB] = ParseData(folder);
% intrinsic matrix
K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];
% camera x
cmx = 1;
% camera y 
cmy = 2; 

% correspondence from image1 and image2
idx = find(V(:,cmx) == 1 & V(:,cmy) == 1);
% number of correspondence
num = size(idx,1);
% correspondence x1
x1 = zeros(num,2);
x1(:,1) = Mu(idx,cmx);
x1(:,2) = Mv(idx,cmx);
% correspondence x2
x2 = zeros(num,2);
x2(:,1) = Mu(idx,cmy);
x2(:,2) = Mv(idx,cmy);
% match outlier rejection via RANSAC
[y1, y2, idx] = GetInliersRANSAC(x1, x2);
% update RGB correspond to y1, y2
RGB2 = RGB(idx,:);

% fundamental matrix estimation
F = EstimateFundamentalMatrix(y1,y2);
% essential matrix estimation
E = EssentialMatrixFromFundamentalMatrix(F, K);
% camera pose extraction
% 4 possibilities 
[Cset, Rset] = ExtractCameraPose(E);
% linear triangulation to construct Xset
R1 = eye(3);
C1 = zeros(3,1);
Xset = cell(4,1);
for i = 1 : 4
Xset{i} = LinearTriangulation(K, C1, R1, Cset{i}, Rset{i}, y1, y2);
% PlotCamerasAndPoints( {[0;0;0],Cset{i}}, {eye(3),Rset{i}}, Xset{i}, 1)
end

% camera pose disambiguation
[C, R, XO, idx3] = DisambiguateCameraPose(Cset, Rset, Xset);
y1 = y1(idx3,:);
y2 = y2(idx3,:);
RGB2 = RGB2(idx3,:);
% error accumulation issue

X_abs = abs(XO(:,1));
Y_abs = abs(XO(:,2));
Z_abs = abs(XO(:,3));
idx2 = find((X_abs) < (mean(X_abs) + 2 * std(X_abs))...
          & (Y_abs) < (mean(Y_abs) + 2 * std(Y_abs))...
          & (Z_abs) < (mean(Z_abs) + 2 * std(Z_abs)));
XO_optimize = XO(idx2,:);
y1_optimize = y1(idx2,:);
y2_optimize = y2(idx2,:);
RGB_optimize = RGB2(idx2,:);

% plot figures
Img1=imread(strcat('image000000', num2str(cmy), '.bmp'));
Img2=imread(strcat('image000000', num2str(cmx), '.bmp'));
% reprojection of the relative camera frame
plot_reprojection(Img1, R, C, K, XO_optimize, y2_optimize)
% reprojective of the based camera frame
figure;
plot_reprojection(Img2, eye(3), [0;0;0], K, XO_optimize, y1_optimize)
figure;
PC3Dshow(XO_optimize, {[0;0;0],C}, {eye(3),R}, uint8(RGB_optimize))

PlotCamerasAndPoints( {[0;0;0],C}, {eye(3),R}, XO_optimize, 1)
