# Structure from Motion

## 1 Introduction
We will implement key elements of structure from motion. 
Structure from motion takes a set of images and compute camera poses and 3D point cloud, simultaneously. 
It has been widely used in computer vision, graphics, robotics, and even medical imaging.
We will implement the key elements in the structure from motion: 
to estimate fundamental matrix and essential matrix, point triangulation, camera localization via PnP, and nonlinear refinement.

## 2 Techincal Details
The script run **sfm.m** sequentially calls **EstimateFundamentalMatrix**, **EssentialMatrixFromFundamentalMatrix**, 
**LinearTriangulation**, **LinearPnP**, and **NonlinearTriangulation**.

## 2.1 Fundamental Matrix Estimation
Given N >= 8 correspondences between two images, x1 <-> x2, implement the following
function that linearly estimates a fundamental matrix **F**, such that x2^T F x1 = 0:

    F = EstimateFundamentalMatrix(x1, x2)
- (INPUT) x1 and x2 : N * 2 matrices whose row represents a correspondence
- (OUTPUT) F : 3 * 3 matrix with rank 2

The fundamental matrix can be estimated by solving linear least squares (Ax = 0).
Because of noise on correspondences, the estimated fundamental matrix can be rank
3. The last singular value of the estimated fundamental matrix must be set to zero to
enforce the rank 2 constraint.  
**NOTE: normalize the fundamental matrix such that
|F| = 1.**

## 2.2 Essential Matrix Estimation
Given the fundamental matrix computed by estimate E = K^T F K :  

    E = EssentialMatrixFromFundamentalMatrix(F, K)
- (INPUT) K : 3 * 3 camera intrinsic parameter  
- (INPUT) F : fundamental matrix  
- (OUTPUT) E : 3 * 3 essential matrix with singular values (1,1,0)

An essential matrix can be extracted from a fundamental matrix given camera intrinsic
parameter, K. Due to noise in the intrinsic parameters, the singular values of the essential 
matrix are not necessarily (1,1,0). The essential matrix can be corrected by
reconstructing it with (1,1,0) singular values.  
**NOTE: normalize the essential matrix such that |E| = 1.**

## 2.3 Linear Triangulation
We will provide the relative camera transform between image 1 (R1 = I3;C1 = O3)
and image 2 (C2;R2). Given the relative transformation and point correspondences,
x1 <-> x2, triangulate 3D points using linear least squares:  

    X = LinearTriangulation(K, C2, R2, x1, x2)
- (INPUT) C1 and R1: the first camera pose
- (INPUT) C2 and R2: the second camera pose
- (INPUT) x1 and x2: two N * 2 matrices whose row represents correspondence
between the first and second images where N is the number of correspondences
- (OUTPUT) X: N * 3 matrix whose row represents 3D triangulated point

## 2.4 Linear Camera Pose Estimation
We will register the image 3 into the reconstructed coordinate system using 2D-3D
correspondences, x <-> X. The camera pose estimation uses linear least squares to
compute the camera projection matrix **P**, and factors out camera intrinsic
parameter: K^-1 P = [ R t ]
where t is camera translation in the camera coordinate
system, i.e., C = -R^T t. R must be cleaned up such that it is orthogonal matrix with
the determinant 1 and t must be rescaled:

    R = U D V^T
    Rc = U V^T; tc = t/D1,1 if det(U V^T) = 1
    Rc = -U V^T; tc = -t/D1,1 if det(U V^T) = 1
where D1;1 is the first singular value of the R. Rc and tc are the cleaned-up rotation
and translation for the camera.    
    
**IMPORTANT NOTE: While theoretically you can use the x directly when solving for the P matrix
then use the K matrix to correct the error, this is more numeically
unstable, and thus it is better to calibrate the x values before the computation of P then
extract R and t directly. This means using instead of x, using the values xc = K^-1 x.**
    
    [C R] = LinearPnP(X, x, K)
- (INPUT) X and x: N * 3 and N * 2 matrices whose row represents correspondences between 3D and 2D points, respectively
- (INPUT) K: intrinsic parameter
- (OUTPUT) C and R: camera pose (C;R)

## 2.5 Nonlinear Triangulation
Given three camera poses and linearly triangulated points, X, refine the locations of
the 3D points that minimizes reprojection error:

    X = NonlinearTriangulation(K, C2, R2, C3, R3, x1, x2, x3, X0)
- (INPUT) C2 and R2: the second camera pose
- (INPUT) C3 and R3: the third camera pose
- (INPUT) x1, x2, and x3: N * 2 matrices whose row represents correspondence
between the first, second, and third images where N is the number of correspondences
- (INPUT and OUTPUT) X: N * 3 matrix whose row represents 3D triangulated
point

## Visualizing Results
We provide two visualization tools: **DisplayCorrespondence** and **Display3D**. These
functions display reprojection error onto an image and 3D camera and points, respectively.
