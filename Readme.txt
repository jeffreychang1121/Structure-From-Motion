%% main function

1. the main function is called test.m

2. there are two variable need to be adjusted:
   cmx: the based camera
   cmy: the relative camera

3. add path of the data file before running the program

4. the figure should be named as 'image000000',number,'.bmp'

5. the txt files should be named as 'matching',number,'.bmp'
   Note: 
   the data in the txt file should only include the matching data, the first line showing how many feature points should be deleted

6. four plots would be created:
   (1) reprojection of the relative camera frame
   (2) reprojection of the based camera frame
   (3) 3D plot of the feature points
   (4) top view of the feature points

7. the "DisambiguateCameraPose" function was adjusted by 
   giving an additional output "idx", which is used for 
   getting rid of the points that are behind the camera
