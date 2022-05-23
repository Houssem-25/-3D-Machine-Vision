# 1. Create the Kinect device class
# 2. Create the capture point cloud function that returns the point cloud using the kinect
# 3. Create the function that transforms the point cloud using a twist vector
# 4. Using ICP estimate the pose and compare it to the ground truth
# 5. What are the drawbacks of the current implementation ? how can you make it better ?
#-----------------------
# 6. Get two seccessive acquisatation of pointclouds
# 7. Use ICP to get the pose that transforms the 2nd pointcloud to the first
# 8. Align the two pointcloud
# 9. What conclusion could you make about the alignement and the algorithem what improvement could be make
# 10. Implement the improvement