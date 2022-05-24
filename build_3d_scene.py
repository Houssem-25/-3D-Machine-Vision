# Objective: Similar to estimate trajectory, using the absolute poses and the 3D point cloud of the features, build the 3D map of the scene.
# TO inverse project the features:
# 1- For one image:
#          * for each feature back-project the feature using the inverse of the camera parameters and the depth.
#          * Transform the 3D points into the world reference frame.
# ********************************************************************
# 2- for all images :
#          * get the 3D points of the back-projected features
#          * plot the 3D trajectory of the robot and the 3D points of the scene.
