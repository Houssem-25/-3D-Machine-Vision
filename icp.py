from datetime import datetime
import numpy as np
import time
from scipy.linalg import expm
from sklearn.neighbors import NearestNeighbors

def skew(vect):
    """
    This function should define the 3X3 skew symmetric matrix.
    :param vect: angular velocity (u_1, u_2, u_3)
    :return: 3X3 skew numpy array
    """
    # TODO
    ## your code below
    return np.eye(3)

def SE3_exp(linear_velocity, angular_velocity):
    """
    This is the function where you will define 4X4 homogeneous twist matrix [x]^ and return pose T= e[x]^
    :param linear_velocity: numpy array of size 1x3
    :param angular_velocity: numpy array of size 1x3
    :return: 4x4 numpy array of pose matrix
    """
    # TODO
    ## your code below
    return np.eye(4)

def transform_pointcloud(pointcloud, pose_T):
    """
    This function will be used for transforming point clouds using the pose T
    :param pointcloud: open3d pointcloud object
    :param pose_T: 4x4 numpy array
    :return: open3d pointcloud object
    """
    #TODO
    ## your code below
    return pointcloud

def log(text):
    logtime = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print("[{}] {}".format(logtime, text))

def estimate_rigid_body_transformation(P_ref, P_current):
    residual = P_current[:, :3] - P_ref[:, :3]
    J = np.vstack([np.array([      [1, 0, 0,          0          , P_current[i,2]   , -P_current[i,1] ],
                                   [0, 1, 0, -P_current[i,2] ,           0          ,  P_current[i,0] ],
                                   [0, 0, 1,  P_current[i,1] , -P_current[i,0]  ,     0               ]]) for i in range(len(residual))])
    # TODO Calculate the inverse
    J_pesudo =
    delta_x = -J_pesudo @ residual.flatten()
    dT = SE3_exp(delta_x[0:3], delta_x[3:6])
    return dT, delta_x, residual

def check_convergence_criteria(#TODO define arguments ):
    #TODO derive a stoping criteria this function should return True when the criteria is fulfilled

def simpleicp(P_ref, P_current,max_iterations=1000):
    start_time = time.time()
    T = np.eye(4)
    log("Start iterations ...:")
    for i in range(0, max_iterations):
        dT, delta_x, residuals = estimate_rigid_body_transformation(P_ref, P_current)
        P_current = #TODO : Transform the current point cloud with the estimated dT
        T = dT @ T
        if i > 0:
            #TODO check_the convergence criteria and stop the for loop when the criteria is fulfilled
            #if check_convergence_criteria(#TODO define the needed arguments):
                #log("Convergence criteria fulfilled -> stop iteration!")
                #break
        log("at iteration %d Current residuals %f " % (i,np.linalg.norm(residuals)))
        log("Current translation dx %.19f rotation %.19f" % (np.linalg.norm(delta_x[:3]), np.linalg.norm(delta_x[:3]) ) )
    print("elapsed %fs" % (time.time()- start_time))
    return T

