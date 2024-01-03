import numpy as np


"""
Computes the fukll kinematic chain of the mainupulator, given a set of Denavit-Hartenberg parameters
expressed in the Craig's convention [J.Craig et al. - "Introduction to Robotics - 1986] and aligned 
for the indexing in [Khalil et al - "A new geometric notation for open and closed-loop robots" - 1986]. 
For more details, refer to "http://www.diag.uniroma1.it/~deluca/rob1_en/09_DirectKinematics.pdf". 
@param modified_dh_param: list containing dictionaries with the DH parameters for every frame 
                          Example: dh_parameters = [
                                        {"a":0, "d":0.333, "alpha":0, "theta":"q1", "details":"joint_1"},     
                                        {"a":0, "d":0, "alpha":-np.pi/2, "theta":"q2, "details":"joint_2"},
                                        ...
                                        {"a":0.088, "d":0, "alpha":np.pi/2, "theta":"q7", "details":"joint_7"},
                                        {"a":0, "d":0.107, "alpha":0, "theta":0, "details":"end-effector"}
                                    ]
@param q: the configuration of the robot
@return kinematic_chain: list containing the transofmration matrix of every frame w.r.t the base frame
                        kinematic_chain[0]: base wrt base -> identity
                        kinematic_chain[1]: joint_1 wrt base
                                        ...
                        kinematic_chain[7]: joint_7 wrt base
                        kinematic_chain[8]: ee wrt base
"""
def calc_forward_kinematics(modified_dh_params:list, q:np.ndarray)->list:
    kinematic_chain = [np.identity(4)]
    for i in range(0, len(modified_dh_params)):
        a_i, d_i, alpha_i = (modified_dh_params[i]["a"], modified_dh_params[i]["d"], modified_dh_params[i]["alpha"])

        if i == len(modified_dh_params)-1:
            c_i = np.cos(0)
            s_i = np.sin(0)
        else:
            c_i = np.cos(q[i])
            s_i = np.sin(q[i])
        
        T_i = kinematic_chain[-1]@np.array([[c_i, -s_i, 0, a_i],
                                            [s_i*np.cos(alpha_i), c_i*np.cos(alpha_i), -np.sin(alpha_i), -d_i*np.sin(alpha_i)],
                                            [s_i*np.sin(alpha_i), c_i*np.sin(alpha_i), np.cos(alpha_i), d_i*np.cos(alpha_i)],
                                            [0, 0, 0, 1]
                                            ])
        kinematic_chain.append(T_i)
    
    return kinematic_chain

'''
Function which computes the Jacobian of the robot.
NOTE: Franka Emika Panda direct knematics is computed using modified Denavit-Hartenberg convention
(frame i is associated o joint i).
@param kinematic_chain: list containing the transformation matrices 
@return jacobian: Jacobian of the manipulator
'''
def calc_full_jacobian_modified_dh(kinematic_chain:list):
    jacobian = np.zeros((6,7))
    p_e = kinematic_chain[-1][0:3,3]
    for i in range(0,7):
        z_i = kinematic_chain[i+1][0:3,2]   # NOTE: add 1 because the first entry of the list is the identity matrix
        p_i = kinematic_chain[i+1][0:3,3] 
    
        jacobian[0:3,i] = np.cross(z_i, (p_e - p_i))
        jacobian[3:6,i] = z_i

    return jacobian



'''
Function which computes translates a rotation expressed as a 3x3 matrix to its angle-axis representation.
@param R: the rotation matrix
@return angle: scalar which denotes the magnitude of the rotation
@return axis: array of 3 elements which represents the axis of the rotation
'''
def rotmat_to_angle_axis(R:np.ndarray) -> (float, np.ndarray):

    if np.allclose(R, np.eye(3), rtol=1e-3, atol=1e-3):
        return 0, np.array([1, 0, 0])
    
    elif np.isclose(np.trace(R), -1): # Check if the trace is equal to -1
        angle = np.pi
        if not np.isclose(R[2,2], -1):    # if r33 is different from -1
            axis = 1/(np.sqrt(2*(1+R[2,2]))) * np.array([R[0,2], R[1,2], 1 + R[2,2]])
            return angle, axis 

        if not np.isclose(R[1,1], -1):    # if r22 is different from -1
            axis = 1/(np.sqrt(2*(1+R[1,1]))) * np.array([R[0,1], 1+ R[1,1], R[2,1]])
            return angle, axis 
        
        if not np.isclose(R[0,0], -1):    # if r11 is different from -1
            axis = 1/(np.sqrt(2*(1+R[0,0]))) * np.array([1+R[0,0], R[1,0], R[2,0]])
            return angle, axis 
        
    else:
        angle = np.arccos(0.5*(np.trace(R)-1))
        axis_cross_prod = (1/(2*np.sin(angle)))*(R - R.T)
        axis = np.array([axis_cross_prod[2,1], axis_cross_prod[0,2], axis_cross_prod[1,0]])

        return angle, axis

'''
Function which computes the damped pseudoinverse.
@param mat: the matrix that must be inverted
@param damping: the damping factor
@return res: the resulting matrix
'''
def damped_pseudoinv(mat:np.ndarray, damping:float = 1e-4) -> np.ndarray:
    res = mat.T @ np.linalg.inv(mat@mat.T + damping*np.eye(mat.shape[0]))
    return res