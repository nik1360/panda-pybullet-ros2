import pybullet as pb
import numpy as np


class PybulletInterface():
    def __init__(self):
        self.physics_client = None
        self.objectsID_in_scene = []

    '''
    Connect to the physics engine.
    @param use_gui: if true, the GUI is rendered
    '''
    def connect(self, use_gui=True):
        if use_gui:
            mode = pb.GUI
        else:
            mode = pb.DIRECT
        self.physics_client = pb.connect(mode)        
    
    '''
    Initialize the simulation scene.
    @param real_time: the simulation is executed in realtime
    @param timestep: simulation timestep [m]
    @param gravity: if True, enable gravity
    '''
    def init_scene(self, real_time=False, timestep=1./240., gravity = True):
        if real_time:
            # Simulation is executed in realtime
            pb.setRealTimeSimulation(1)
        else:
            # Simulation requires stepSimulation() to advance
            pb.setRealTimeSimulation(0)
            pb.setTimeStep(timestep)
        if gravity:
            pb.setGravity(0., 0., -9.81)
        else:
            pb.setGravity(0., 0., 0.)
        
        pb.resetDebugVisualizerCamera(cameraDistance=1.6, cameraYaw=118.8, 
            cameraPitch=-20.4, cameraTargetPosition=[0.24, -0.07, 1.07])

    
    '''
    Perform a simulation step. This function is neede only if the simulation
    is NOT executed in realtime.
    '''
    def step(self):
        pb.stepSimulation()
    
    '''
    Loads and URDF file and adds it to the simulation.
    @param urdf_path: filepath of the .urdf
    @param position: position with respect to the world frame {w}
    @param orientation: quaternion which express the orientation 
                        with respect to the world frame {w}
    @param use_fixed_base: if true, the base link of the object is static 
    @return objID: ID of the object inside Bullet 
    '''
    def add_object_to_scene(self, urdf_path, position=[0,0,0], 
        orientation=[0,0,0,1], use_fixed_base=True):

        objID = pb.loadURDF(urdf_path, basePosition=position, 
            baseOrientation=orientation, useFixedBase=use_fixed_base)
        self.objectsID_in_scene.append(objID)

        return objID
    
    '''
    Make an object not collidable with ALL the objects present in the scene.
    @param objID: ID of the desired object
    '''
    def make_object_non_collidable_all(self, objID):
        # Find the number of links of the object objID
        n_links_objID = pb.getNumJoints(objID) # in pybullet n_joints = n_links

        # Disable the collision between the objID and the other objects in the scene
        for o in self.objectsID_in_scene:
            if o != objID:
                # Find the number of links of the object o
                n_links_o = pb.getNumJoints(o) # in pybullet n_joints = n_links
                for i in range(-1, n_links_o): # -1 is the base link
                    for j in range(-1, n_links_objID):
                        pb.setCollisionFilterPair(o, objID, i, j, 0)

    '''
    Disable collisions between two objects
    @param objID_A: ID of the object A
    @param objID_B: ID of the object B
    '''
    def make_object_non_collidable(self, objID_A, objID_B):
        # Find the number of links of the object objID
        n_links_objID_A = pb.getNumJoints(objID_A) # in pybullet n_joints = n_links
        n_links_objID_B = pb.getNumJoints(objID_B) 
        # Disable collisions between object A and object B
        for i in range(-1, n_links_objID_B): # -1 is the base link
            for j in range(-1, n_links_objID_A):
                pb.setCollisionFilterPair(objID_A, objID_B, i, j, 0)

    '''
    Override the physics of the object and change the position and the orientation
    of its base link.
    @param objID: ID of the object
    @param new_position: desired position
    @param new_orientation: new orientation expressed as a quaternion
    '''
    def change_object_pose(self, objID, new_position, new_orientation):
       pb.resetBasePositionAndOrientation(objID, new_position, new_orientation)

    '''
    Returns the minimum distance (squared) between object A and object B
    @param objID_A: ID of the first object
    @param objID_B: ID of the second object
    @return min_dist: square of the minimum distance
    '''
    def minimum_distance_between_objects(self, objID_A, objID_B):
        points = pb.getClosestPoints(objID_A, objID_B, 10)

        min_dist = 10e5
        for i in range(0, len(points)):
            p_A = points[i][5] # point on body A
            p_B = points[i][6] # point on body B
        
            dist = 0
            for j in range(0,3):
                dist += (p_A[j] - p_B[j])**2 
            
            min_dist = min(dist, min_dist)

        return min_dist

    '''
    Retrieve joint positions, velocities and torques
    @param robotID: ID of the robot
    @return q: joint positions
    @return dq: joint velocities
    @return tau: joint torques
    '''
    def get_robot_state(self, robotID):
        n_joints = pb.getNumJoints(robotID)
        q = []
        dq = []
        tau = []
        for i in range(0, n_joints):
            info = pb.getJointInfo(robotID, i)
            # if the joint is a revolute joint, retrieve its status 
            if info[2]==pb.JOINT_REVOLUTE:
                state = pb.getJointState(robotID, i)
                q.append(state[0])
                dq.append(state[1])
                tau.append(state[3])
        return np.array(q, dtype=np.float32), np.array(dq, dtype=np.float32), np.array(tau, dtype=np.float32)

    '''
    Controls the robot modulating joint target velocities. 
    @param robotID: ID of the robot
    @param dq_target: joint target_velocities
    @param joint_indices: indices of the revolute joints
    @param max_torques: maximum motor torques used to reach the target velocities
    '''
    def send_velocity_control(self, robotID:int, dq_target:list, joint_indices:list, max_torques:list=None) -> None:
        if max_torques is None:
            pb.setJointMotorControlArray(bodyUniqueId=robotID, 
                jointIndices=joint_indices, controlMode=pb.VELOCITY_CONTROL, 
                targetVelocities=dq_target)
        else:
            pb.setJointMotorControlArray(bodyUniqueId=robotID, 
                jointIndices=joint_indices, controlMode=pb.VELOCITY_CONTROL, 
                targetVelocities=dq_target, forces=max_torques)
    
    '''
    Controls the robot modulating joint target positions. 
    @param robotID: ID of the robot
    @param q_target: joint target positions
    @param joint_indices: indices of the revolute joints
    @param max_torques: maximum torque with which the target position is reached
    '''
    def send_position_control(self, robotID, q_target, joint_indices, max_torques=None):
        if max_torques is None:
            pb.setJointMotorControlArray(bodyUniqueId=robotID, 
                jointIndices=joint_indices, controlMode=pb.POSITION_CONTROL, 
                targetPositions=q_target)
        else:
            pb.setJointMotorControlArray(bodyUniqueId=robotID, 
                jointIndices=joint_indices, controlMode=pb.POSITION_CONTROL, 
                targetPositions=q_target, forces = max_torques)

    '''
    Controls the robot modulating joint torques. 
    @param robotID: ID of the robot
    @param tau: joint torques
    @param joint_indices: indices of the revolute joints
    '''
    def send_torque_control(self, robotID:int, tau:list, joint_indices:list):
        pb.setJointMotorControlArray(bodyUniqueId=robotID, 
            jointIndices=joint_indices, controlMode=pb.TORQUE_CONTROL, 
            forces=tau)
    
    '''
    Disable velocity motors to be able to perform torque control.
    From the Pybullet's quickstart guide:
    "By default, each revolute joint and prismatic joint is motorized using a velocity
    motor. You can disable those default motor by using a maximum force of 0."
    @param robotID: ID of the robot
    @param joint_indices: indices of the revolute joints
    '''
    def disable_default_motors(self, robotID:int, joint_indices:list, friction_coeff:float):
        # Disable the motors for torque control:
        pb.setJointMotorControlArray(robotID,
                                        joint_indices,
                                        pb.VELOCITY_CONTROL,
                                        forces=[friction_coeff]*len(joint_indices))
    
    '''
    Performs the inverse dynamics of the robot.
    @param robotID: ID of the robot
    @param q: current position of the joints 
    @param dq: current velocity of the joints
    @param ddq_target: desired acceleration for the joints
    @return tau: result of the inverse dynamics
    '''
    def inverse_dynamics(self, robotID, q, dq, ddq_target):
    
        tau = pb.calculateInverseDynamics(bodyUniqueId=robotID,
            objPositions=q, objVelocities=dq,
            objAccelerations=ddq_target)

        return tau
        
    '''
    Retrieve informations about the robot joints.
    @param robotID: ID of the robot
    @return revolute_joint_indices: indices of the revolute joints of the robot
    @return lower_pos: lower bound for the position of the revolute joints [rad]
    @return upper_pos: upper bound for the position of the revolute joints [rad]
    @return max_vel: maximum velocity for the joint [rad/s]
    @return max_torques: maximum torques that the revolute joints can exert [Nm]
    '''
    def get_joint_infos(self, robotID):
        n_joints = pb.getNumJoints(robotID)

        revolute_joint_indices = []
        lower_pos = []
        upper_pos = []
        max_vel = []
        max_torques = []

        for i in range(0, n_joints):
            info = pb.getJointInfo(robotID, i)
            
            if info[2]==pb.JOINT_REVOLUTE:
                revolute_joint_indices.append(i)
                lower_pos.append(info[8])
                upper_pos.append(info[9])
                max_torques.append(info[10])
                max_vel.append(info[11])

        return revolute_joint_indices, lower_pos, upper_pos, max_vel, max_torques

    '''
    Enable the force sensor of the joints
    @param robotID: ID of the robot
    @param joint_indices: indices of the revolute joints
    '''
    def enable_force_sensors(self, robotID, joint_indices):
        for i in joint_indices:
            pb.enableJointForceTorqueSensor(robotID, i, 1)

    '''
    Disconnect from the physics engine.
    '''
    def disconnect(self):
        pb.disconnect()
        self.physics_client = None


    '''
    Retrieve the orientation of a certain link
    '''
    def get_link_orientation(self, robotID, link_index):

        infos = pb.getLinkState(bodyUniqueId=robotID, linkIndex=link_index, 
            computeForwardKinematics=True)
        # Retrieve the orientation (quaternion) of the URDF frame wrt the world frame
        pos = infos[0]
        quaternion = infos[1] 
        
        # Obtain the 3x3 rotation matrix
        R_list = pb.getMatrixFromQuaternion(quaternion)

        return quaternion, R_list
    

    '''
    Considering the robot model
        M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau 
    this function returns M(q), C(q, \dot{q}), and G(q).
    '''
    def get_robot_model(self, robot_id:int, q:list, dq:list) -> (np.ndarray, np.ndarray, np.ndarray):        
        # Compute the mass matrix
        mass_matrix = np.array(pb.calculateMassMatrix(bodyUniqueId=robot_id, objPositions=q), dtype=np.float32)

        # Compute the term \eta(q,\dot{q}) = C(q,\dot{q})\dot{q} + G(q) using the inverse 
        # dynamics with \ddot{q} = 0
        eta = np.array(pb.calculateInverseDynamics(bodyUniqueId=robot_id, objPositions=q, objVelocities=dq, 
                                          objAccelerations=[0.]*len(q)), dtype=np.float32)

        # Compute the term G(q) using invers dynamics with \dot{q}=0 and \ddot{q}=0
        g = np.array(pb.calculateInverseDynamics(bodyUniqueId=robot_id, objPositions=q, objVelocities=[0.]*len(q), 
                                          objAccelerations=[0.]*len(q)), dtype=np.float32)
        # Calculate coriolis as  C(q,\dot{q})\dot{q} = \eta(q,\dot{q}) - G(q)
        coriolis = eta - g

        return mass_matrix, coriolis, g






    '''
    Retrieves the inertia matrix of the robot
    @param RobotID: ID of the robot
    @param q: joint positions
    @return M: mass matrix of the manipulator
    '''
    def get_mass_matrix(self, robotID, q):
        M = pb.calculateMassMatrix(bodyUniqueId=robotID, objPositions=q)
        return np.array(M, dtype=np.float32)


    '''
    Considering the dynamic equation 
        M(q)*ddq + N(q,dq)  = tau
    it recovers the Coriolis, centripetal and gravity term N(q, dq)
    performing inverse dynamics with desired acceleration for all the 
    joints equal to 0.
    @param robotID: ID of the robot
    @param q: joint positions
    @param dq: joint velocities
    @return N: Coriolis, centripetal and gravity term
    '''
    def get_coriolis_centripetal_gravity(self, robotID, q, dq):
        n_joints = len(q)
        zero_acc = [0.]*n_joints
        N = pb.calculateInverseDynamics(robotID, q, dq, zero_acc)

        return np.array(N, dtype=np.float32)

    '''
    Returns the position and the orientation of the end effector 
    with respect to the world reference frame.
    @param robotID: ID of the robot
    @param ee_index: index of the link which is considered as end-effector
    @return pos: position of the end effector in world frame 
    @return orient_quat: orientation (quaternion) of the end effector frame with respect to world frame
    @return rot_matrix: returns the rotation matrix which express the orientation of the end effector
    '''
    def get_ee_position_orientation(self, robotID, ee_index):
        res = pb.getLinkState(bodyUniqueId=robotID,
            linkIndex=ee_index)
        pos = res[0]
        orient_quat = res[1]
        rotmat = pb.getMatrixFromQuaternion(orient_quat)
        rot_matrix = [[rotmat[0], rotmat[1], rotmat[2]],
                      [rotmat[3], rotmat[4], rotmat[5]],
                      [rotmat[6], rotmat[7], rotmat[8]]]

        return pos, orient_quat, rot_matrix

    def reset_joints_state(self, robot_id, q_init:list, joint_indices:list):
        for i in range(len(joint_indices)):
            pb.resetJointState(bodyUniqueId=robot_id, jointIndex=joint_indices[i], targetValue=q_init[i])

    def apply_force(self, robot_id, link_index:int, force:list, position:list):
        pb.applyExternalForce(objectUniqueId=robot_id, linkIndex=link_index, forceObj=force, posObj=position,
                              flags=pb.WORLD_FRAME)   