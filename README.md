# Franka Emika Panda with PyBullet and ROS2
This repository contains the ROS2 nodes which allow to control a simulated Franka Emika Panda inside [PyBullet](https://pybullet.org/wordpress/). 

## Prerequisites

It is required to have a machine with ROS2 installed. The software has been developed and tested on a machine with [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/) and [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html), but other versions should work fine.

## Repository structure
The repository contains three packages
1. The package ```simulation_pkg``` contain the node ```panda_pybullet_node.py```, which manages the PyBullet simulation. Such a node can be launched with a custom set of parameters using the launch file, i.e.,
   ```
   ros2 launch simulation_packages panda_pybullet.launch.xml
   ```
    Once the node is running, the robot joints data will be available on the ROS topic 
    ```/panda/robot/state/joints```, while the dynamics of the robot, i.e. inertia matrix, coriolis and gravity, are published on ```/panda/robot/state/dynamics```. Specifically, the messages published on the two topics are respectively of the type ```PandaJointsInfo``` and ```PandaDynamics```, both defined in the ```simulation_interfaces``` package.

    Moreover, the package contains the DH parameters of the Panda robot in the file ```panda_parameters.py```.

    
  
2. The package ```simulation_interfaces``` contains the definition of the message types ```PandaJointsInfo``` and ```PandaDynamics```. The former is defined in ```PandaJointsInfo.msg``` as  
    ```
    float32 time            
    float32[7] position     # Joint positions
    float32[7] velocity     # Joint velocites
    ```
    while the definition of the latter is provided in ```PandaDynamics.msg``` as 
    ```
    float32 time            
    float32[49] mass_matrix     # Colum major vectorization of the 7x7 mass matrix M(q)
    float32[7] coriolis         # Coriolis and centripetal term C(q, \dot{q})\dot{q}
    float32[7] gravity          # Gravity term G(q)
    ```
3. The package ```example_controllers_pkg``` contains a useful template for a generic ROS2 node which acts as a controller for the robot and some example controllers. 
   - ```generic_controller_node.py``` defines the class ```GenericControllerNode```, which implements useful methods and the essential elements that a controller node must have. In order to write a custom controller, the user can just create a new class which inherits from ```GenericControllerNode``` and overload the method ```compute_reference``` and set a custom value for the ROS parameter ```reference_pub_period```.
   - ```torque_control_node.py``` contains an example of controller which controls the robot so that it reaches a configuration in the joint space, modulating the joint torques according to the Computed Torque strategy.
   - ```cartesian_velocity_controller_node.py``` controls the robot so that its end-effector reaches a desired pose in the cartesian space, providing a reference for the joint velocities. 
   - ```external_force_node.py``` contains a node which periodically (with a period defined by the ROS parameter ```force_generation_period```) publish a constant force (defined by the user through the ROS parameters ```force_x```, ```force_y```, and ```force_z```) on the ```panda/robot/external_force``` topic. 

