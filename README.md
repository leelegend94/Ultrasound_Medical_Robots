# Ultrasound_Medical_Robots
Automatic ultrasound scan for vascular tissue.
Codes are verified in the environment:
* Ubuntu 18.04
* ROS melodic
* KUKA LBR IIWA R800/R820 (7/14 Kg)
---
## Introduction
This repo includes implementation of vessel simulator, modified UNet and all corresponding ROS nodes.
| Folder/File | Description |
| :----: | :----: |
| matlab/simGeo_pc2_vis | Simulate a virual vessel in space, calculate and publish the vessel cross section points in PointCloud2 format. |
| matlab/opt_grad.m | Use matlab symbolic toolbox to find gradients of the objective function. |
| matlab/experiment | Scripts for result analysis |
| network/modules | Implementation of UNet archtecture, loss function, and dataset constructor. |
| network/mkDataSet.py | Create dataset from multiple image sets. |
| network/train_eval.ipynb | Training script |
| plugin_ws | ImFusion Plugins: Send Cephasonics image streams via ROS topic. |
|ws/src/iiwa_tools| Modified robot description with tool link. |
|ws/src/iiwa_bringup| 1. Set tool link 2. Upload robot description 3. Transform /iiwa/state/JointPositionVelocity into /joint_states |
| ws/src/us_controller | Calculate target pose according to vessel direction vector. |
| ws/src/us_image_processing/src/igtl_read_node | Transform openIGTLink message to ROS messages. |
| ws/src/us_image_processing/src/image_processing_node_vanilla | Deploy UNet for vessel segmentation. |
| ws/src/us_image_processing/src/estimator_node | Implementation of the proposed optimization problem. |

---
## Prerequirements 

### iiwa_stack
Install iiwa_stack https://github.com/IFL-CAMP/iiwa_stack. Source the workspace.

### NLopt
The optimizer is dependent on NLopt https://nlopt.readthedocs.io/en/latest/. Please follow the instruction to install the library in default location.

### openIGTLink
To compile igtl_read_node, OpenIGTLink library is necessary. Please build from source! https://github.com/openigtlink/OpenIGTLink.
Compile with:

    mkdir build
    cd build
    cmake ..
    make

The compile flag of us_image_processing OpenIGTLink_DIR should be set to the build directory. eg. By default, it is set with 

    set(OpenIGTLink_DIR $ENV{HOME}/.local/OpenIGTLink/build)

## Compile 
	cd <project path>/ws
	catkin_make
---
## Launch 
1. Setup Ultrasound system, make the ros topic of US images available.
2. Setup the following configuration files according to your system:
    - us_image_processing/launch/config.yaml
    - us_controller/launch/config.yaml
3. Setup network paths in file ws/src/us_image_processing/src/image_processing_node_vanilla line 15,60
4. Launch ROS nodes in the following sequence:

(in separate terminals)

    rviz
    roslaunch iiwa_bringup iiwa_bringup.launch
    roslaunch us_image_processing  image_processing.launch
    roslaunch us_controller us_controller.launch

Call the service according to your initialization method:
Go to the pre-defined initial pose:
    
    rosservice call /start "data: true"

Initial pose is already reached (eg. init. by hand guide mode): 

    rosservice call /start "data: false"

Run the estimator: 

    rosrun us_image_processing estimator_node

### Simulation
(in separate terminals)
    
    roscore
    rviz
    roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch

Wait until gazebo is ready.
In Matlab: Launch /matlab/simGeo_pc2_vis/us_image_ros.m

    roslaunch us_image_processing  image_processing.launch
    roslaunch us_controller us_controller.launch

Call the service according to your initialization method:
Go to the pre-defined initial pose:
    
    rosservice call /start "data: true"

Initial pose is already reached (eg. init. by black magic): 

    rosservice call /start "data: false"

Run the estimator: 

    rosrun us_image_processing estimator_node
