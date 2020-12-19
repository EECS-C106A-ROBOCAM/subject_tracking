# Active Robotic Arm Tracking of a Moving Subject

## Instructions

### Setup
Clone this repository into your `ros_workspaces`, and `cd` into the root directory of the cloned repository.

KDL install:
`sudo apt-get install ros-kinetic-orocos-kdl` and `sudo apt-get install ros-kinetic-kdl-parser-py`.

Quaternion install:
`python -m pip install numpy-quaternion`

### Running Simulation
To load Baxter into Gazebo, run `/baxter.sh sim` followed by `roslaunch baxter_gazebo baxter_world.launch right_electric_gripper:=false left_electric_gripper:=false`. 

To load the moving subject (and the walled surface upon which it randomly moves) into Gazebo, run `rosrun object_movement simulate_movement.py` in a seperate terminal.

To initialize the MoveIt arm path planner and manipulator arm controller, run `rosrun baxter_tools enable_robot.py -e` followed by `rosrun baxter_interface joint_trajectory_action_server.py`. Then, in a seperate terminal, run  `roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=false left_electric_gripper:=false`. Finally, in yet another terminal, run `rosrun manipulator_sim arm_controller.py` to initialize the arm to its starting position and begin listening for relative poses to move along from the camera (left arm camera image_raw feed can be seen live in rviz).

Make sure to `source devel/setup.bash` whenever necessary.
