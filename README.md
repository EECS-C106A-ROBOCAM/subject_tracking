# Active Robotic Arm Tracking of a Moving Subject

## Instructions

### Setup
Clone this repository into your `ros_workspaces`, and `cd` into the root directory of the cloned repository.

KDL install:
`sudo apt-get install ros-kinetic-orocos-kdl` and `sudo apt-get install ros-kinetic-kdl-parser-py`.

Quaternion install:
`python -m pip install numpy-quaternion`

Please run `catkin_make` and `source devel/setup.bash` as necessary, and install any other library dependencies (e.g. scipy) with pip or conda if prompted.

### ROBOCAM Execution
To setup all the necessary services and nodes required for the sensing, planning, and actuation of ROBOCAM, please begin by running `roslaunch rviz_animator robocam.launch`. This should also open up an RViz interface, pre-configured with the RobotModel and the current frame of the nonstationary/stationary subject being tracked. Within this RViz environment is enabled ROBOCAM's custom Keyframe planning and publishing plugin, from which you as the operator can set Keyframes - goal poses and the timestamps to reach those poses at. To do so, you can simply adjusts your current view to a desired pose, and presses the "K" key on your keyboard to place a Keyframe at that position and orientation. Once the Keyframe has been created, you may adjust the designated timestamp of the Keyframe, its name, or delete it all together if you so choose. After repeating this process until you are satisfied with your created sequence, you may check the `Publish?` box in the plugin's viewing pane to send that target to the robot, which then actuates with respect to the tracked subject while maintaining fidelity to your operator-defined Keyframes. Even if you do not have a physical ROBOCAM available, RViz will visualize the execution in real-time simulation.

