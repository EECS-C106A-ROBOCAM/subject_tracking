# Active Robotic Arm Tracking of a Moving Subject

## Instructions

### Setup
Clone this repository into your `ros_workspaces`, and `cd` into the root directory of the cloned repository.

### Running Simulation
To load Sawyer into Gazebo, run `/intera.sh sim` followed by `roslaunch sawyer_gazebo sawyer_world.launch`. 
To load the moving subject (and the walled surface upon which it randomly moves), run `rosrun planning simulate_moving_subject.py` in a seperate terminal, and you should be good to go! 
Make sure to `source devel/setup.bash` whenever necessary.
