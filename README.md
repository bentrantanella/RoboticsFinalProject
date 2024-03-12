# Robotics Final Project
The goal of this project was to have two ROS turtlebots play tic-tac-toe against eacother.
Although it does not entirely work, the python game files are completely set up to work. No additional dependencies need to be
installed.
# Files Overview
The main driver file is [playGame.py](playGame.py), which contains the GoToPose() class and the Robot() class. GoToPose() is used for
navigation to a coordinate point on the map, and Robot() contains all of the information and methods needed for each robot.
The main launch file is [run_gamev2.launch](run_gamev2.launch), which calls all of the other launch files to set up the turtlebots, 
map, AMCL, move_base, and rviz. Lastly, [tic-tac-toe.yaml](tic-tac-toe.yaml) if the map file used, with [tic-tac-toe.pgm](tic-tac-toe.pgm)
being the map image.
# How to use the code (if it worked)
1. Start roscore
2. Run: ROS_NAMESPACE=robot_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="robot_1" set_lidar_frame_id:="robot_1/base_scan" on the first robot
3. Run: ROS_NAMESPACE=robot_2 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="robot_2" set_lidar_frame_id:="robot_2/base_scan" on the second robot
4. Run: rosrun tic-tac-toe run_gamev2.launch on host computer
5. Set up the launched rviz by:
    1. Changing project decription to: robot_1/description
    2. Changing tf prefix to: robot_1
    3. Change topics to namespaced robot_1 topics, specifically the local and global cost maps
    4. Add all fields for robot_2 and repeat previous steps
6. Run [playGame.py](playGame.py)
