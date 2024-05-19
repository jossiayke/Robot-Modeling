Instructions to run programs in this CAD Modeling & Gazebo Simulation Project

Note: Code was run on Ubuntu 18.04 LTS, melodic, so Python2.7 was used to run the python scripts in this package. So, update the shebang lines, and make other adjustments accordingly.

1. First place all of the files in the package folder inside a workspace/src/ of your choosing
2. Run catkin clean && catkin build inside your workspace, and source bashrc with workspace
3. Run the following command to launch the robot in the competition arena world
	> roslaunch robotBookShelf template_launch.launch
4. To perform teleop, 
	> run the command below in the terminal
		> rosrun robotBookShelf teleop_template.py
		> run the sub.py file as "python3 sub.py" 
			to see results being printed and have theta dot getting published to joints 
	> Use the directions given to control the robot except,
		> use 'u' to turn right
		> use 'o' to turn left

				>> The End << 

