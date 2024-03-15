# Nuturtle_Control Description
The package provides operations responsible to control the turtlebot in real world and simulation. It includes three nodes : Circle ,Turtle_control, Odometry.
Circle is responsible to make the robot move in a circle at a set velocity and radius. Turtle_control is responsible of converting twsit messages to wheel commands. The odometry node is responsible for providing odometry calculations.
* `ros2 launch nuturtle_control start_robot.launch.xml` 

# Launch File Details
* `ros2 launch nuturtle_control start_robot.launch.xml --show-args`
   Arguments (pass arguments as '<name>:=<value>'):

    `cmd_src`:
        Source of cmd_vel commands, decides which node publishes command vel to the turtlebot - circle, teleop, none
        (default: 'circle')

    `robot`:
        Launch in simulation or real robot - nusim, localhost, none
        (default: 'nusim')

    `use_rviz2`:
        Determines whether to use rviz or not - true or false
        (default: 'true')

# Test on physical robot

https://github.com/ME495-Navigation/slam-project-roy2909/assets/144197977/98a93165-4894-47d3-a368-035152a0378b