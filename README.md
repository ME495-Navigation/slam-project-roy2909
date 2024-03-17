# ME495 Sensing, Navigation and Machine Learning For Robotics
* Rahul Roy
* Winter 2024

## Building the project
To build the project, doo the follwing:
`CLone the repository`
use `vcs import < turtle.repos` to import nuturtlebotmsgs
use `colcon build` to build the project

# Package List
This repository consists of several ROS2/C++ packages
- nuturtle_description - The package contains urdf files, testing, and visualization code to display colored turtlebot3's in rviz2.
- turtlelib - Library to handle 2D rigid body transformations and other functionality.
- nusim - Simulation and visualization package to display turtlebot3 in Rviz2
- nuturtle_control - The package contains nodes resposible for the control of the turtlebot in simulation and on the real one.
- nuslam - The package contains nodes responsible for the SLAM of the turtlebot in simulation and on the real one.


Here is an image of the robot using the nuslam package to perform SLAM.

![](images/slam.png)


 Here is the robot using the nuslam package to perform SLAM. 

 [Screencast from 03-01-2024 01:32:32 PM.webm](https://github.com/ME495-Navigation/slam-project-roy2909/assets/144197977/591c6368-7890-4466-b56e-6498aa5c9ff8)