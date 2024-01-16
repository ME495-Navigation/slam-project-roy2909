from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_rviz",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Controls whether rviz2 is launched."),
        DeclareLaunchArgument(name="use_jsp",
                              default_value="true",
                              choices=['true', 'false'],
                              description="Controls whether the joint_state_publisher is used to publish default joint states."),
        DeclareLaunchArgument(name="color",
                              default_value="purple",
                              choices=['red', 'green', 'blue', 'purple', ''],
                              description="Determines color of turtlebot3 to be passed to xacro file."),

        SetLaunchConfiguration(name="rviz_config",
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]
                               ),

        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             namespace=LaunchConfiguration("color"),
             condition=IfCondition(EqualsSubstitution(
                 LaunchConfiguration('use_jsp'), "true"))
             ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("color"),
            parameters=[
                {
                    "frame_prefix":
                    PathJoinSubstitution([(LaunchConfiguration('color')), '']),
                    "robot_description":
                    Command([ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution(
                            [FindPackageShare("nuturtle_description"),
                             "urdf/turtlebot3_burger.urdf.xacro"]),
                        " color:=",
                        LaunchConfiguration('color')
                    ])},
            ]),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d",
                       PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                             LaunchConfiguration("rviz_config")])],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('use_rviz'), "true")),
            on_exit=Shutdown()
        )

    ])
