#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Differential Drive kinematics 
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{   
    /// \brief the wheel radius of the turtlebot3 burger
    constexpr double tb3_wheel_radius = 0.033;
    /// \brief the wheel track of the turtlebot3 burger
    constexpr double tb3_track_width = 0.16;

    /// \brief Wheel Position
    struct WheelPos
    {   
        /// \brief left wheel position in radians 
        double left = 0.0;
        /// \brief right wheel position in radians 
        double right = 0.0;
    };

    /// \brief Configuration of robot
    struct RobotConfig
    {   
        /// \brief x coordinate of robot
        double x = 0.0;
        /// \brief y coordinate of robot
        double y = 0.0;
        /// \brief orientation of robot
        double theta = 0.0;
    };

    /// \brief Dirrerential Drive Kinematics
    class DiffDrive
    {
    public:
        DiffDrive();

        /// \brief set wheel radius and track width of robot
        /// \param wheel_radius - radius of wheels
        /// \param track - distance between wheels
        DiffDrive(double wheel_radius, double track);

        /// \brief set wheel radius,track , position of wheels and configuration of robot
        /// \param wheel_radius - radius of wheels
        /// \param track - distance between wheels
        /// \param pos - Angular postion of wheels in radians
        /// \param config - configuration of robot
        DiffDrive(double wheel_radius, double track, WheelPos pos, RobotConfig config);

        /// \brief get wheel position
        /// \returns wheel positions
        WheelPos get_pos() const;

        /// \brief set new wheel positions
        /// \param new_pos - new wheel position
        void set_pos(WheelPos new_pos);

        /// \brief Calculate forward kinematics from new wheel position
        /// \param wheel_pos_new - new wheel positions
        void ForwardKinematics(WheelPos wheel_pos_new);

        /// \brief gives body twist from wheel position
        /// \param wheel_pos_new - new wheel positions
        /// \return body twist
        Twist2D BodyTwist(WheelPos wheel_pos_new);

        /// \brief get robot configuration
        /// \return robot configuration
        RobotConfig get_config() const;

        /// \brief set robot configuration
        /// \param new_config new configuration
        void set_config(RobotConfig new_config);

        /// \brief Calculate inverse kinematics from  body twist
        /// \param Tb - the body twist 
        /// \return wheel Positions to achieve twist
        WheelPos InverseKinematics(Twist2D Tb);

    private:
        /// \brief radius of the wheels
        double wheel_radius_;
        /// \brief distance between the wheels
        double wheel_track_;
        /// \brief Position of the wheels
        WheelPos wheel_position_;
        /// \brief configuration of the robot
        RobotConfig q;
    };

}
#endif  