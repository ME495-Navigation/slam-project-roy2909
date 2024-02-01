#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    constexpr double tb3_wheel_radius=0.033;
   
    constexpr double tb3_track_width=0.16;

    struct WheelPos
    {
        double left = 0.0;
        double right = 0.0;
    };

    struct RobotConfig
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };

    class DiffDrive
    {
    public:
        DiffDrive();

        DiffDrive(double wheel_radius, double track);
        DiffDrive(double wheel_radius, double track, WheelPos pos, RobotConfig config);
        WheelPos get_pos() const;
        void set_pos(WheelPos new_pos);
        void ForwardKinematics(WheelPos wheel_pos_new);
        RobotConfig get_config() const;
        void set_config(RobotConfig new_config);
        WheelPos InverseKinematics(Twist2D Tb);

    private:
        double wheel_radius_;
        double wheel_track_;
        WheelPos wheel_position_;
        RobotConfig q;
    };

}
#endif