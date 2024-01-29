#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius_(tb3_wheel_radius), wheel_position_{0.0, 0.0}, wheel_track_(tb3_track_width), q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius, double track) : wheel_radius_(wheel_radius), wheel_position_{0.0, 0.0}, wheel_track_(track), q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius, WheelPos pos, double track, RobotConfig config) : wheel_radius_(wheel_radius), wheel_position_{pos}, wheel_track_(track), q{config} {}
    turtlelib::WheelPos DiffDrive::get_pos() const
    {
        return wheel_position_;
    }

    void DiffDrive::set_pos(WheelPos new_pos)
    {
        wheel_position_.left = new_pos.left;
        wheel_position_.right = new_pos.right;
    }
    turtlelib::RobotConfig DiffDrive::get_config() const
    {
        return q;
    }
    void DiffDrive::set_config(RobotConfig new_config)
    {
        q.x = new_config.x;
        q.y = new_config.y;
        q.theta = new_config.theta;
    }
    turtlelib::Twist2D DiffDrive::ForwardKinematics(WheelPos wheel_pos_new){
        const auto rpos_del =wheel_pos_new.right;
        const auto lpos_del=wheel_pos_new.left;
        const auto  twist_theta = wheel_radius_*0.5*(rpos_del-lpos_del)/wheel_track_;
        const auto  twist_x = wheel_radius_*0.5*(-rpos_del+lpos_del);
        Transform2D Tbb_prime = turtlelib::integrate_twist({twist_theta,twist_x,0.0});
        const auto b_theta=Tbb_prime.rotation();
        Vector2D b = Tbb_prime.translation();
        // const auto dq_x = 

    }
    turtlelib::WheelPos DiffDrive::InverseKinematics(Twist2D Tb)
    {
        if (Tb.y != 0.0)
        {
            throw std::logic_error("Twist cannot be accomplished unless the wheels slips");
        }

        return {(((wheel_track_/2) * Tb.omega + Tb.x) / wheel_radius_), ((-(wheel_track_/2) * Tb.omega + Tb.x) / wheel_radius_)};
    }
}