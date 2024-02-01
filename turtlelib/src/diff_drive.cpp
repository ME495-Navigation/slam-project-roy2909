#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius_(tb3_wheel_radius), wheel_track_(tb3_track_width), wheel_position_{0.0, 0.0}, q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius, double track) : wheel_radius_(wheel_radius), wheel_track_(track), wheel_position_{0.0, 0.0}, q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius,double track, WheelPos pos, RobotConfig config) : wheel_radius_(wheel_radius), wheel_track_(track), wheel_position_{pos}, q{config} {}
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
    void DiffDrive::ForwardKinematics(WheelPos wheel_pos_new){
        //Twist=H†(0)*deltheta(∆θL, ∆θR) 
        Twist2D t;
        t.omega = ((-wheel_pos_new.left +wheel_pos_new.right)/wheel_track_ )*wheel_radius_ ;
        t.x = ((wheel_pos_new.left + wheel_pos_new.right)/2) * wheel_radius_  ;
        t.y = 0.0;
        //New frame relative to initial frame
        Transform2D Tbb_prime = integrate_twist(t);

        // Extract change in coordinaes relative to body frame

        double dqb_theta = Tbb_prime.rotation();
        double dqb_x = Tbb_prime.translation().x;
        double dqb_y = Tbb_prime.translation().y;


        // Transform dqb in body frame to dq in fixed frame

        double dq_theta = dqb_theta;
        double dq_x = std::cos(q.theta) * dqb_x - std::sin(q.theta) * dqb_y;
        double dq_y = std::sin(q.theta) * dqb_x + std::cos(q.theta) * dqb_y;

        // Update robot config

        q.theta += dq_theta;
        q.theta = normalize_angle(q.theta);
        q.x += dq_x;
        q.y += dq_y;

        wheel_position_.left += wheel_pos_new.left;
        wheel_position_.right += wheel_pos_new.right;
    }
    turtlelib::WheelPos DiffDrive::InverseKinematics(Twist2D Tb)
    {
        if (Tb.y != 0.0)
        {
            throw std::logic_error("Twist causes slipping");
        }

        return {((-(wheel_track_/2) * Tb.omega + Tb.x) / wheel_radius_), (((wheel_track_/2) * Tb.omega + Tb.x) / wheel_radius_)};
    }
}