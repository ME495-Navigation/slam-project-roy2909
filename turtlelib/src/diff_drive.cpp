#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius_(tb3_wheel_radius), wheel_track_(tb3_track_width), wheel_position_{0.0, 0.0}, q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius, double track) : wheel_radius_(wheel_radius), wheel_track_(track), wheel_position_{0.0, 0.0}, q{0.0, 0.0, 0.0} {}
    DiffDrive::DiffDrive(double wheel_radius, double track, WheelPos pos, RobotConfig config) : wheel_radius_(wheel_radius), wheel_track_(track), wheel_position_{pos}, q{config} {}
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
    void DiffDrive::ForwardKinematics(WheelPos del_wheels)
    {
        
        // [1] Get the body twist Vb

        Twist2D Vb;
        Vb.omega = ((wheel_radius_ / (2.0 * (wheel_track_/2.0))) * (del_wheels.right - del_wheels.left));
        Vb.x = (wheel_radius_ / 2.0) * (del_wheels.left + del_wheels.right);
        Vb.y = 0.0;

       
        // [2] Find the body transformation from the twist
       
        Transform2D T_bb_p = integrate_twist(Vb);

        // Extract the transformation values from
        // b to b_p

        double d_qb_p_theta = T_bb_p.rotation();
        double d_qb_x = T_bb_p.translation().x;
        double d_qb_y = T_bb_p.translation().y;


        // [3] Transform dqb in {body frame} to dq in {space frame}

        double dqtheta = d_qb_p_theta;
        double dqx = std::cos(q.theta) * d_qb_x - std::sin(q.theta) * d_qb_y;
        double dqy = std::sin(q.theta) * d_qb_x + std::cos(q.theta) * d_qb_y;

        // [4] Update robot config

        q.theta += dqtheta;
        q.theta = normalize_angle(q.theta);
        q.x += dqx;
        q.y += dqy;

        wheel_position_.left += del_wheels.left;
        wheel_position_.right += del_wheels.right;
    }
    

    Twist2D DiffDrive::BodyTwist(WheelPos del_wheels)
    {
        Twist2D Vb;
        Vb.omega = ((wheel_radius_ / (2.0 * (wheel_track_/2.0))) * (del_wheels.right - del_wheels.left));
        Vb.x = (wheel_radius_ / 2.0) * (del_wheels.left + del_wheels.right);
        return Vb;
    }

    turtlelib::WheelPos DiffDrive::InverseKinematics(Twist2D Tb)
    {
        if (Tb.y != 0.0)
        {
            throw std::logic_error("Twist causes slipping");
        }
        //equation 1 and 2 from kinematics pdf
        return {((-(wheel_track_ / 2) * Tb.omega + Tb.x) / wheel_radius_), (((wheel_track_ / 2) * Tb.omega + Tb.x) / wheel_radius_)};
    }
}