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
    void DiffDrive::ForwardKinematics(WheelPos wheel_pos_new)
    {
        
        // [1] Get the body twist Vb

        Twist2D Vb;
        Vb.omega = ((wheel_radius_ / (2.0 * (wheel_track_/2.0))) * (wheel_pos_new.right - wheel_pos_new.left));
        Vb.x = (wheel_radius_ / 2.0) * (wheel_pos_new.left + wheel_pos_new.right);
        // Vb.y = 0.0;

        // Vb.omega = ((-delta_wheels.phi_l + delta_wheels.phi_r) / wheel_track) * wheel_radius;
        // Vb.x = ((delta_wheels.phi_l + delta_wheels.phi_r) / 2.0) * wheel_radius;


        // [2] Find the body transformation from the twist
        // This expresses the new chassis frame, b_p, 
        // relative to the initial frame, b.

        Transform2D T_bb_p = integrate_twist(Vb);

        // Extract the transformation values from
        // b to b_p

        double d_qb_p_theta = T_bb_p.rotation();
        double d_qb_x = T_bb_p.translation().x;
        double d_qb_y = T_bb_p.translation().y;


        // [3] Transform dqb in {b} to dq in {s}

        double dqtheta = d_qb_p_theta;
        double dqx = std::cos(q.theta) * d_qb_x - std::sin(q.theta) * d_qb_y;
        double dqy = std::sin(q.theta) * d_qb_x + std::cos(q.theta) * d_qb_y;

        // [4] Update robot config

        q.theta += dqtheta;
        q.theta = normalize_angle(q.theta);
        q.x += dqx;
        q.y += dqy;

        // Update wheel positions as well?
        wheel_position_.left += wheel_pos_new.left;
        wheel_position_.right += wheel_pos_new.right;
    }
    

    Twist2D DiffDrive::BodyTwist(WheelPos wheel_pos_new)
    {
        Twist2D t;
        t.omega = ((-wheel_pos_new.left + wheel_pos_new.right) / wheel_track_) * wheel_radius_;
        t.x = ((wheel_pos_new.left + wheel_pos_new.right) / 2) * wheel_radius_;
        t.y = 0.0;
        return{t.omega,t.x,t.y};
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