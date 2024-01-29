#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    std::istream &operator>>(std::istream &is, Twist2D &tw)
    {
        const auto c = is.peek();
        if (c == '[')
        {
            is.get();
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get();
        }
        else
        {
            is >> tw.omega >> tw.x >> tw.y;
        }
        is.ignore(50, '\n');
        return is;
    }

    Transform2D::Transform2D() : trans{0.0, 0.0}, angle(0.0) {}
    Transform2D::Transform2D(Vector2D trans) : trans(trans), angle(0.0) {}
    Transform2D::Transform2D(double radians) : trans{0.0, 0.0}, angle(radians) {}
    Transform2D::Transform2D(Vector2D trans, double radians) : trans(trans), angle(radians) {}

    Point2D Transform2D::operator()(Point2D p) const
    {
        return {
            cos(angle) * p.x - sin(angle) * p.y + trans.x,
            sin(angle) * p.x + cos(angle) * p.y + trans.y};
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        return {
            cos(angle) * v.x - sin(angle) * v.y + trans.x,
            sin(angle) * v.x + cos(angle) * v.y + trans.y};
    }
    Twist2D Transform2D::operator()(Twist2D v) const
    {

        return {
            v.omega, v.omega * trans.y + v.x * cos(angle) - v.y * sin(angle), -v.omega * trans.x + v.x * sin(angle) + v.y * cos(angle)

        };
    }

    Transform2D Transform2D::inv() const
    {
        Vector2D inv_xy = {-trans.x * cos(angle) - trans.y * sin(angle), -trans.y * cos(angle) + trans.x * sin(angle)};
        double inv_rot = -angle;
        return Transform2D{inv_xy, inv_rot};
    }

    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        double currentX = trans.x;
        double currentY = trans.y;
        double currentAngle = angle;

        trans.x = cos(currentAngle) * rhs.trans.x - sin(currentAngle) * rhs.trans.y + currentX;
        trans.y = sin(currentAngle) * rhs.trans.x + cos(currentAngle) * rhs.trans.y + currentY;

        angle = currentAngle + rhs.angle;
        return *this;
    }
    Vector2D Transform2D::translation() const
    {
        return trans;
    }

    double Transform2D::rotation() const
    {
        return angle;
    }

    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        return os << "deg:"
                  << " " << rad2deg(tf.angle) << " "
                  << "x:"
                  << " " << tf.trans.x << " "
                  << "y:"
                  << " " << tf.trans.y;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        double angle = 0.0;
        Vector2D trans{0.0, 0.0};
        std::string s1, s2, s3;
        const auto c = is.peek();
        if (c == 'd')
        {
            is >> s1;
            is >> angle;
            is >> s2;
            is >> trans.x;
            is >> s3;
            is >> trans.y;
        }
        else
        {
            is >> angle >> trans.x >> trans.y;
        }
        is.ignore(50, '\n');
        angle = deg2rad(angle);
        tf = Transform2D{{trans}, angle};
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        lhs *= rhs;
        return lhs;
    }
    Transform2D integrate_twist(Twist2D t)
    {
        Transform2D Tbs_prime;
        Transform2D Tsb_prime;
        Transform2D Tbs;
        Transform2D Tbb_prime;
        if (t.omega == 0)
        {
            double x = t.x;
            double y = t.y;
            Transform2D Tbb_prime(Vector2D{x, y});
            return Tbb_prime;
        }
        else
        {
            double x = t.y / t.omega;
            double y = -t.x / t.omega;
            Transform2D Tsb(Vector2D{x, y});
            Transform2D Tss_prime(t.omega);
            Tbs = Tsb.inv();
            Tbs_prime = Tbs;
            Tsb_prime = Tbs_prime.inv();
            Tbb_prime = Tbs * Tss_prime * Tsb_prime;
            return Tbb_prime;
        }
    }
}