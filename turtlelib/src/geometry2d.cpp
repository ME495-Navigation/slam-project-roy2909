#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Point2D &p)
    {
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream &operator>>(std::istream &is, Point2D &p)
    {
        const auto c = is.peek();
        if (c == '[')
        {
            is.get();
            is >> p.x;
            is >> p.y;
            is.get();
        }
        else
        {
            is >> p.x >> p.y;
        }
        is.ignore(50, '\n');
        return is;
    }

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        const auto c = is.peek();
        if (c == '[')
        {
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();
        }
        else
        {
            is >> v.x >> v.y;
        }
        is.ignore(50, '\n');
        return is;
    }

    double normalize_angle(double rad)
    {
// ############################## Begin_Citation [2] ##############################
        double normalizedAngle = rad - (ceil((rad + PI) / (2.0 * PI)) - 1.0) * 2.0 * PI;
        return normalizedAngle;
// ############################## Begin_Citation [2] ##############################
    }

    Vector2D operator-(const Point2D &head, const Point2D &tail)
    {
        Vector2D result;
        return {head.x - tail.x, head.y - tail.y};
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & rhs) {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs)
    {
        Vector2D result ;
        result.x=lhs.x-rhs.x;
        result.y=lhs.y-rhs.y;
        return result;
    }
    Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs)
    {
        Vector2D result ;
        return {lhs.x+rhs.x, lhs.y+rhs.y};
    }
    Vector2D operator*(const double & lhs, Vector2D rhs)
    {
        return rhs *= lhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs)
    {
            return lhs*=rhs;
    }

    double dot(Vector2D v1, Vector2D v2)
    {
        double result = v1.x*v2.x +v1.y *v2.y;
        return result;
    }

    
    double magnitude(Vector2D v)
    {
        double mag=sqrt(pow(v.x,2)+pow(v.y,2));
        return mag;
    }

    
    double angle(Vector2D v1, Vector2D v2)
    {
        double dotp = dot(v1,v2);
        double mag_v1=magnitude(v1);
        double mag_v2=magnitude(v2);
        double ang = acos(dotp/(mag_v1*mag_v2));
        return ang;
    }


    Vector2D normalize_vector(Vector2D v)
    {
        auto v_x2 = pow(v.x, 2);
        auto v_y2 = pow(v.y, 2);
        const auto mag = sqrt((v_x2 + v_y2));
        return {v.x / mag, v.y / mag};
    }

    Point2D operator+(const Point2D &tail, const Vector2D &disp)
    {
        Point2D result;
        result.x = tail.x + disp.x;
        result.y = tail.y + disp.y;
        return result;
    }

}