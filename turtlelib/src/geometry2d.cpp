#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Point2D & p)
    {
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        const auto c = is.peek();
        if ( c=='[')
        {
            is.get();
            is >> p.x;
            is >> p.y;
            is.get();    
        }
        else{
        is >> p.x >> p.y;      
        }
        is.ignore(50, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        const auto c = is.peek();
        if ( c=='[')
        {
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();    
        }
        else{
        is >> v.x >> v.y;      
        }
        is.ignore(50, '\n');
        return is;
    }


    
double normalize_angle(double rad) {
    
    double normalizedAngle = rad - (ceil((rad + PI)/(2*PI))-1)*2*PI;
    return normalizedAngle;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
    Vector2D result;
    result.x = head.x - tail.x;
    result.y = head.y - tail.y;
    return result;
}

Vector2D normalize_vector(Vector2D v)
{   auto v_x2 = pow(v.x,2);
    auto v_y2 = pow(v.y,2);
    const auto mag= sqrt((v_x2+ v_y2));
    return {v.x / mag, v.y / mag};
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
    Point2D result;
    result.x = tail.x + disp.x;
    result.y = tail.y + disp.y;
    return result;
}












































}