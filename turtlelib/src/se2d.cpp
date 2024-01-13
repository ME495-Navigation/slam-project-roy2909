#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        const auto c = is.peek();
        if ( c=='[')
        {
            is.get();
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get();    
        }
        else{
        is >> tw.omega >> tw.x >> tw.y;      
        }
        is.ignore(50, '\n');
        return is;
    }

    Transform2D::Transform2D():trans{0.0,0.0}, rot(0.0){}
    Transform2D::Transform2D(Vector2D trans):trans(trans),rot(0.0){}
    Transform2D::Transform2D(double radians):trans{0.0,0.0},rot(radians){}
    Transform2D::Transform2D(Vector2D trans, double radians):trans(trans),rot(radians){}

    Point2D Transform2D::operator()(Point2D p) const
    {
        return {
            cos(rot) * p.x - sin(rot) * p.y + trans.x,
            sin(rot) * p.x + cos(rot) * p.y + trans.y
        };
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        return {
            cos(rot) * v.x - sin(rot) * v.y + trans.x,
            sin(rot) * v.x + cos(rot) * v.y + trans.y
        };
    }
    Twist2D Transform2D::operator()(Twist2D v) const
    {

        return{
            v.omega, v.omega * trans.y + v.x * cos(rot) - v.y * sin(rot), -v.omega * trans.x + v.x * sin(rot) + v.y * cos(rot)

        };
    }

    Transform2D Transform2D::inv() const
    {

    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {

    }
    Vector2D Transform2D::translation() const
    {
        return trans;
    }

    double Transform2D::rotation() const
    {
        return rot;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        return os << "deg" << rad2deg(tf.rot) << " " << "x" << tf.trans.x << " " << " y " << tf.trans.y;
    }
    
    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {   double rot = 0.0;
        Vector2D trans{0.0, 0.0};
        std::string s1,s2,s3;
        const auto c = is.peek();
        if ( c=='d')
        {  
            is >> s1;
            is >> rot;
            is >> s2;
            is >> trans.x;
            is >> s3; 
            is >> trans.y;   
        }
        else{
        is >> rot >> trans.x >> trans.y;      
        }
        is.ignore(50, '\n');
        rot = deg2rad(rot);
        tf = Transform2D{trans,rot};
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs *= rhs;
    }
}