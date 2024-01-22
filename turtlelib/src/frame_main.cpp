#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

int main()
{
    turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
    turtlelib::Svg svg;
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;
    T_ba = T_ab.inv();
    T_cb = T_bc.inv();
    T_ac = T_ab * T_bc;
    T_ca = T_ac.inv();
    //a frame
    turtlelib::Point2D origin{0.0,0.0};
    turtlelib::Vector2D X{1.0,0};
    svg.drawCoordinateFrame(origin,X,"a");
    //b frame
    turtlelib::Point2D origin_b=T_ba(origin);
    turtlelib::Vector2D X_b = T_ba(X);
    svg.drawCoordinateFrame(origin_b,X_b,"b");
    //c frame
    turtlelib::Point2D origin_c=T_ca(origin);
    turtlelib::Vector2D X_c = T_ca(X);
    svg.drawCoordinateFrame(origin_c,X_c,"c");
    turtlelib::Point2D p_a, p_b, p_c;
    
    std::cout << "T_{a, b}: " << T_ab << "\n";
    std::cout << "T_{b, a}: " << T_ba << "\n";
    std::cout << "T_{b, c}: " << T_bc << "\n";
    std::cout << "T_{c, b}: " << T_cb << "\n";
    std::cout << "T_{a, c}: " << T_ac << "\n";
    std::cout << "T_{c, a}: " << T_ca << std::endl;


    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;
    p_b = T_ba(p_a);
    p_c = T_ca(p_a);
    std::cout << "p_a: " << p_a << "\n";
    std::cout << "p_b: " << p_b << "\n";
    std::cout << "p_c: " << p_c << "\n";
    turtlelib::PointParams P, Q, W;
    P.x = p_a.x;
    P.y = p_a.y;
    P.strokeColor = "purple";
    P.fillColor = "purple";
    Q.x = p_b.x;
    Q.y = p_b.y;
    Q.strokeColor = "brown";
    Q.fillColor = "brown";
    W.x = p_c.x;
    W.y = p_c.y;
    W.strokeColor = "orange";
    W.fillColor = "orange";
    svg.drawPoint(P);
    svg.drawPoint(Q);
    svg.drawPoint(W);

    turtlelib::Vector2D v_a, v_b, v_c, v_bhat;
    turtlelib::VectorParams A, B, C;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    v_a = T_ab(v_b);
    v_c = T_cb(v_b);
    v_bhat = normalize_vector(v_b);
    std::cout << "v_bhat " << v_bhat << "\n";
    std::cout << "v_a " << v_a << "\n";
    std::cout << "v_b " << v_b << "\n";
    std::cout << "v_c " << v_c << std::endl;
    A.x1 = v_bhat.x;
    A.y1 = v_bhat.y;
    A.x2 = 0.0;
    A.y2 = 0.0;
    A.strokeColor = "brown";
    B.x1 = v_a.x;
    B.y1 = v_a.y;
    B.x2 = 0.0;
    B.y2 = 0.0;
    B.strokeColor = "purple";
    C.x1 = v_c.x;
    C.y1 = v_c.y;
    C.x2 = 0.0;
    C.y2 = 0.0;
    C.strokeColor = "orange";
    svg.drawVector(A);
    svg.drawVector(B);
    svg.drawVector(C);

    svg.writeToFile("frames.svg");
    turtlelib::Twist2D V_a, V_b, V_c;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;
    V_a = T_ab(V_b);
    V_c = T_cb(V_b);
    std::cout << "V_a " << V_a << "\n";
    std::cout << "V_b " << V_b << "\n";
    std::cout << "V_c " << V_c << std::endl;

    return 0;
}