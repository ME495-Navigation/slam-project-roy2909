#include <armadillo>
#include <cmath>
#include <vector>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/circle_fit.hpp"

namespace turtlelib
{
    Circle_chords circle_fit(const std::vector<Vector2D> &points)
    {
        // Compute the centroid of the points
        auto x_hat = 0.0;
        auto y_hat = 0.0;
        auto num=0.0;
        for (const auto &p : points)
        {
            x_hat += p.x;
            y_hat += p.y;
            num+=1;
        }
        //mean
        x_hat /= num;
        y_hat /= num;

        //shift coordinates so that the centroid is at the origin
        std::vector<Vector2D> shifted_points;
        for (const auto &p : points)
        {
            shifted_points.push_back(p - Vector2D{x_hat, y_hat});
        }

        //compute z_i= x_i^2 + y_i^2
        std::vector<double> z_i;
        for (const auto &p : shifted_points)
        {
            z_i.push_back(p.x * p.x + p.y * p.y);
        }
        //compute mean of z
        auto z_hat = 0.0;
        double n = points.size();
        for (const auto &z : z_i)
        {
            z_hat += z;
        }
        z_hat /= n;
        // form data matrix from n data points
        arma::mat Z{n, 4};
        for (size_t i = 0; i < n; i++)
        {
            Z(i, 0) = z_i.at(i);
            Z(i, 1) = shifted_points[i].x;
            Z(i, 2) = shifted_points[i].y;
            Z(i, 3) = 1;
        }
        //form the momemt matrix , M=1/n*Z^T*Z
        arma::mat M = (1 / n) * Z.t() * Z;

        //form the constarint matrix, H
        arma::mat H =arma::mat(4, 4, arma::fill::zeros);
        H(0, 0) = 8.0*z_hat;
        H(3,0) = 2.0;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;

        //compute H inverse
        arma::mat H_inv = arma::mat(4, 4, arma::fill::zeros);
        H_inv(3,0) = 0.5;
        H_inv(0,3) = 0.5;
        H_inv(1,1) = 1.0;
        H_inv(2,2) = 1.0;
        H_inv(4,4) = -2.0*z_hat;

        //compute singular value decomposition of Z
        arma::mat U;
        arma::vec s;    
        arma::mat V;
        arma::svd(U, s, V, Z);

        //compute A matrix
        arma:: vec A;
        //first condition  f the smallest singular value σ4 is less than 10−12, then Let A be the the 4th column of the V matrix
        if(s(3) < 10e-12)
        {
            A = V.col(3);
        }
        else
        {
            //second condition
            arma::mat Y=V*arma::diagmat(s)*V.t();
            arma::mat Q = Y*H_inv*Y;
            arma::vec eigen_values;
            arma::mat eigen_vectors;
            arma::eig_sym(eigen_values, eigen_vectors, Q);
            auto min_Index = arma::index_min(arma::real(eigen_values.elem(arma::find(arma::real(eigen_values) > 0))));
            arma::vec A_star = eigen_vectors.col(min_Index);
            A = V*A_star;


        }
        //compute the circle equation
        double a=-A(1)/(2.0*A(0));
        double b=-A(2)/(2.0*A(0));
        double r=std::sqrt(A(1)*A(1)+A(2)*A(2)-4*A(0)*A(3))/(2*std::abs(A(0)));
        //shift the circle back to the original coordinates
        double circle_x = a + x_hat;
        double circle_y = b + y_hat;
        return{circle_x, circle_y, r};

    }
}