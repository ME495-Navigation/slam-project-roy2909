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

        // Shift coordinates so that the centroid is at the origin and compute z_i = x_i^2 + y_i^2
        double n = static_cast<double>(points.size());
        arma::mat Z(points.size(), 4, arma::fill::zeros);
        arma::vec sum_z(points.size(), arma::fill::zeros);

        for (size_t i = 0; i < points.size(); i++)
        {
            Z(i, 1) = points[i].x - x_hat;
            Z(i, 2) = points[i].y - y_hat;
            Z(i, 0) = Z(i, 1) * Z(i, 1) + Z(i, 2) * Z(i, 2);
            Z(i, 3) = 1;

            sum_z(i) = Z(i, 0);
        }

        // Compute mean of z
            auto z_hat = arma::mean(sum_z);
        
        //form the momemt matrix , M=1/n*Z^T*Z
        arma::mat M = (1 / n) * Z.t() * Z;

        //form the constarint matrix, H
        arma::mat H =arma::mat(4, 4, arma::fill::zeros);
        H(0, 0) = 8.0*z_hat;
        H(3,0) = 2.0;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;
        
        //compute H inverse
        arma::mat H_inv = arma::mat(4, 4, arma::fill::zeros);
        H_inv(3,0) = 0.5;
        H_inv(0,3) = 0.5;
        H_inv(1,1) = 1.0;
        H_inv(2,2) = 1.0;
        H_inv(3,3) = -2.0*z_hat;

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
            auto min = 1e12;
            size_t min_index = 0;
            for (size_t i = 0; i < arma::real(eigen_values).size(); i++)
            {
                if (arma::real(eigen_values)(i) < min && arma::real(eigen_values)(i) > 0.0) 
                {
                    min = arma::real(eigen_values)(i);
                    min_index = i;
                }
            }
            
            arma::vec A_star = arma::real(eigen_vectors).col(min_index);
            A = Y.i()*A_star;


        }
        //compute the circle equation
        double a=-A.at(1)/(2.0*A.at(0));
        double b=-A.at(2)/(2.0*A.at(0));
        double r=std::sqrt(A.at(1)*A.at(1)+A.at(2)*A.at(2)-4*A.at(0)*A.at(3))/(2*std::abs(A.at(0)));
        //shift the circle back to the original coordinates
        double circle_x = a + x_hat;
        double circle_y = b + y_hat;
        return{circle_x, circle_y, r};

    }
}