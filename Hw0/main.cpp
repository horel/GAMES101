#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include<iostream>
#include <math.h>

int main(){
    // TO DO: Define point P
    // TO DO: Define rotation matrix M
    // TO DO: M * P

    const float PI = 3.1415926f;
    const float DEG2RED = PI / 180.0f;

    std::cout << "Hw0：P_prime" << std::endl;
    float rad = 45.0f * DEG2RED;

    Eigen::Vector3f P(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f M;
    M << cos(rad), -sin(rad), 1,
         sin(rad), cos(rad),  2,
         0,        0,         1;

    Eigen::Vector3f P_prime = M * P;
    std::cout << P_prime << std::endl;

    return 0;
}
