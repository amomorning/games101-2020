#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>

int main() { 
    Eigen::Vector3f a = Eigen::Vector3f(2.0f, 2.0f, 0.0f);
    Eigen::Vector3f c = Eigen::Vector3f(2.0f, 0.0f, 0.0f);
    Eigen::Vector3f b = Eigen::Vector3f(3.0f, 2.0f, 1.0f);
    
    Eigen::Matrix3f m;
    m << a, b, c;
    std::cout << m << std::endl;
    
    return 0;
}
