#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>

int main() { 
    Eigen::Matrix3Xi F;
    int a[12];
    for(int i = 0; i < 10; ++ i) a[i] = i;
    F << 1, 2, 3;
    F << 3, 4, 6;

    std::cout << F << std::endl;

    return 0;
}
