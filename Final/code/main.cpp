#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include "mesh_io.h"


int main() { 
    const char * filename = "../../models/bunny.obj";
    Eigen::Matrix3Xd V;
	Eigen::Matrix3Xi F;
    common::read_obj(filename, V, F);

    std::cout << V.rows() << " " << V.cols() << std::endl;
    std::cout << F.rows() << " " << F.cols() << std::endl;

    
    return 0;
}
