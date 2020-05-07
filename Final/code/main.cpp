#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "mesh_io.h"
#include "binary_io.h"

const int vN = 34817;
const int vF = 69630;


void calculateNormals(const Eigen::MatrixXd &V, const Eigen::Matrix3Xi &F) {
    Eigen::MatrixXd N;
    N.resize(V.rows(), V.cols()); 

    for (int i = 0; i < F.cols(); ++ i) {
 
        auto vid = F.col(i);
        for (int i = 0; i < 3; ++ i) {
            // edge b ccw to a
            Eigen::Vector3d a = V.col(vid((i+1)%3)) - V.col(vid(i));
            Eigen::Vector3d b = V.col(vid((i+2)%3)) - V.col(vid(i));

            auto fn = a.cross(b);
            fn.normalize();
            double angle = acos(a.dot(b) / a.norm() / b.norm());
            N.col(vid(i)) += fn * angle;
        }
    }

    // don't forget to normalize
    for (int i = 0; i < V.cols(); ++ i) {
        N.col(i).normalize();
    }

    // check point 1603
    std::cout << "v 1603 = " << N.col(1603) << std::endl;
    common::write_matrix_binary_to_file("../data/V", V);
    common::write_matrix_binary_to_file("../data/N", N);
}

int main() { 
    const char * filename = "../../models/bunny.obj";
    Eigen::Matrix3Xd V;
	Eigen::Matrix3Xi F;
    common::read_obj(filename, V, F);

    std::cout << V.rows() << " " << V.cols() << std::endl;
    // std::cout << F.rows() << " " << F.cols() << std::endl;

    calculateNormals(V, F);
    
    return 0;
}
