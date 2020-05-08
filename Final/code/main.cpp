#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "mesh_io.h"
#include "binary_io.h"

const double ro = 0.01;

void calculateNormals(const Eigen::MatrixXd &V, 
                      const Eigen::Matrix3Xi &F, 
                      Eigen::MatrixXd &N) {
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

}

void movePositives(Eigen::MatrixXd &V) { 
    double mpt[3];
    mpt[0] = mpt[1] = mpt[2] = 1e15;
    for(int i = 0; i < V.cols(); ++ i) {
        auto pt = V.col(i);
        for(int j = 0; j < 3; ++ j)
            mpt[j] = std::min(mpt[j], pt[j]);
    }

    for(int i = 0; i < V.cols(); ++ i) {
        auto pt = V.col(i);
        for(int j = 0; j < 3; ++ j)
            pt[j] -= mpt[j];
    }
    return;
}

int main(int argc, const char** argv) { 

    Eigen::MatrixXd V, N;
    Eigen::Matrix3Xi F;

    if (argc == 2) {
        const char * filename = argv[1];
        Eigen::Matrix3Xd VV;
        common::read_obj(filename, VV, F);
        std::cout << "Read " << filename << " successfully :" << std::endl; 
        std::cout << "V: " << VV.rows() << " " << VV.cols() << std::endl;
        std::cout << "F: " << F.rows() << " " << F.cols() << std::endl;

        // calculate normals;
        calculateNormals(VV, F, N);
        V = VV;
    } else {
        common::read_matrix_binary_from_file("../data/V", V);
        common::read_matrix_binary_from_file("../data/N", N);
    }

    movePositives(V);

    // common::save_obj("./bunny.obj", V, F);
    
    return 0;
}
