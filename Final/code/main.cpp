#include <math.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "mesh_io.h"
#include "binary_io.h"
#include "Pivoter.h"

int used[4000];
std::vector<Eigen::Vector3i> tris;


void calculate_normals(const Eigen::MatrixXd &V, 
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

void move_positives(Eigen::MatrixXd &V) { 
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
    

    if (argc == 2) {
        const char * filename = argv[1];
        Eigen::Matrix3Xd VV;
        Eigen::Matrix3Xi F;
        common::read_obj(filename, VV, F);
        std::cout << "Read " << filename << " successfully :" << std::endl; 
        std::cout << "V: " << VV.rows() << " " << VV.cols() << std::endl;
        std::cout << "F: " << F.rows() << " " << F.cols() << std::endl;

        // calculate normals;
        calculate_normals(VV, F, N);
        V = VV;
        common::save_obj("./bunny.obj", V, F);
    } else {
        common::read_matrix_binary_from_file("../data/V", V);
        common::read_matrix_binary_from_file("../data/N", N);
    }

    move_positives(V);


    Pivoter pvt;
    pvt.bucketsort(V);

    while(true) {
        if(!pvt.find_seed_triangle(tris, V, N, 233)) break;

        while(pvt.front.size() > 0) { 
            pvt.find_next_triangle(tris, V, N);
        }
        break;
    }
    Eigen::Matrix3Xi F;
    F.resize(3, tris.size());
    int i = 0;
    std::cout << tris.size() << std::endl;
    for(auto tri:tris) {
        std::cout << tri[0] << " " << tri[1] << " " << tri[2] << std::endl;
        F.col(i++) = tri;
    }

    common::save_obj("result.obj", V, F);
    
    return 0;
}
