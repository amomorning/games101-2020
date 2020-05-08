#include <math.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "mesh_io.h"
#include "binary_io.h"

const double ro = 0.005;
std::vector<int> bucket[20][20][20];


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

Eigen::Vector3i get_position(const Eigen::Vector3d pt) {
    double sigma = ro * 2; 
    int x = pt[0]/sigma + 1;
    int y = pt[1]/sigma + 1;
    int z = pt[2]/sigma + 1;
    return Eigen::Vector3i(x, y, z);
}

void bucketsort(const Eigen::MatrixXd &V) {
    // bunny with 15x15x15 with max grid 14 points (2, 0, 6);
    Eigen::Vector3i mpt;
    for (int i = 0; i < V.cols(); ++ i) {
        auto ps = get_position(V.col(i));
        bucket[ps[0]][ps[1]][ps[2]].push_back(i); 
    }
}

std::vector<int> pick_neighbors(const Eigen::Vector3d &pt) {
    auto ps = get_position(pt);
    std::vector<int> ret;
    for(int i = -1; i < 1; ++ i) {
        for(int j = -1; j < 1; ++ j) {
            for(int k = -1; k < 1; ++ k) {
                for(auto e:bucket[i][j][k]) {
                    ret.push_back(e);
                }
            }
        }
    }
    return ret;
}

void find_seed_triangle(const Eigen::MatrixXd &V, int vid) {
    auto a = V.col(vid);
    auto pt_list = pick_neighbors(a);
    for(int i = 0; i < pt_list.size(); ++ i) {
        auto b = V.col(pt_list[i]);
        for(int j = 0; j < pt_list.size(); ++ j) {
            auto c = V.col(pt_list[j]);
            
            
        }
    }
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
        calculate_normals(VV, F, N);
        V = VV;
    } else {
        common::read_matrix_binary_from_file("../data/V", V);
        common::read_matrix_binary_from_file("../data/N", N);
    }

    move_positives(V);
    bucketsort(V);

    // common::save_obj("./bunny.obj", V, F);
    
    return 0;
}
