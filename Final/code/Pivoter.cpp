#include "Pivoter.h"
#include <iostream>
#include <stdlib.h>


Pivoter::Pivoter() {
    memset(notUsed, 0, sizeof(notUsed));
}

Eigen::Vector3i Pivoter::get_position(const Eigen::Vector3d pt) {
    double sigma = ro * 2; 
    int x = pt[0]/sigma + 1;
    int y = pt[1]/sigma + 1;
    int z = pt[2]/sigma + 1;
    return Eigen::Vector3i(x, y, z);
}


std::vector<int> Pivoter::pick_neighbors(const Eigen::Vector3d &pt) {
    auto ps = get_position(pt);
    std::vector<int> ret;
    for(int i = 0; i < 3; ++ i) {
        for(int j = 0; j < 3; ++ j) {
            for(int k = 0; k < 3; ++ k) {
                int dx = (i == 2)?-1:i;
                int dy = (j == 2)?-1:j;
                int dz = (k == 2)?-1:k;
                for(auto e:bucket[ps[0] + dx][ps[1] + dy][ps[2] + dz]) {
                    ret.push_back(e);
                }
            }
        }
    }
    return ret;
}

void Pivoter::find_seed_triangle(const Eigen::MatrixXd &V, const Eigen::MatrixXd &N, int seed) {

    srand(seed);
    int vid = rand()%1000 + 1000;
    std::cout << vid << std::endl;

    auto v = V.col(vid);
    auto vn = N.col(vid);

    auto pt_list = pick_neighbors(v);
    for(int i = 0; i < pt_list.size(); ++ i) {
        Eigen::Vector3d a = V.col(pt_list[i]) - v;
        for(int j = 0; j < pt_list.size(); ++ j) {
            Eigen::Vector3d b = V.col(pt_list[j]) - v;
            auto normal = a.cross(b);
            normal.normalize();

            double theta = acos(vn.dot(normal));
            if(theta > 0.2) continue;
            
            auto circle = get_circle(v, a+v, b+v);
            if(circle.second > ro) continue;

            auto ball_center = get_ball_center(circle, normal);
            std::cout << pt_list[i] << " " << pt_list[j] << std::endl;

            std::cout << ball_center << std::endl;
            break;
        }
    }
}

void Pivoter::bucketsort(const Eigen::MatrixXd &V) {
    // bunny with 15x15x15 with max grid 14 points (2, 0, 6);
    int mx = 0;
    for (int i = 0; i < V.cols(); ++ i) {
        auto ps = get_position(V.col(i));
        bucket[ps[0]][ps[1]][ps[2]].push_back(i); 
    }
}

std::pair<Eigen::Vector3d, double> Pivoter::get_circle(const Eigen::Vector3d &p0, 
                const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) 
{
    auto d01 = p0 - p1;
    auto d10 = p1 - p0;
    auto d12 = p1 - p2;
    auto d21 = p2 - p1;
    auto d20 = p2 - p0;
    auto d02 = p0 - p2;

    double norm01 = d01.norm();
    double norm12 = d12.norm();
    double norm20 = d20.norm();

    double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm20 * norm20 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

    Eigen::Vector3d center = alpha * p0 + beta * p1 + gamma * p2;
    double radius = (norm01 * norm12 * norm20) / (2 * norm01C12);

    return std::make_pair(center, radius);
}

Eigen::Vector3d Pivoter::get_ball_center(const std::pair<Eigen::Vector3d, double> &circle, const Eigen::Vector3d &normal)
{
    auto center = circle.first; 
    auto radius = circle.second;

    double h = sqrt(ro*ro - radius*radius);
    return center + h*normal;
}
