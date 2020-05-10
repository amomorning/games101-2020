#include "Pivoter.h"
#include <iostream>
#include <stdlib.h>


Pivoter::Pivoter() {
    memset(used, 0, sizeof(used));
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

bool Pivoter::check_front(int x){
    for(auto e:front) {
        if(e.u == x || e.v == x) return true;
    }
    return false;
}

void Pivoter::find_next_triangle(std::vector<Eigen::Vector3i> &tris, const Eigen::MatrixXd &V, const Eigen::MatrixXd &N)
{
    Edge e = front.front();
    front.pop_front();

    auto v = V.col(e.u);
    auto vn = N.col(e.u);
    auto obc = e.ball_center;

    auto m = (V.col(e.u) + V.col(e.v))/2;

    auto pt_list = pick_neighbors(v);
    Eigen::Vector3d b = V.col(e.v) - v;
    double max_angle = 0;
    int vid = -1;
    Eigen::Vector3d nbc;
    std::vector<int> tmp;
    for(int i = 0; i < pt_list.size(); ++ i) {
        bool flag = false;
        if(!used[pt_list[i]] || check_front(pt_list[i])) {

            Eigen::Vector3d a = V.col(pt_list[i]) - v;

            auto normal = a.cross(b);
            normal.normalize();
            
            // check normal
            double theta = acos(vn.dot(normal));
            if(theta > 0.5) continue;
            
            // get new triangle circle
            auto circle = get_circle(v, a+v, b+v);
            if(circle.second > ro) continue;
            
            // check if other point in ball
            auto ball_center = get_ball_center(circle, normal);
            if(used[pt_list[i]]) flag = true;
            else used[pt_list[i]] = true;

            if(check_ball(V, ball_center, pt_list)) {
                used[pt_list[i]] = false;
                continue;
            }

            // calculate angle 
            Eigen::Vector3d c0 = obc - m;
            c0.normalize();
            Eigen::Vector3d c1 = ball_center - m;
            c1.normalize();

            double angle = acos(c0.dot(c1));
            if(angle > max_angle) { 
                max_angle = angle;
                vid = pt_list[i];
                nbc = ball_center;

                if(flag) tmp.push_back(vid);
            }
            continue;
        }
    }
    if(~vid) {
        // std::cout << max_angle << " " << vid << std::endl;
        // std::cout << "find triangle: " << e.u << " " << vid << " " << e.v << std::endl;
        tris.push_back({e.u, vid, e.v});
        
        front.push_back({e.u, vid, e.v, nbc});
        front.push_back({vid, e.v, e.u, nbc});

        bool flag = false;
        for(auto t:tmp) if(t==vid) flag = true;
        if(flag) {
            delete_edge_from_front(e.u, vid);
            delete_edge_from_front(e.v, vid);
            delete_edge_from_front(vid, e.u);
            delete_edge_from_front(vid, e.v);
            delete_edge_from_front(e.u, e.v);
            delete_edge_from_front(e.v, e.u);
        }
    }

}

bool Pivoter::delete_edge_from_front(int u, int v) {
    std::list<Edge>::iterator it;
    for(it=front.begin(); it!=front.end();) {
        if(it->u == u && it->v == v) {
            front.erase(it++);
        } else it++;
    }
}

int Pivoter::find_seed_triangle(std::vector<Eigen::Vector3i> &tris, const Eigen::MatrixXd &V, const Eigen::MatrixXd &N, int seed) {

    srand(seed);
    int vid = rand()%2502 + 1;
    if(used[vid]) return -1;
    used[vid] = true;

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
            if(theta > 0.5) continue;
            
            auto circle = get_circle(v, a+v, b+v);
            if(circle.second > ro) continue;

            auto ball_center = get_ball_center(circle, normal);
            used[pt_list[i]] = true;
            used[pt_list[j]] = true;

            if(check_ball(V, ball_center, pt_list)) {
                used[pt_list[i]] = false;
                used[pt_list[j]] = false;
                continue;
            }


            front.push_back({vid, pt_list[i], pt_list[j], ball_center});
            front.push_back( {pt_list[i], pt_list[j], vid, ball_center});
            front.push_back({pt_list[j], vid, pt_list[i], ball_center});

            std::cout << pt_list[i] << " " << pt_list[j] << std::endl;

            std::cout << ball_center << std::endl;
            
            tris.push_back({vid, pt_list[i], pt_list[j]}); 
            return true;
            

        }
    }
    return false;
}

bool Pivoter::check_ball(const Eigen::MatrixXd &V, 
            const Eigen::Vector3d &ball_center, 
            const std::vector<int> &pt_list)
{
    for (auto e:pt_list) {
        if(used[e]) continue;
        
        double dist = (ball_center - V.col(e)).norm();
        if(dist < ro - 1e-7) {
            return true;
        }
    }
    return false;
}

void Pivoter::bucketsort(const Eigen::MatrixXd &V) {
    // bunny with 15x15x15 with max grid 14 points (2, 0, 6);
    int mx = 0;
    Eigen::Vector3i tmp;
    for (int i = 0; i < V.cols(); ++ i) {
        auto ps = get_position(V.col(i));
        // if(tmp[0] > ps[0] || tmp[1] > ps[1] || tmp[2] > ps[2])
            // tmp = ps;
        bucket[ps[0]][ps[1]][ps[2]].push_back(i); 
    }
    // std::cout << tmp << std::endl;
    
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
