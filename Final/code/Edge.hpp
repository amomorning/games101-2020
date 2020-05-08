#pragma once
#include <eigen3/Eigen/Dense>

class Edge {
    bool active;
    double radius;
    Eigen::Vector3d v0, v1;
    Eigen::Vector3d ball_center;
    Eigen::Vector3d middle_point;

    Edge(const Eigen::Vector3d &_v0, const Eigen::Vector3d &_v1, const Eigen::Vector3d &_bc){
        v0 = _v0;
        v1 = _v1;
        ball_center = _bc;
        middle_point = (v0+v1) / 2;
        radius = (ball_center - v0).norm();
        active = true;
    };
    Edge();
    ~Edge();

    inline void setActive(bool _active) { active = _active; };
    inline bool isActive() const { return active; }
    inline double get_radius() { return radius; }

    inline Eigen::Vector3d get_vertex(int _n) { return _n?v1:v0;}
    inline Eigen::Vector3d get_ball_center() { return ball_center; }


};
