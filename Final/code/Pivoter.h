#include <eigen3/Eigen/Dense>

#include <vector>

class Pivoter
{
public:
    const double ro = 0.005;
    std::vector<int> bucket[20][20][20];
    int notUsed[4000];

    Pivoter();
    ~Pivoter(){};

    void bucketsort(const Eigen::MatrixXd &V);
    void find_seed_triangle(const Eigen::MatrixXd &V,const Eigen::MatrixXd &N, int vid);
    Eigen::Vector3i get_position(const Eigen::Vector3d pt);
    std::vector<int> pick_neighbors(const Eigen::Vector3d &pt);

    std::pair<Eigen::Vector3d, double> get_circle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
    Eigen::Vector3d get_ball_center(const std::pair<Eigen::Vector3d, double> &circle, const Eigen::Vector3d &normal);
};
