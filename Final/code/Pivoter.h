#include <eigen3/Eigen/Dense>

#include <vector>
#include <queue>

struct Edge {
    int u, v;
    bool active;
};

struct Triangle {
    int a, b, c;
    Eigen::Vector3d ball_center;
};

class Pivoter
{
public:
    const double ro = 0.005;
    std::vector<int> bucket[20][20][20];
    std::queue<Edge> front;
    int used[4000];

    Pivoter();
    ~Pivoter(){};

    void bucketsort(const Eigen::MatrixXd &V);
    Triangle find_seed_triangle(const Eigen::MatrixXd &V,const Eigen::MatrixXd &N, int vid);
    Eigen::Vector3i get_position(const Eigen::Vector3d pt);
    std::vector<int> pick_neighbors(const Eigen::Vector3d &pt);

    std::pair<Eigen::Vector3d, double> get_circle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
    Eigen::Vector3d get_ball_center(const std::pair<Eigen::Vector3d, double> &circle, const Eigen::Vector3d &normal);
    bool check_ball(const Eigen::MatrixXd &V, 
            const Eigen::Vector3d &ball_center, 
            const std::vector<int> &pt_list);
};
