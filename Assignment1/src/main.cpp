#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector3f rotation_axies, 
                                float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    rotation_axies = rotation_axies / rotation_axies.norm();
    float x = rotation_axies[0];
    float y = rotation_axies[1];
    float z = rotation_axies[2];

    float theta = rotation_angle * MY_PI / 180.0;
    
    float psi = std::atan2(y, x);
    float phi = std::atan2(std::sqrt(x*x + y*y), z);

    Eigen::Matrix4f rzpsi, ryphi, rztheta;
    rzpsi << std::cos(psi), -std::sin(psi), 0, 0,
             std::sin(psi), std::cos(psi), 0, 0,
             0, 0, 1, 0, 
             0, 0, 0, 1;

    ryphi << std::cos(phi), 0, std::sin(phi), 0,
             0, 1, 0, 0,
             -std::sin(phi), 0, std::cos(phi), 0,
             0, 0, 0, 1;
    
    rztheta << std::cos(theta), -std::sin(theta), 0, 0,
               std::sin(theta), std::cos(theta), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    model = rzpsi.transpose() * model;
    model = ryphi.transpose() * model;
    model = rztheta * model;
    model = ryphi * model;
    model = rzpsi * model;

    return model;    
}


Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float a = rotation_angle * MY_PI / 180.0;
    Eigen::Matrix4f rotation;
    rotation << std::cos(a), -std::sin(a), 0, 0, 
                std::sin(a), std::cos(a), 0, 0,
                 0, 0, 1, 0, 0, 0, 0, 1;

    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

  
    float f = zFar;
    float n = zNear;
    float t = std::tan(eye_fov * MY_PI / 180.0 / 2.0) * std::fabs(n);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    Eigen::Matrix4f translate, scale, ortho, persp, ortho_persp;

    translate << 1, 0, 0, -(r+l)/2.0,
                 0, 1, 0, -(t+b)/2.0,
                 0, 0, 1, -(n+f)/2.0,
                 0, 0, 0, 1;

    scale << 2.0/(r-l), 0, 0, 0,
             0, 2.0/(t-b), 0, 0,
             0, 0, 2.0/(n-f), 0,
             0, 0, 0, 1;

    ortho_persp <<  n, 0, 0, 0,
                    0, n, 0, 0,
                    0, 0, n+f, -n*f,
                    0, 0, 1, 0;

    ortho = scale * translate;
    persp = ortho * ortho_persp;

    std::cout << persp << std::endl;

    projection = persp * projection;    

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    Eigen::Vector3f axies;
    bool any_axies = false;
    if (argc == 2) {
        std::cout << "input the axies: \n";
        float x, y, z;
        std::cin >> x >> y >> z;
        axies = {x, y, z};
        any_axies = true;
    }

    if (argc >= 3) {
        std::cout << argc << std::endl;
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if(any_axies) r.set_model(get_model_matrix(axies, angle));
        else r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
