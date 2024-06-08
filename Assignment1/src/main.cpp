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
    translate << 1, 0, 0
        -eye_pos[0], 0, 1, 0,
        -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float angle = rotation_angle / 180 * MY_PI;
    model << std::cos(angle), -std::sin(angle), 0, 0,
        std::sin(angle), std::cos(angle), 0, 0,
         0, 0, 1, 0,
          0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float alpha = eye_fov / 180 * MY_PI / 2;
    float n = zNear;
    float h = -n * std::tan(alpha);
    float w = h * aspect_ratio;
    float f = zFar;
    Eigen::Matrix4f presp_to_ortho = Eigen::Matrix4f::Identity();
    presp_to_ortho << n, 0, 0, 0,
                      0, n, 0, 0,
                      0, 0, n + f, -n * f,
                      0, 0, 1, 0;
    Eigen::Matrix4f ortho_trans = Eigen::Matrix4f::Identity();
    ortho_trans << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, -(n + f) / 2,
                   0, 0, 0, 1;
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
    ortho_scale << 1 / w, 0, 0, 0,
                   0, 1 / h, 0, 0,
                   0, 0, 2 / (n - f), 0,
                   0, 0, 0, 1;
    auto ortho = ortho_trans * ortho_scale;
    projection = ortho * presp_to_ortho;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float angle_rad = angle / 180 * MY_PI;
    axis.normalize();
    float x = axis[0];
    float y = axis[1];
    float z = axis[2];
    Eigen::Matrix3f cosi = Eigen::Matrix3f::Identity();
    cosi = cosi * std::cos(angle_rad);
    Eigen::Matrix3f nna = (1 - std::cos(angle_rad)) * (axis * axis.transpose());
    Eigen::Matrix3f nsi;
    nsi << 0, -z, y,
           z, 0, -x,
           -y, x, 0;
    nsi = std::sin(angle_rad) * nsi;
    auto R = cosi + nna + nsi;
    rotation.block(0, 0, 3, 3) = R;
    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f axis = {1,0,1};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // rotate by z
        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(30, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // rotate by z
        // r.set_model(get_model_matrix(angle));

        // rotate by any axis

        r.set_model(get_rotation(axis, angle));
        std::cout << "rot_ang" << '\n';
        r.set_view(get_view_matrix(eye_pos));
        std::cout << "view" << '\n';
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        std::cout << "proj" << '\n';

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        std::cout << "draw" << '\n';

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
