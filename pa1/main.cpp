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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rad = rotation_angle / 180.0 * MY_PI;
    model << std::cos(rad), -std::sin(rad), 0.0, 0.0,
        std::sin(rad), std::cos(rad), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthographic = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //aspect_ration = Width / Height = (r - l) / (t - b)
    /*float t = std::fabs(zNear) * std::tan(eye_fov / 2.0 / 180.0 * MY_PI);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;*/
    float Height = 2.0 * std::fabs(zNear) * std::tan(eye_fov / 2.0 / 180.0 * MY_PI);
    float Width = Height * aspect_ratio;
    orthographic << 2.0 / Width, 0.0, 0.0, 0.0,
                0.0, 2.0 / Height, 0.0, 0.0,
                0.0, 0.0, 2.0/(zNear - zFar), 0.0,
                0.0, 0.0, 0.0, 1.0;
    translation << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, (zNear + zFar)/2,
                0.0, 0.0, 0.0, 1.0;
    projection << zNear, 0.0, 0.0, 0.0,
                0.0, zNear, 0.0, 0.0,
                0.0, 0.0, zNear + zFar, -zNear * zFar,
                0.0, 0.0, 1.0, 0.0;
    projection = orthographic * translation * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
    float rad = angle / 180.0 * MY_PI;
    float SinA = std::sin(rad);
    float CosA = std::cos(rad);
    float Nx = axis(0);
    float Ny = axis(1);
    float Nz = axis(2);
    N << 
        0.0, -Nz, Ny,
        Nz, 0.0, -Nx,
        -Ny, Nx, 0.0;
    N = CosA * I + (1 - CosA) * axis*axis.transpose() + SinA * N;
    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity(); 
    ret<<
        N(0,0), N(0,1), N(0,2), 0.0,
        N(1,0), N(1,1), N(1,2), 0.0,
        N(2,0), N(2,1), N(2,2), 0.0,
        0.0,    0.0,    0.0,    1.0;
    return ret;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f rotation_axis (0.0f, 0.0f, 1.0f);
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else if (argc == 7) {
        // ./Rasterizer -r angel axis_x axis_y axis_z output_filename
        command_line = true;
        angle = std::stof(argv[2]);
        float axis_x = std::stof(argv[3]);
        float axis_y = std::stof(argv[4]);
        float axis_z = std::stof(argv[5]);
        rotation_axis = {axis_x, axis_y, axis_z};
        filename = std::string(argv[6]);
    }
        else
            return 0;
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

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(rotation_axis, angle));
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

        //r.set_model(get_model_matrix(angle));
        rotation_axis.normalize();
        Eigen::Matrix4f model_rotation = get_rotation(rotation_axis, angle);
        r.set_model(model_rotation);
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
