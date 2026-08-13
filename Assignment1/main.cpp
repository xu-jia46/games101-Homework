#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen-3.4.0/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

//* 原理：将相机位置 (eye_pos) 平移到原点。因为物体和相机是相对运动，所以矩阵是平移矩阵的逆（即负平移）。
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();//初始化视图矩阵为单位矩阵

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;//构建平移矩阵，使相机回到原点

    view = translate * view;

    return view;
}

//* 作用：将物体从局部坐标系变换到世界坐标系。
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
   // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float cos_angle = cos(rotation_angle * MY_PI / 180);
    float sin_angle = sin(rotation_angle * MY_PI / 180);
    Eigen::Matrix4f model;
    model << cos_angle, -sin_angle,   0,   0,
            sin_angle,  cos_angle,   0,   0,
            0,           0,          1,   0,
            0,           0,          0,   1;

    //  Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    float n = zNear;
    float f = zFar;
    float t =abs( n) * tanf(eye_fov * MY_PI / 180 / 2);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;
    Eigen::Matrix4f M1;
    M1<< -n, 0, 0, 0,
         0, -n, 0, 0,
         0, 0, -(n + f), -n * f,
         0, 0, 1, 0;
    Eigen::Matrix4f Ms;
    Ms << 2 / (r - l), 0, 0, 0,
          0, 2 / (t - b), 0, 0,
          0, 0, 2 / (n - f), 0,
          0, 0, 0, 1;
    Eigen::Matrix4f Mt;
    Mt <<1, 0, 0, -(r + l) / 2,
         0, 1, 0, -(t + b) / 2,
         0, 0, 1, -(n + f) / 2,
         0, 0, 0, 1;
    Eigen::Matrix4f Mortho = Ms * Mt;

  
    Eigen::Matrix4f projection =  Mortho*M1;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;//当前旋转角度
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

    rst::rasterizer r(700, 700);//创建像素渲染窗口

    Eigen::Vector3f eye_pos = {0, 0, 5};//设置相机位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//定义三角形三个顶点位置

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);//数据写入光栅化渲染器的缓冲区，获取对应id

    int key = 0;//储存键盘按键值
    int frame_count = 0;//帧计数器

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//清除缓冲区

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//设置变换矩阵

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//绘制
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//将帧缓冲数据转为opencv图像并保存
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(90, 1, 0.1, 100));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {//a键逆时针旋转10度
            angle += 10;
        }
        else if (key == 'd') {//d键顺时针旋转10度
            angle -= 10;
        }
    }

    return 0;
}
