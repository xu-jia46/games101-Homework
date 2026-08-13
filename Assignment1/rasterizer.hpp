//
// Created by goksu on 4/6/19.
//

#pragma once//预处理指令，确保头文件在编译时只被包含一次

#include "Triangle.hpp"
#include <algorithm>//stl算法库
#include <eigen-3.4.0/Eigen/Eigen>/
using namespace Eigen;//使用eigen命名空间

namespace rst {//定义命名空间rst，将相关类型和类封装起来
enum class Buffers
{
    Color = 1,
    Depth = 2
};

inline Buffers operator|(Buffers a, Buffers b)
{
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b)
{
    return Buffers((int)a & (int)b);
}

enum class Primitive
{
    Line,//线框模式
    Triangle//填充模式
};

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
struct pos_buf_id
{
    int pos_id = 0;//储存位置缓冲区id，默认初始化为0
};

struct ind_buf_id
{
    int ind_id = 0;//储存索引缓冲区
};
//光栅化器类，3d模型转化为2d图像，并处理深度测试
class rasterizer
{
  public:
    rasterizer(int w, int h);//接收窗口的宽高，初始化缓冲区大小
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);//加载顶点位置数据到缓冲区
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);//加载三角形索引数据
    //构建mvp矩阵
    void set_model(const Eigen::Matrix4f& m);
    void set_view(const Eigen::Matrix4f& v);
    void set_projection(const Eigen::Matrix4f& p);

    void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);

  private:
    Eigen::Matrix4f model;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;//储存顶点位置数据到位置缓冲区
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;//储存三角形顶点索引数据到索引缓冲区

    std::vector<Eigen::Vector3f> frame_buf;//储存颜色值到帧缓冲区
    std::vector<float> depth_buf;//储存深度值到深度缓冲区
    int get_index(int x, int y);//根据像素计算索引

    int width, height;

    int next_id = 0;
    int get_next_id() { return next_id++; }//获取下一个id，每次调用返回当前值并递增
};
} // namespace rst
