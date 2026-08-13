//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

//数据加载，缓冲区管理

//加载顶点位置数据
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}
//加载三角形索引数据
rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;//定义整数变量

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);//取绝对值
    px=2*dy1-dx1;//决策参数
    py=2*dx1-dy1;
    //情况1；|k|<=1,步进x找y
    if(dy1<=dx1)
    {
        if(dx>=0)//确保x从小到大遍历
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else//否则交换起点终点
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        //先画起点像素
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
    else//情况2|k|>1，步进y，找x
    {
        if(dy>=0)//确保y是从小到大遍历
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}
auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//----------------------------核心·绘制函数----------------------------
void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//遍历所有三角形索引，ind中每一个元素是一个vector3i，存了三个顶点下标
    {
        Triangle t;//创建临时三角形对象

        Eigen::Vector4f v[] = {//对三个顶点进行mvp变换并齐次化
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        
        for (auto& vec : v) {
            vec /= vec.w();//透视除法，将齐次坐标从裁剪空间转换到标准化设备坐标 NDC，即 w=1）
        }

        //视口变换，视口变换（将 NDC 中的 [-1,1]^2 映射到屏幕坐标 [0, width] x [0, height]）
        // 同时将深度 z 从 [-1,1] 映射到 [0.1, 100]（近远平面）
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0); // x: (ndc.x + 1) / 2 * width
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2; // z: 线性映射到深度范围
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        t.setColor(0,255,  0,0 );//设置顶点0的颜色
        t.setColor(1, 0, 255,0 );//设置顶点1的颜色
        t.setColor(2, 0,  0,255);//设置顶点2的颜色

        rasterize_wireframe(t);
    }
}

//--------------------------线框光栅化（调用画线）-----------------------------------
void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}
//--------------------------矩阵设置------------------------------------------------
void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}
//---------------------清除缓冲区-------------------
void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {// std::fill 用于将容器区间填充为指定值
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {// std::numeric_limits<float>::infinity() 返回正无穷大
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}
//------------------------构造函数
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);//设置缓冲区大小
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;//计算缓冲区的像素索引
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;//裁剪屏幕外的点

    auto ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;//赋值颜色
}

