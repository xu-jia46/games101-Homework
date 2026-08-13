#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"//定义三角形数据结构，包含顶点坐标，法线，纹理坐标等
#include "Shader.hpp"//定义着色器的payload，也就是着色器函数接收的参数包
#include "Texture.hpp"//加载图片，提供采样接口
#include "OBJ_Loader.h"//解析obj文件


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{

    float n = zNear;
    float f = zFar;
    float t = abs(n) * tanf(eye_fov * MY_PI / 180 / 2);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;
    Eigen::Matrix4f M1;
    M1 << -n, 0, 0, 0,
        0, -n, 0, 0,
        0, 0, -(n + f), -n * f,
        0, 0, 1, 0;
    Eigen::Matrix4f Ms;
    Ms << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    Eigen::Matrix4f Mt;
    Mt << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f Mortho = Ms * Mt;
    Eigen::Matrix4f projection = Mortho * M1;

    return projection;


}
//顶点shader
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}
//将法向量从【-1，1】映射到【0-1】作为颜色输出
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = std::max(0.0f, std::min(1.0f, payload.tex_coords.x()));
        float v = std::max(0.0f, std::min(1.0f, payload.tex_coords.y()));
        //Eigen::Vector3f getColor(float u, float v);
        return_color = payload.texture->getColor(u, v);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        //  For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Vector3f light_dir = (light.position - point).normalized();
        float r2 = (light.position - point).squaredNorm();   // 距离平方
        Vector3f intensity = light.intensity / r2;
        float diff = std::max(0.0f, normal.dot(light_dir));
        Vector3f diffuse = kd.cwiseProduct(intensity) * diff;

        Vector3f v = (eye_pos - point).normalized();
        Vector3f h = (v + light_dir).normalized();
        float spec = powf(std::max(0.0f, normal.dot(h)), p);
        Vector3f specular = ks.cwiseProduct(intensity) * spec;

        result_color += diffuse + specular;
    }
    Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
    result_color += ambient;
   
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Vector3f light_dir = (light.position - point).normalized();
        float r2 = (light.position - point).squaredNorm();   // 距离平方
        Vector3f intensity = light.intensity / r2;
        float diff = std::max(0.0f, normal.dot(light_dir));
        Vector3f diffuse = kd.cwiseProduct(intensity)  * diff;

        Vector3f v = (eye_pos - point).normalized();
        Vector3f h = (v + light_dir).normalized();
        float spec = powf(std::max(0.0f, normal.dot(h)), p);
        Vector3f specular = ks.cwiseProduct(intensity) * spec;

        result_color += diffuse + specular;    
    }
    Vector3f ambient = ka.cwiseProduct(amb_light_intensity) ;
    result_color += ambient;
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal.normalized();//原始法线
    float x = n.x(), y = n.y(), z = n.z();
    Eigen::Vector3f t = Eigen::Vector3f(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));//切线
    Eigen::Vector3f  b = n.cross(t).normalized();//副切线，（t,b,n)构成一组标准正交基
    Eigen::Matrix3f TBN;
    TBN << t, b, n;//把切线空间的向量转化为视图空间的矩阵

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;//拿到纹理坐标及纹理像素

    // 对当前点坐标 clamp
    float u_uv = std::max(0.0f, std::min(1.0f, u));
    float v_uv = std::max(0.0f, std::min(1.0f, v));

    float u_next = std::max(0.0f, std::min(1.0f, u + 1.0f / w));
    float v_next = std::max(0.0f, std::min(1.0f, v + 1.0f / h));

    float h_uv = payload.texture->getColor(u_uv, v_uv).norm();//当前点高度
    float h_u = payload.texture->getColor(u_next, v_uv).norm();//u方向步进一个单位（像素宽度）的高度
    float h_v = payload.texture->getColor(u_uv, v_next).norm();//v方向上步进一个单位的高度

    float dU = kh * kn * (h_u - h_uv);//u方向上的导数
    float dV = kh * kn * (h_v - h_uv);//v方向上的导数
    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f).normalized();//最终法向量
    Eigen::Vector3f new_normal = (TBN * ln).normalized();//切线空间转换到视图空间

    Eigen::Vector3f result_color = {0, 0, 0};
    point = point + kn * n * h_uv;
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Vector3f light_dir = (light.position - point).normalized();
        float r2 = (light.position - point).squaredNorm();   // 距离平方
        Vector3f intensity = light.intensity / r2;
        float diff = std::max(0.0f, new_normal.dot(light_dir));
        Vector3f diffuse = kd.cwiseProduct(intensity) * diff;

        Vector3f v = (eye_pos - point).normalized();
        Vector3f h = (v + light_dir).normalized();
        float spec = powf(std::max(0.0f, new_normal.dot(h)), p);
        Vector3f specular = ks.cwiseProduct(intensity) * spec;

        result_color += diffuse + specular;

    }

    Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
    result_color += ambient;

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    //  Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal.normalized();//原始法线
    float x = n.x(), y = n.y(), z = n.z();
    Eigen::Vector3f t = Eigen::Vector3f(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));//切线
    Eigen::Vector3f  b = n.cross(t).normalized();//副切线，（t,b,n)构成一组标准正交基
    Eigen::Matrix3f TBN;
    TBN << t, b, n;//把切线空间的向量转化为视图空间的矩阵

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;//拿到纹理坐标及纹理像素

    // 对当前点坐标 clamp
    float u_uv = std::max(0.0f, std::min(1.0f, u));
    float v_uv = std::max(0.0f, std::min(1.0f, v));

    float u_next = std::max(0.0f, std::min(1.0f, u + 1.0f / w));
    float v_next = std::max(0.0f, std::min(1.0f, v + 1.0f / h));

    float h_uv = payload.texture->getColor(u_uv, v_uv).norm();//当前点高度
    float h_u = payload.texture->getColor(u_next, v_uv).norm();//u方向步进一个单位（像素宽度）的高度
    float h_v = payload.texture->getColor(u_uv, v_next).norm();//v方向上步进一个单位的高度

    float dU = kh * kn * (h_u - h_uv);//u方向上的导数
    float dV = kh * kn * (h_v - h_uv);//v方向上的导数
    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f).normalized();//最终法向量
    Eigen::Vector3f new_normal = (TBN * ln).normalized();//切线空间转换到视图空间

    //Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f result_color = new_normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;//若程序启动带有命令行参数则为真，（渲染一帧就退出）假则走交互式实时渲染

    std::string filename = "output.png";//默认输出文件名
    objl::Loader Loader;//第三方obj解析器对象
    std::string obj_path = "models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("models/spot/spot_triangulated_good.obj");//加载模型，参数是文件路径
    for(auto mesh:Loader.LoadedMeshes)//遍历加载进来的所有网格
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)//三个顶点组成一个三角形，所以步长是3
        {
            Triangle* t = new Triangle();//在堆上动态分配一个三角形对象，并返回其指针
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));//齐次坐标下的point
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);//将这个组装好的三角形填入指针
        }
    }

    rst::rasterizer r(700, 700);

   auto texture_path = "hmap.jpg";
   r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)//argc是命令行参数个数
    {
        command_line = true;
        filename = std::string(argv[1]);//文件名打印第一个参数名

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);//设置片元着色器状态，在draw过程中调用

    int key = 0;
    int frame_count = 0;

    if (command_line)//检测之前的命令行参数，进入这个分支
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//同时清空颜色和深度缓冲区
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));//计算变换矩阵供光栅化器调用

        r.draw(TriangleList);//光栅化器遍历所有三角新，执行光栅化算法
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);//转换颜色通道

        cv::imwrite(filename, image);

        return 0;
    }
    //交互实时渲染循环
    while(key != 27)//27是esc键的ASCll码，按esc退出循环
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);//每帧重新设置矩阵，重新绘制

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);//弹出显示渲染画面
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
