#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;//全局变量，存储鼠标点击的四个点

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{//鼠标回调函数
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}
/**
 * 直接计算法绘制贝塞尔曲线（naive 实现）。
 * 使用三次贝塞尔曲线的显式公式：B(t) = (1-t)^3 P0 + 3t(1-t)^2 P1 + 3t^2(1-t) P2 + t^3 P3
 * 在窗口图像上逐点绘制，步长 dt = 0.001，将曲线点设为蓝色（B通道=255）。
 *
 * @param points 包含4个控制点的 vector，顺序为 P0, P1, P2, P3
 * @param window 要绘制的 OpenCV 图像（CV_8UC3，RGB 顺序）
 */
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];//拿到四个控制点

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
        return control_points[0];
    }

    // 生成下一层的新点集（大小 = 当前大小 - 1）
    std::vector<cv::Point2f> next_points;
    next_points.reserve(control_points.size() - 1);

    for (int i = 0; i < control_points.size() - 1; ++i) {
        // 线性插值： (1-t)*P_i + t*P_{i+1}
        cv::Point2f p = (1 - t) * control_points[i] + t * control_points[i + 1];
        next_points.push_back(p);//在数组末尾添加新元素
    }

    // 递归调用，继续缩减点集
    return recursive_bezier(next_points, t);
    //return cv::Point2f();

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0.0; t <= 1.0; t += 0.001)
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        // 3. 把浮点坐标四舍五入为整数（OpenCV 的像素索引必须为整数）
        int x = static_cast<int>(point.x + 0.5f);
        int y = static_cast<int>(point.y + 0.5f);

            window.at<cv::Vec3b>(y, x)[1] = 255;

    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
           naive_bezier(control_points, window);
             bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
