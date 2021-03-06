#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

// 代数形式的方法
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 迭代中间点方法
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm

    // get control points num
    int num_control_points = control_points.size();

    if(num_control_points == 1) {
        // 返回像素坐标
        return control_points[0];
    } else {
        std::vector<cv::Point2f> next_control_points;
        cv::Point2f lerp_point;
        // 找到下次迭代的n-1个控制点
        for(int i = 1; i < num_control_points; i++) {
            // 第i个点和第i+1个点的插值
            lerp_point = (1 - t) * control_points[i - 1] + t * control_points[i];
            next_control_points.push_back(lerp_point);
        }
        // 进行下次迭代
        return recursive_bezier(next_control_points, t);
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.

    const float t_step = 0.001;
    static cv::Point2f point;
    for(float t = 0.0f; t <= 1.0f; t += t_step) {
        point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        // anti-aliasing
        // 判断采样点与像素的位置关系(注意opencv的坐标轴y是向下的)
        // (x, y)是采样点与所在像素左上角坐标的距离
        float x_len = point.x - std::floor(point.x);
        float y_len = point.y - std::floor(point.y);
        // x_len小于0.5则采样点靠所在像素左侧
        // y_len小于0.5则采样点靠所在像素上侧，可以由flag找到最靠近采样点的其他三个像素
        float x_flag = x_len < 0.5f ? -1.0f : 1.0f;
        float y_flag = y_len < 0.5f ? -1.0f : 1.0f;

        cv::Point2f p0 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p1 = cv::Point2f(std::floor(point.x) + 0.5f + x_flag, std::floor(point.y) + 0.5f);
        cv::Point2f p2 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f + y_flag);
        cv::Point2f p3 = cv::Point2f(std::floor(point.x) + 0.5f + y_flag, std::floor(point.y) + 0.5f + y_flag);

        std::vector<cv::Point2f> vec;
        vec.push_back(p1);
        vec.push_back(p2);
        vec.push_back(p3);

        // p0是采样点所在像素，最近的距离
        cv::Point2f min_dis_point = p0 - point;
        float min_distance = sqrt(min_dis_point.x * min_dis_point.x + min_dis_point.y * min_dis_point.y);

        // 求其他三点与采样点的距离，由距离插值
        for(auto p : vec) {
            cv::Point2f dis_point = p - point;
            float distance = sqrt(dis_point.x * dis_point.x + dis_point.y * dis_point.y);
            float percent = min_distance / distance;
            window.at<cv::Vec3b>(p.y, p.x)[1] = 255 * percent;
        }
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
