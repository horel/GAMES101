// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <array>
#include <cfloat>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f P(x, y, 1.0f);

    const Eigen::Vector3f &A = _v[0];
    const Eigen::Vector3f &B = _v[1];
    const Eigen::Vector3f &C = _v[2];

    Eigen::Vector3f AB = B - A;
    Eigen::Vector3f BC = C - B;
    Eigen::Vector3f CA = A - C;

    Eigen::Vector3f AP = P - A;
    Eigen::Vector3f BP = P - B;
    Eigen::Vector3f CP = P - C;

    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
// 只有光栅化的时候z值反转了
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // std::array<Vector4f, 3> toVector4() const;
    // v存储三角形三个点的向量
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // 用第一个点初始化包围盒
    float aabb_minX = v[0].x();
    float aabb_maxX = v[0].x();
    float aabb_minY = v[0].y();
    float aabb_maxY = v[0].y();
    // 遍历三个点得到包围盒
    for(int i = 0; i < 3; i++) {
        const Eigen::Vector3f &P = t.v[i];

        aabb_minX = std::min(P.x(), aabb_minX);
        aabb_maxX = std::max(P.x(), aabb_maxX);
        aabb_minY = std::min(P.x(), aabb_minY);
        aabb_maxY = std::max(P.x(), aabb_maxY);
    }

    // 提高题 MSAA 4X
    bool MSAA = true;
    if(MSAA) {
        // 对每个像素
        for(int x = aabb_minX; x < aabb_maxX; x++) {
            for(int y = aabb_minY; y < aabb_maxX; y++) {
                int sample_count = 0;

                // 对每个采样点
                for(int sup_x : {0, 1}) {
                    for(int sup_y : {0, 1}) {
                        float x_pos = (float)x + 0.25 + 0.5 * sup_x;
                        float y_pos = (float)y + 0.25 + 0.5 * sup_y;

                        if(insideTriangle(x_pos, y_pos, t.v)) {
                            auto[alpha, beta, gamma] = computeBarycentric2D(x_pos, y_pos, t.v);
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            // 从左下角开始计数
                            int buf_ind = (x*2 + sup_x) + (y*2 + sup_y)*width*2;
                            if(z_interpolated < depth_buf_msaa2x2[buf_ind]) {
                                depth_buf_msaa2x2[buf_ind] = z_interpolated;
                                sample_count++;
                            }
                        }
                    }
                }

                if(sample_count > 0) {
                    float intensity = (float)sample_count / 4.0f;
                    mix_pixel(Eigen::Vector3f(x, y, 1.0f), t.getColor()*intensity);
                }
            }
        }
    } else {

        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
        for(int x = (int)aabb_minX; x < aabb_maxX; x++) {
            for(int y = (int)aabb_minX; y < aabb_maxY; y++) {

                // iterate through the pixel and find if the current pixel is inside the triangle
                if(!insideTriangle(x+0.5f, y+0.5f, t.v))
                    continue;
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // index是给整个屏幕的深度缓冲区排序了, buf_index获得对应(x,y)点的序号
                float buf_index = get_index(x, y);
                // 插值深度值与深度缓冲区depth_buf[序号]中的值比较
                if(z_interpolated >= depth_buf[buf_index])
                    continue;
                // 当前点更靠近相机，设置像素颜色并更新深度缓冲区
                depth_buf[buf_index] = z_interpolated;
                set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());
            }
        }
        // TODO_END

    }
}

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

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_msaa2x2.begin(), depth_buf_msaa2x2.end(), std::numeric_limits<float>::infinity());  // added by user for msaa bonus
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_msaa2x2.resize(w * h * 2 * 2);  // added by user for msaa bonus
}

// 得到(x,y)是包围盒的第几个像素
int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// added by user for msaa bonus
void rst::rasterizer::mix_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    // 两三角形边界处应该是颜色的混合
    // 题目里边界处就是3/4的绿色混合1/4的蓝色
    frame_buf[ind] += color;
}

// clang-format on
