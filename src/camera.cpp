#include "DynamicObjectsAvoidance/camera.h"

namespace DynamicObjectsAvoidance {

Camera::Camera() {
}

Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    return pose_ * T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Vec3 Camera::camera2camera(const Vec3 &p_c, const SE3 &T_c_c) {
    return T_c_c * p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Vec2 Camera::camera2pixelDistortion(const Vec3 &p_c) {
    double x = p_c(0, 0) / p_c(2, 0), y = p_c(1, 0) / p_c(2, 0);
    double r2 = x * x + y * y;
    double d = 1 + coeffs_[0] * r2 + coeffs_[1] * r2 * r2;
    double xd = x * d;
    double yd = y * d;
    double dx = xd + 2 * coeffs_[2] * x * y + coeffs_[3] * (r2 + 2 * x * x);
    double dy = yd + 2 * coeffs_[3] * x * y + coeffs_[2] * (r2 + 2 * y * y);

    x = dx;
    y = dy;
    return Vec2(fx_ * x + cx_, fy_ * y + cy_);
}

Vec2 Camera::camera2pixel(const Vec3 &p_c, const int &lvl) {
    return Vec2(
            fx_pyr_[lvl] * p_c(0, 0) / p_c(2, 0) + cx_pyr_[lvl],
            fy_pyr_[lvl] * p_c(1, 0) / p_c(2, 0) + cy_pyr_[lvl]
    );
}

Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth /*=1*/) {
    return Vec3(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

// Order for Brown-Conrady: [k1, k2, p1, p2, k3]
Vec3 Camera::pixel2cameraDistortion(const Vec2 &p_p, double depth /*=1*/) {
    double x = (p_p(0, 0) - cx_) / fx_;
    double y = (p_p(1, 0) - cy_) / fy_;

    double xo = x;
    double yo = y;
    double r2 = x * x + y * y;
    double icdist = (double)1 / (double)(1 + ((coeffs_[4] * r2 + coeffs_[1]) * r2 + coeffs_[0]) * r2);
    double delta_x = 2 * coeffs_[2] * x * y + coeffs_[3] * (r2 + 2 * x * x);
    double delta_y = 2 * coeffs_[3] * x * y + coeffs_[2] * (r2 + 2 * y * y);
    x = (xo - delta_x) * icdist;
    y = (yo - delta_y) * icdist;
    return Vec3(x * depth, y * depth, depth);
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth /*=1*/) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

}
