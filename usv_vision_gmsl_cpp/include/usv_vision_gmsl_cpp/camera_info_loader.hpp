/**
 * CameraInfoLoader —— 相机内参标定加载器（header-only）
 *
 * 与 Python 侧 usv_vision/common/camera_info.py 的 CameraInfoLoader 对齐：
 * - 从 camera_info_url 指向的 JSON 文件读取 fx/fy/cx/cy 与畸变系数
 * - 标定分辨率与实际分辨率不一致时按比例缩放内参
 * - 按分辨率缓存，避免重复解析
 * - 未配置 / 文件不存在 / 解析失败时返回空标定（K/D 全零、distortion_model 为空），
 *   保证内参缺失不影响节点正常出图
 *
 * 供 gmsl_camera_node_cpp 与 gmsl_multi_camera_sync_node 共用。
 */
#ifndef USV_VISION_GMSL_CPP_CAMERA_INFO_LOADER_HPP
#define USV_VISION_GMSL_CPP_CAMERA_INFO_LOADER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <exception>
#include <cmath>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nlohmann/json.hpp>

namespace usv_vision_gmsl_cpp
{

class CameraInfoLoader
{
public:
    // fov_x_deg / fov_y_deg：规格书标称视场角（度），仅在标定文件缺失时用于反算近似内参；
    // 两者 ≤ 0 表示未提供，则退回空标定
    CameraInfoLoader(rclcpp::Logger logger, std::string frame_id, std::string camera_info_url,
                     double fov_x_deg = 0.0, double fov_y_deg = 0.0)
        : logger_(logger),
          frame_id_(std::move(frame_id)),
          camera_info_url_(std::move(camera_info_url)),
          fov_x_deg_(fov_x_deg),
          fov_y_deg_(fov_y_deg)
    {
    }

    // 获取指定分辨率的 camera_info（按分辨率缓存）
    sensor_msgs::msg::CameraInfo get(int width, int height)
    {
        if (!cached_ || cached_width_ != width || cached_height_ != height) {
            cached_info_ = load(width, height);
            cached_width_ = width;
            cached_height_ = height;
            cached_ = true;
        }
        return cached_info_;
    }

private:
    static bool file_exists(const std::string& path)
    {
        struct stat buffer;
        return !path.empty() && stat(path.c_str(), &buffer) == 0;
    }

    sensor_msgs::msg::CameraInfo load(int width, int height)
    {
        using json = nlohmann::json;

        sensor_msgs::msg::CameraInfo msg;
        msg.width = static_cast<uint32_t>(width);
        msg.height = static_cast<uint32_t>(height);
        msg.header.frame_id = frame_id_;

        if (file_exists(camera_info_url_)) {
            try {
                std::ifstream file(camera_info_url_);
                json data = json::parse(file);

                double fx = data["focal_length"]["fx"].get<double>();
                double fy = data["focal_length"]["fy"].get<double>();
                double cx = data["principal_point"]["cx"].get<double>();
                double cy = data["principal_point"]["cy"].get<double>();

                // 标定分辨率：优先 image_width/height，其次 resolution.width/height
                int calib_w = 0, calib_h = 0;
                if (data.contains("image_width") && data.contains("image_height")) {
                    calib_w = data["image_width"].get<int>();
                    calib_h = data["image_height"].get<int>();
                } else if (data.contains("resolution")) {
                    calib_w = data["resolution"].value("width", 0);
                    calib_h = data["resolution"].value("height", 0);
                }

                // 分辨率不一致时按比例缩放内参
                if (calib_w > 0 && calib_h > 0 && (calib_w != width || calib_h != height)) {
                    double sx = static_cast<double>(width) / calib_w;
                    double sy = static_cast<double>(height) / calib_h;
                    fx *= sx; fy *= sy; cx *= sx; cy *= sy;
                    RCLCPP_WARN(logger_,
                        "camera info resized from %dx%d to %dx%d", calib_w, calib_h, width, height);
                }

                // 畸变系数：distortion_coefficients 或 dist_coeffs，支持嵌套数组 [[...]]
                std::vector<double> distortion;
                json dist_node;
                if (data.contains("distortion_coefficients")) {
                    dist_node = data["distortion_coefficients"];
                } else if (data.contains("dist_coeffs")) {
                    dist_node = data["dist_coeffs"];
                }
                if (dist_node.is_array() && !dist_node.empty()) {
                    const json& row = dist_node[0].is_array() ? dist_node[0] : dist_node;
                    for (const auto& v : row) {
                        distortion.push_back(v.get<double>());
                    }
                }
                if (distortion.empty()) {
                    distortion.assign(5, 0.0);
                }

                msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
                msg.d = distortion;
                msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
                msg.distortion_model = "plumb_bob";
                RCLCPP_INFO(logger_, "loaded camera info: %s", camera_info_url_.c_str());
                return msg;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_,
                    "failed to parse camera info, fallback to empty: %s", e.what());
            }
        }

        // 无标定文件：若提供了规格书 FOV，则反算近似内参兜底
        if (fov_x_deg_ > 0.0 || fov_y_deg_ > 0.0) {
            // 允许只给一个方向，另一方向按像素等效焦距推算，保证 fx/fy 都有值
            constexpr double kDeg2Rad = M_PI / 180.0;
            double fx = 0.0, fy = 0.0;
            if (fov_x_deg_ > 0.0) {
                fx = (width / 2.0) / std::tan(fov_x_deg_ * kDeg2Rad / 2.0);
            }
            if (fov_y_deg_ > 0.0) {
                fy = (height / 2.0) / std::tan(fov_y_deg_ * kDeg2Rad / 2.0);
            }
            // 只给了一个方向时，用等效焦距（像素）补齐另一方向，避免 0 焦距
            if (fx <= 0.0) fx = fy;
            if (fy <= 0.0) fy = fx;

            if (fx > 0.0 && fy > 0.0) {
                double cx = width / 2.0;
                double cy = height / 2.0;
                msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
                msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};  // 规格书无畸变数据，按无畸变处理
                msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
                msg.distortion_model = "plumb_bob";
                RCLCPP_WARN(logger_,
                    "no calibration file, using spec-sheet FOV fallback: "
                    "fov=(%.1f, %.1f)deg -> fx=%.1f fy=%.1f @ %dx%d (approx, no distortion)",
                    fov_x_deg_, fov_y_deg_, fx, fy, width, height);
                return msg;
            }
        }

        // 无标定、无 FOV / 解析失败：发布空标定，不影响节点运行（与 Python 版一致）
        msg.k = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        msg.p = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
        msg.distortion_model = "";
        return msg;
    }

    rclcpp::Logger logger_;
    std::string frame_id_;
    std::string camera_info_url_;
    double fov_x_deg_ = 0.0;
    double fov_y_deg_ = 0.0;

    sensor_msgs::msg::CameraInfo cached_info_;
    bool cached_ = false;
    int cached_width_ = 0;
    int cached_height_ = 0;
};

}  // namespace usv_vision_gmsl_cpp

#endif  // USV_VISION_GMSL_CPP_CAMERA_INFO_LOADER_HPP
