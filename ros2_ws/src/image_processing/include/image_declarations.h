#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <utility>
#include <vector>
#include <opencv2/core.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#define DEBUG false
using std::placeholders::_1;

// ─────────────────────────────────────────────────────────────────────────────
// Node: image_processor
//  Input  : /vehicle_front_camera/image_raw      (sensor_msgs/Image, RGB8)
//  Output :
//    /line/reference_path  (Float64MultiArray)  lateral offset [m] gần → xa
//    /line/cte             (Float64)             Cross-Track Error [m]
//    /line/poly_coeffs     (Float64MultiArray)   cubic pixel visualization
//    /line/debug_image     (sensor_msgs/Image)   scan bands xanh + centroid
//    /line/fitting_image   (sensor_msgs/Image)   đường cong cam + vanishing point đỏ
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int N = 12; // số scan line (band)

// clone_image tạo một bản sao của ảnh đầu vào, giữ nguyên header, kích thước, encoding và dữ liệu pixel.
sensor_msgs::msg::Image::SharedPtr clone_image(const sensor_msgs::msg::Image::SharedPtr src);

class ImageProcessor : public rclcpp::Node
{
  // ── Members ───────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ref_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cte_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_err_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_angle_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_evaluation_forward_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_kappa_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_poly_coeffs_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_fitting_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_yellow_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_dark_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_corridor_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_scan_mask_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_boundary_;
  // Thêm publisher cho s_metric và kappa_s
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_s_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kappa_s_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_mode_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_confidence_;

  double scan_top_ratio_;
  double scan_bot_ratio_;
  double scan_left_ratio_;
  double scan_right_ratio_;
  int black_threshold_;
  int band_height_;
  int min_valid_bands_;
  bool boundary_corridor_enabled_;
  int boundary_margin_px_;
  bool boundary_recovery_enabled_;
  int boundary_recovery_max_age_ms_;
  int boundary_recovery_min_observations_;
  double boundary_search_left_ratio_;
  double boundary_search_right_ratio_;
  bool previous_boundary_corridor_valid_ = false;
  rclcpp::Time previous_boundary_stamp_;
  std::vector<double> previous_left_boundary_poly_;
  std::vector<double> previous_right_boundary_poly_;
  double cte_evaluation_forward_;
  std::string ground_frame_;

public:
  ImageProcessor();

private:
  // Camera intrinsic matrix and flag
  cv::Mat K_; // Intrinsic matrix from CameraInfo
  bool has_camera_info_ = false;
  std::string camera_frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Trả về trung điểm giữa pixel đen ngoài cùng bên trái và ngoài cùng bên phải trên 1 dòng.
  double scan_row(const unsigned char *data, int row, int width, int step, int channels);

  bool check_valid_image(const sensor_msgs::msg::Image::SharedPtr msg);

  cv::Mat build_scan_mask(
      const sensor_msgs::msg::Image::SharedPtr msg,
      int scan_top,
      int scan_bot);

  // Trả về false nếu không tìm thấy vạch ở bất kỳ band nào; out_valid_bands set to number of bands with at least one valid midpoint.
  bool compute_offsets(
      const unsigned char *data, int width, int height, int step, int channels,
      double image_center, int scan_bot, int span,
      std::vector<double> &lateral_offset, std::vector<int> &scan_rows,
      std::vector<bool> &valid_bands,
      int &out_valid_bands);

  // ROS 2 sẽ gọi image_callback mỗi khi có frame mới trên /my_camera/image_raw; tại đây xử lý ảnh, tính toán offset và publish kết quả.
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void publish_debug_image(
      const sensor_msgs::msg::Image::SharedPtr src,
      const cv::Mat &scan_mask,
      const std::vector<int> &rows,
      const std::vector<double> &offsets,
      const std::vector<bool> &valid_bands,
      double image_center);

  // Vẽ màu các scan line và đánh dấu centroid để dễ dàng debug khi xem ảnh đầu ra trên /line/debug_image.
  void draw_all_bands(
      unsigned char *d,
      const cv::Mat &scan_mask,
      const sensor_msgs::msg::Image::SharedPtr src,
      int step, int channels, int width,
      const std::vector<int> &rows,
      const std::vector<double> &offsets,
      const std::vector<bool> &valid_bands,
      double image_center);

  // ── Fitting (image_fitting.cpp) ──────────────────────────────────────────
  // Trả về degree+1 hệ số: x = c0 + c1*y + ... + c_degree*y^degree.
  std::vector<double> poly_fit(
      const std::vector<int> &rows,
      const std::vector<double> &centroid_x,
      int degree = 3);

  // Vẽ đường cong polynomial (màu cam) lên buffer ảnh
  void draw_poly_curve(
      unsigned char *d,
      int step, int channels, int width,
      const std::vector<double> &coeffs,
      int scan_top, int scan_bot);

  // Publish /line/fitting_image: ảnh sạch + curve cam + centroid trắng + vanishing đỏ
  void publish_fitting_image(
      const sensor_msgs::msg::Image::SharedPtr src,
      const std::vector<int> &scan_rows,
      const std::vector<double> &centroid_x,
      const std::vector<bool> &valid_bands,
      const std::vector<double> &poly_coeffs,
      int scan_top, int scan_bot);

  void publish_line_health(const std::string &status, double confidence);
};
