#include <sensor_msgs/msg/camera_info.hpp>
#include "image_declarations.h"

// ── Constructor (public) ──────────────────────────────────────────────────────
ImageProcessor::ImageProcessor() : Node("image_processor")
{
  this->declare_parameter("scan_top_ratio", 0.40); // bắt đầu quét tại 40% chiều cao ảnh
  this->declare_parameter("scan_bot_ratio", 0.63); // dừng quét tại 63% chiều cao ảnh
  this->declare_parameter("scan_left_ratio", 0.0); // quét từ mép trái ảnh
  this->declare_parameter("scan_right_ratio", 1.0); // quét đến mép phải ảnh
  this->declare_parameter("black_threshold", 60);  // pixel < threshold → màu đen
  // this->declare_parameter("white_threshold",  200);   // pixel > 200 → coi là vạch trắng
  this->declare_parameter("band_height", 6); // chiều dày mỗi scan band [px]
  this->declare_parameter("min_valid_bands", 6);
  this->declare_parameter("boundary_corridor_enabled", true);
  this->declare_parameter("boundary_margin_px", 4);
  this->declare_parameter("boundary_recovery_enabled", true);
  this->declare_parameter("boundary_recovery_max_age_ms", 150);
  this->declare_parameter("boundary_recovery_min_observations", 3);
  this->declare_parameter("boundary_search_left_ratio", 0.0);
  this->declare_parameter("boundary_search_right_ratio", 1.0);
  // Giá trị âm: tự động đánh giá CTE/heading tại điểm IPM gần nhất.
  this->declare_parameter("cte_evaluation_forward", -1.0);
  this->declare_parameter("ground_frame", "base_link");
  scan_top_ratio_ = this->get_parameter("scan_top_ratio").as_double();
  scan_bot_ratio_ = this->get_parameter("scan_bot_ratio").as_double();
  scan_left_ratio_ = this->get_parameter("scan_left_ratio").as_double();
  scan_right_ratio_ = this->get_parameter("scan_right_ratio").as_double();
  black_threshold_ = this->get_parameter("black_threshold").as_int();
  // white_threshold_ = this->get_parameter("white_threshold").as_int();
  band_height_ = this->get_parameter("band_height").as_int();
  min_valid_bands_ = this->get_parameter("min_valid_bands").as_int();
  boundary_corridor_enabled_ =
      this->get_parameter("boundary_corridor_enabled").as_bool();
  boundary_margin_px_ =
      this->get_parameter("boundary_margin_px").as_int();
  boundary_recovery_enabled_ =
      this->get_parameter("boundary_recovery_enabled").as_bool();
  boundary_recovery_max_age_ms_ =
      this->get_parameter("boundary_recovery_max_age_ms").as_int();
  boundary_recovery_min_observations_ =
      this->get_parameter("boundary_recovery_min_observations").as_int();
  boundary_search_left_ratio_ =
      this->get_parameter("boundary_search_left_ratio").as_double();
  boundary_search_right_ratio_ =
      this->get_parameter("boundary_search_right_ratio").as_double();
  cte_evaluation_forward_ =
      this->get_parameter("cte_evaluation_forward").as_double();
  ground_frame_ = this->get_parameter("ground_frame").as_string();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Subscribers / Publishers ─────────────────────────────────────
  // Tạo subscriber lắng nghe topic ảnh thô từ camera; mỗi khi có frame mới,
  // ROS 2 sẽ gọi image_callback để xử lý. Queue size=10 cho phép giữ tối đa
  // 10 message trong hàng đợi nếu callback xử lý không kịp tốc độ publish.
  sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/front_camera/image_raw", 10,
      std::bind(&ImageProcessor::image_callback, this, _1));

  // Subscribe to camera info
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/front_camera/camera_info", 10,
      std::bind(&ImageProcessor::camera_info_callback, this, std::placeholders::_1));

  pub_ref_path_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/reference_path", 10);
  pub_cte_ = this->create_publisher<std_msgs::msg::Float64>("/line/cte", 10);
  pub_heading_err_ = this->create_publisher<std_msgs::msg::Float64>("/line/heading_error", 10);
  pub_poly_coeffs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/poly_coeffs", 10);
  pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/line/debug_image", 10);
  pub_fitting_image_ = this->create_publisher<sensor_msgs::msg::Image>("/line/fitting_image", 10);
  pub_debug_yellow_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/line/debug/yellow_mask", 10);
  pub_debug_dark_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/line/debug/dark_mask", 10);
  pub_debug_corridor_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/line/debug/corridor_mask", 10);
  pub_debug_scan_mask_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/line/debug/scan_mask", 10);
  pub_debug_boundary_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/line/debug/boundary_overlay", 10);
  pub_kappa_ = this->create_publisher<std_msgs::msg::Float64>("/line/kappa", 10);

  // Thêm publisher cho heading_angle, s_metric và kappa_s
  pub_heading_angle_ = this->create_publisher<std_msgs::msg::Float64>("/line/heading_angle", 10);
  pub_evaluation_forward_ =
      this->create_publisher<std_msgs::msg::Float64>("/line/evaluation_forward", 10);
  pub_s_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/s_metric", 10);
  pub_kappa_s_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line/kappa_s", 10);

  pub_status_ = this->create_publisher<std_msgs::msg::String>("/line/status", 10);
  pub_debug_mode_ = this->create_publisher<std_msgs::msg::String>(
      "/line/debug/mode", 10);
  pub_confidence_ = this->create_publisher<std_msgs::msg::Float64>("/line/confidence", 10);

  // RCLCPP_INFO(this->get_logger(),
  //   "ImageProcessor started | bands=%d | scan=[%.2f, %.2f] | threshold=%d",
  //   N, scan_top_ratio_, scan_bot_ratio_, white_threshold_);
}

// ── ROS 2 entry point ────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
