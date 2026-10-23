#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "image_declarations.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <cmath>

#include <algorithm>
#include <limits>

// ====== Polyfit cho metric và curvature ======
#include <Eigen/Dense>

namespace
{
int pixel_luminance_rgb(const unsigned char *pixel, int channels)
{
  if (channels < 3)
    return pixel[0];

  const int r = pixel[0];
  const int g = pixel[1];
  const int b = pixel[2];
  return static_cast<int>(0.299 * r + 0.587 * g + 0.114 * b + 0.5);
}

double binomial(int n, int k)
{
  if (k < 0 || k > n)
    return 0.0;
  double result = 1.0;
  for (int i = 1; i <= k; ++i)
    result *= static_cast<double>(n - (k - i)) / i;
  return result;
}

std::vector<double> denormalize_coefficients(
    const Eigen::VectorXd &normalized, double mean, double scale)
{
  const int degree = static_cast<int>(normalized.size()) - 1;
  std::vector<double> raw(degree + 1, 0.0);
  for (int k = 0; k <= degree; ++k)
  {
    const double factor = normalized(k) / std::pow(scale, k);
    for (int j = 0; j <= k; ++j)
    {
      raw[j] += factor * binomial(k, j) * std::pow(-mean, k - j);
    }
  }
  return raw;
}

// Fit y(x) với degree+1 hệ số, dùng QR trên x đã chuẩn hóa.
std::vector<double> poly_fit_metric(
    const std::vector<double> &x,
    const std::vector<double> &y,
    int degree)
{
  const int count = static_cast<int>(x.size());
  if (count != static_cast<int>(y.size()) || count < degree + 1)
    return {};

  double mean = 0.0;
  for (double value : x)
    mean += value;
  mean /= count;

  double scale = 0.0;
  for (double value : x)
    scale = std::max(scale, std::abs(value - mean));
  if (scale < 1e-9)
    return {};

  Eigen::MatrixXd A(count, degree + 1);
  Eigen::VectorXd values(count);
  for (int i = 0; i < count; ++i)
  {
    const double z = (x[i] - mean) / scale;
    double power = 1.0;
    for (int j = 0; j <= degree; ++j)
    {
      A(i, j) = power;
      power *= z;
    }
    values(i) = y[i];
  }

  const Eigen::VectorXd normalized =
      A.colPivHouseholderQr().solve(values);
  return denormalize_coefficients(normalized, mean, scale);
}

// Fit boundary với sigma-clipping đơn giản. Yellow mask có thể tạo một vài
// run sai do phản xạ hoặc đứt nét; least-squares thuần túy sẽ để các điểm đó
// kéo polynomial, đặc biệt ở đầu ROI nơi số observation ít.
double evaluate_polynomial(const std::vector<double> &poly, double x);

std::vector<double> robust_boundary_fit(
    const std::vector<double> &rows,
    const std::vector<double> &xs,
    int degree,
    int min_observations)
{
  if (rows.size() != xs.size() ||
      rows.size() < static_cast<size_t>(std::max(degree + 1, min_observations)))
    return {};

  auto median = [](std::vector<double> values)
  {
    if (values.empty())
      return 0.0;
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    if (values.size() % 2 == 0)
      return 0.5 * (values[middle - 1] + values[middle]);
    return values[middle];
  };

  std::vector<double> fit = poly_fit_metric(rows, xs, degree);
  if (fit.empty())
    return {};

  // Hai vòng đủ để loại một điểm nhảy lớn mà không làm pipeline nặng.
  for (int iteration = 0; iteration < 2; ++iteration)
  {
    std::vector<double> residuals;
    residuals.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i)
    {
      residuals.push_back(
          std::abs(xs[i] - evaluate_polynomial(fit, rows[i])));
    }

    const double residual_median = median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (double residual : residuals)
      deviations.push_back(std::abs(residual - residual_median));

    const double mad = median(deviations);
    const double threshold = std::max(5.0, residual_median + 2.5 * mad);

    std::vector<double> inlier_rows;
    std::vector<double> inlier_xs;
    for (size_t i = 0; i < rows.size(); ++i)
    {
      if (residuals[i] <= threshold)
      {
        inlier_rows.push_back(rows[i]);
        inlier_xs.push_back(xs[i]);
      }
    }

    if (inlier_rows.size() <
            static_cast<size_t>(std::max(degree + 1, min_observations)) ||
        inlier_rows.back() - inlier_rows.front() < 6.0)
      break;

    std::vector<double> refined =
        poly_fit_metric(inlier_rows, inlier_xs, degree);
    if (refined.empty())
      break;
    fit = refined;
  }

  return fit;
}

double evaluate_polynomial(const std::vector<double> &poly, double x)
{
  double value = 0.0;
  for (auto it = poly.rbegin(); it != poly.rend(); ++it)
    value = value * x + *it;
  return value;
}

// Tính đạo hàm bậc 1, 2 của đa thức tại x
void poly_derivatives(const std::vector<double> &poly, double x, double &d1, double &d2)
{
  d1 = 0.0;
  d2 = 0.0;
  int n = poly.size();
  for (int i = 1; i < n; ++i)
  {
    d1 += i * poly[i] * std::pow(x, i - 1);
  }
  for (int i = 2; i < n; ++i)
  {
    d2 += i * (i - 1) * poly[i] * std::pow(x, i - 2);
  }
}

// Signed curvature của lateral = f(forward). Dương là rẽ trái.
std::vector<double> compute_curvature(
    const std::vector<double> &forward,
    const std::vector<double> &lateral_poly)
{
  std::vector<double> kappa(forward.size(), 0.0);
  for (size_t i = 0; i < forward.size(); ++i)
  {
    double d1, d2;
    poly_derivatives(lateral_poly, forward[i], d1, d2);
    kappa[i] = d2 / std::pow(1.0 + d1 * d1, 1.5);
  }
  return kappa;
}

// IPM tương đương ray-ground intersection. Tia ở optical frame được đổi sang
// base_link bằng TF rồi cắt mặt phẳng z=0.
bool pixel_to_ground(
    double u,
    double v,
    const cv::Mat &K,
    const geometry_msgs::msg::TransformStamped &optical_to_ground,
    cv::Point2d &lateral_forward)
{
  const double fx = K.at<double>(0, 0);
  const double fy = K.at<double>(1, 1);
  const double cx = K.at<double>(0, 2);
  const double cy = K.at<double>(1, 2);
  if (fx <= 0.0 || fy <= 0.0)
    return false;

  tf2::Transform transform;
  tf2::fromMsg(optical_to_ground.transform, transform);

  const tf2::Vector3 ray_optical(
      (u - cx) / fx,
      (v - cy) / fy,
      1.0);
  const tf2::Vector3 ray_ground =
      transform.getBasis() * ray_optical;
  const tf2::Vector3 origin = transform.getOrigin();

  if (ray_ground.z() >= -1e-6)
    return false;
  const double distance = -origin.z() / ray_ground.z();
  if (distance <= 0.0)
    return false;

  const tf2::Vector3 hit = origin + distance * ray_ground;
  if (hit.x() <= 0.0)
    return false;

  // cv::Point2d.x = lateral (+left), .y = forward.
  lateral_forward = cv::Point2d(hit.y(), hit.x());
  return std::isfinite(hit.x()) && std::isfinite(hit.y());
}

std::vector<double> compute_arc_length_s(
    const std::vector<double> &forward,
    const std::vector<double> &lateral_poly)
{
  std::vector<double> s(forward.size(), 0.0);
  if (lateral_poly.size() < 2)
    return s;
  for (size_t i = 1; i < forward.size(); ++i)
  {
    const double y0 = forward[i - 1];
    const double y1 = forward[i];
    double dx1, ddx1, dx2, ddx2;
    poly_derivatives(lateral_poly, y0, dx1, ddx1);
    poly_derivatives(lateral_poly, y1, dx2, ddx2);
    const double ds =
        0.5 * (std::sqrt(1.0 + dx1 * dx1) +
               std::sqrt(1.0 + dx2 * dx2)) *
        std::abs(y1 - y0);
    s[i] = s[i - 1] + ds;
  }
  return s;
}
}  // namespace

double ImageProcessor::scan_row(const unsigned char *data, int row, int width, int step, int channels)
{
  int row_off = row * step;
  int left = -1, right = -1;
  for (int x = 0; x < width; x++)
  {
    int brightness = pixel_luminance_rgb(&data[row_off + x * channels], channels);
    if (brightness < black_threshold_)
    {
      if (left == -1)
        left = x;
      right = x;
    }
  }
  if (left != -1 && right != -1)
  {
    return (left + right) / 2.0;
  }
  return -1.0;
}

void ImageProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!has_camera_info_)
  {
    RCLCPP_WARN(this->get_logger(), "[IMG] Chưa nhận được CameraInfo, bỏ qua frame!");
    publish_line_health("no_line_detected", 0.0);
    return;
  }
  if (!check_valid_image(msg))
  {
    publish_line_health("no_line_detected", 0.0);
    return;
  }

  std::vector<double> lateral_offset(N, 0.0);
  std::vector<bool> valid_band_mask(N, false);
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  const int step = static_cast<int>(msg->step); // stride amount of bytes per row

  const double image_center = width / 2.0;
  int scan_top = static_cast<int>(height * scan_top_ratio_);
  int scan_bot = static_cast<int>(height * scan_bot_ratio_);
  int span = scan_bot - scan_top;
  const cv::Mat scan_mask = build_scan_mask(msg, scan_top, scan_bot);
  const unsigned char *scan_data = scan_mask.data;
  const int scan_step = static_cast<int>(scan_mask.step);
  const int scan_channels = scan_mask.channels();

  std::vector<int> scan_rows(N);
  int valid_bands = 0;
  bool any_valid = compute_offsets(scan_data, width, height, scan_step, scan_channels,
                                   image_center, scan_bot, span, lateral_offset,
                                   scan_rows, valid_band_mask, valid_bands);

  if (!any_valid || valid_bands < min_valid_bands_)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[IMG] Không đủ scan bands hợp lệ: %d/%d (cần ít nhất %d)",
        valid_bands, N, min_valid_bands_);
    publish_line_health("no_line_detected", 0.0);
    return;
  }

  const std::string source_frame =
      camera_frame_id_.empty() ? msg->header.frame_id : camera_frame_id_;
  geometry_msgs::msg::TransformStamped optical_to_ground;
  try
  {
    optical_to_ground = tf_buffer_->lookupTransform(
        ground_frame_, source_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &error)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[IMG] Chưa có TF %s <- %s: %s",
        ground_frame_.c_str(), source_frame.c_str(), error.what());
    publish_line_health("no_line_detected", 0.0);
    return;
  }

  // Centroid pixel vẫn được lấy hoàn toàn từ scan bands.
  std::vector<double> centroid_x(N);
  for (int i = 0; i < N; i++)
    centroid_x[i] = lateral_offset[i] + image_center;

  std::vector<int> valid_rows;
  std::vector<double> valid_centroids;
  std::vector<cv::Point2d> ground_points;
  for (int i = 0; i < N; ++i)
  {
    if (!valid_band_mask[i])
      continue;

    cv::Point2d ground_point;
    if (!pixel_to_ground(
            centroid_x[i], scan_rows[i], K_,
            optical_to_ground, ground_point))
    {
      valid_band_mask[i] = false;
      continue;
    }
    valid_rows.push_back(scan_rows[i]);
    valid_centroids.push_back(centroid_x[i]);
    ground_points.push_back(ground_point);
  }

  const int projected_bands = static_cast<int>(ground_points.size());
  if (projected_bands < min_valid_bands_)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[IMG] IPM chỉ chiếu được %d/%d bands", projected_bands, N);
    publish_line_health("no_line_detected", 0.0);
    return;
  }

  // Pixel polynomial chỉ dùng để vẽ fitting_image; degree=3 là cubic.
  const std::vector<double> pixel_poly =
      poly_fit(valid_rows, valid_centroids, 3);

  // Sắp điểm metric theo forward gần -> xa để s[0] luôn là điểm gần xe.
  std::sort(
      ground_points.begin(), ground_points.end(),
      [](const cv::Point2d &a, const cv::Point2d &b)
      {
        return a.y < b.y;
      });

  std::vector<double> forward;
  std::vector<double> path_lateral;
  forward.reserve(ground_points.size());
  path_lateral.reserve(ground_points.size());
  for (const cv::Point2d &point : ground_points)
  {
    path_lateral.push_back(point.x);
    forward.push_back(point.y);
  }

  // Cubic metric fit: path_lateral = f(forward).
  const std::vector<double> lateral_poly =
      poly_fit_metric(forward, path_lateral, 3);
  if (lateral_poly.size() != 4 || pixel_poly.size() != 4)
  {
    publish_line_health("no_line_detected", 0.0);
    return;
  }

  const double evaluation_forward =
      cte_evaluation_forward_ < 0.0
          ? forward.front()
          : std::max(
                forward.front(),
                std::min(forward.back(), cte_evaluation_forward_));
  const double path_at_evaluation =
      evaluate_polynomial(lateral_poly, evaluation_forward);
  double path_heading_slope = 0.0;
  double unused_second_derivative = 0.0;
  poly_derivatives(
      lateral_poly, evaluation_forward,
      path_heading_slope, unused_second_derivative);

  // NMPC state uses vehicle-relative errors: n and alpha are opposite to
  // the path position/heading observed in the vehicle frame.
  const double cte = -path_at_evaluation;
  const double heading_angle = -std::atan(path_heading_slope);
  const std::vector<double> s_metric =
      compute_arc_length_s(forward, lateral_poly);
  const std::vector<double> kappa_s =
      compute_curvature(forward, lateral_poly);

  publish_line_health(
      "line_detected", static_cast<double>(projected_bands) / N);

  std_msgs::msg::Float64MultiArray ref_msg;
  ref_msg.data = path_lateral;
  pub_ref_path_->publish(ref_msg);

  std_msgs::msg::Float64 cte_msg;
  cte_msg.data = cte;
  pub_cte_->publish(cte_msg);

  std_msgs::msg::Float64MultiArray poly_msg;
  poly_msg.data = pixel_poly;
  pub_poly_coeffs_->publish(poly_msg);

  std_msgs::msg::Float64 heading_angle_msg;
  heading_angle_msg.data = heading_angle;
  pub_heading_angle_->publish(heading_angle_msg);

  std_msgs::msg::Float64 evaluation_forward_msg;
  evaluation_forward_msg.data = evaluation_forward;
  pub_evaluation_forward_->publish(evaluation_forward_msg);

  if (!kappa_s.empty())
  {
    std_msgs::msg::Float64 kappa_msg;
    kappa_msg.data = kappa_s.front();
    pub_kappa_->publish(kappa_msg);
  }

  std_msgs::msg::Float64MultiArray s_msg;
  s_msg.data = s_metric;
  std_msgs::msg::Float64MultiArray kappa_s_msg;
  kappa_s_msg.data = kappa_s;
  pub_s_->publish(s_msg);
  pub_kappa_s_->publish(kappa_s_msg);

  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[IMG] bands=%d/%d | CTE=%.4f m | heading=%.2f deg | "
      "eval=%.2f m | range=%.2f..%.2f m | kappa0=%.4f 1/m",
      projected_bands, N, cte, heading_angle * 180.0 / M_PI,
      evaluation_forward, forward.front(), forward.back(),
      kappa_s.empty() ? 0.0 : kappa_s.front());

  publish_debug_image(
      msg, scan_mask, scan_rows, lateral_offset, valid_band_mask, image_center);
  publish_fitting_image(
      msg, scan_rows, centroid_x, valid_band_mask,
      pixel_poly, scan_top, scan_bot);
}

// CameraInfo callback
void ImageProcessor::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // Lấy mảng 9 số thực từ trường k
  const double *k = msg->k.data();
  K_ = (cv::Mat_<double>(3, 3) << k[0], k[1], k[2],
        k[3], k[4], k[5],
        k[6], k[7], k[8]);
  camera_frame_id_ = msg->header.frame_id;
  has_camera_info_ = true;
  RCLCPP_INFO_ONCE(
      this->get_logger(),
      "[IMG] CameraInfo frame=%s fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      camera_frame_id_.c_str(), k[0], k[4], k[2], k[5]);
}

bool ImageProcessor::compute_offsets(
    const unsigned char *data, int width, int height, int step, int channels,
    double image_center, int scan_bot, int span,
    std::vector<double> &lateral_offset, std::vector<int> &scan_rows,
    std::vector<bool> &valid_bands,
    int &out_valid_bands)
{
  bool any_valid = false;
  out_valid_bands = 0;
  for (int i = 0; i < N; i++)
  {
    double ratio = static_cast<double>(N - 1 - i) / (N - 1);
    int center = scan_bot - static_cast<int>(ratio * span);
    scan_rows[i] = std::max(0, std::min(height - 1, center));

    double sum_mid = 0;
    int cnt = 0;
    for (int r = center - band_height_ / 2; r <= center + band_height_ / 2; r++)
    {
      if (r < 0 || r >= height)
        continue;
      double midpoint = scan_row(data, r, width, step, channels);
      if (midpoint >= 0)
      {
        sum_mid += midpoint;
        cnt++;
      }
    }
    if (cnt > 0)
    {
      lateral_offset[i] = sum_mid / cnt - image_center;
      valid_bands[i] = true;
      any_valid = true;
      out_valid_bands++;
    }
    else
    {
      lateral_offset[i] = 0.0;
      valid_bands[i] = false;
    }
  }
  return any_valid;
}

bool ImageProcessor::check_valid_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->encoding != "rgb8" && msg->encoding != "bgr8")
  {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("ImageProcessor"),
                     "Encoding '%s' chưa được hỗ trợ (cần rgb8/bgr8)", msg->encoding.c_str());
    return false;
  }
  return true;
}

cv::Mat ImageProcessor::build_scan_mask(
    const sensor_msgs::msg::Image::SharedPtr msg,
    int scan_top,
    int scan_bot)
{
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  const int step = static_cast<int>(msg->step);
  cv::Mat scan_mask(height, width, CV_8UC1, cv::Scalar(255));

  if (width <= 0 || height <= 0 || step <= 0)
    return scan_mask;

  scan_top = std::max(0, std::min(height - 1, scan_top));
  scan_bot = std::max(scan_top + 1, std::min(height, scan_bot));
  const cv::Rect roi_rect(0, scan_top, width, scan_bot - scan_top);

  cv::Mat raw(
      height, width, CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()),
      static_cast<size_t>(step));
  cv::Mat hsv;
  if (msg->encoding == "rgb8")
    cv::cvtColor(raw, hsv, cv::COLOR_RGB2HSV);
  else
    cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv, hsv_channels);

  cv::Mat value_blur;
  cv::GaussianBlur(hsv_channels[2], value_blur, cv::Size(5, 5), 0.0);

  cv::Mat yellow_mask;
  cv::inRange(
      hsv,
      cv::Scalar(15, 45, 45),
      cv::Scalar(45, 255, 255),
      yellow_mask);
  const cv::Mat yellow_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, yellow_kernel);

  cv::Mat dark_roi;
  cv::threshold(
      value_blur(roi_rect), dark_roi,
      0.0, 255.0,
      cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  dark_roi.setTo(0, yellow_mask(roi_rect));

  const cv::Mat close_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 3));
  const cv::Mat open_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(dark_roi, dark_roi, cv::MORPH_CLOSE, close_kernel);
  cv::morphologyEx(dark_roi, dark_roi, cv::MORPH_OPEN, open_kernel);
  const cv::Mat dark_mask_before_corridor = dark_roi.clone();

  int scan_left = static_cast<int>(std::round(width * scan_left_ratio_));
  int scan_right = static_cast<int>(std::round(width * scan_right_ratio_));
  scan_left = std::max(0, std::min(width - 1, scan_left));
  scan_right = std::max(scan_left + 1, std::min(width, scan_right));

  const double search_left_ratio = std::max(
      0.0, std::min(1.0, boundary_search_left_ratio_));
  const double search_right_ratio = std::max(
      search_left_ratio + 1.0 / std::max(1, width),
      std::min(1.0, boundary_search_right_ratio_));
  const int boundary_left = std::max(
      0, std::min(width - 1,
                  static_cast<int>(std::round(width * search_left_ratio))));
  const int boundary_right = std::max(
      boundary_left + 1,
      std::min(width, static_cast<int>(std::round(width * search_right_ratio))));

  const cv::Mat yellow_roi = yellow_mask(roi_rect);
  const int boundary_margin_px = std::max(0, boundary_margin_px_);
  const int min_boundary_gap_px = std::max(40, width / 8);
  std::vector<int> row_left_boundary(dark_roi.rows, -1);
  std::vector<int> row_right_boundary(dark_roi.rows, -1);
  std::vector<double> left_observation_rows;
  std::vector<double> left_boundary_x;
  std::vector<double> right_observation_rows;
  std::vector<double> right_boundary_x;

  auto append_boundary_observation = [](
      int row, int x, std::vector<double> &rows, std::vector<double> &xs)
  {
    rows.push_back(static_cast<double>(row));
    xs.push_back(static_cast<double>(x));
  };

  for (int y = 0; y < dark_roi.rows; ++y)
  {
    const unsigned char *yellow_row = yellow_roi.ptr<unsigned char>(y);
    std::vector<std::pair<int, int>> yellow_runs;
    int run_left = -1;
    for (int x = boundary_left; x < boundary_right; ++x)
    {
      if (yellow_row[x] != 0)
      {
        if (run_left < 0)
          run_left = x;
      }
      else if (run_left >= 0)
      {
        yellow_runs.emplace_back(run_left, x - 1);
        run_left = -1;
      }
    }
    if (run_left >= 0)
      yellow_runs.emplace_back(run_left, boundary_right - 1);

    if (yellow_runs.empty())
      continue;

    const int first_center =
        (yellow_runs.front().first + yellow_runs.front().second) / 2;
    const int last_center =
        (yellow_runs.back().first + yellow_runs.back().second) / 2;

    if (yellow_runs.size() >= 2 &&
        last_center - first_center >= min_boundary_gap_px)
    {
      row_left_boundary[y] = first_center;
      row_right_boundary[y] = last_center;
      append_boundary_observation(
          y, first_center, left_observation_rows, left_boundary_x);
      append_boundary_observation(
          y, last_center, right_observation_rows, right_boundary_x);
      continue;
    }

    // Một biên có thể chỉ còn xuất hiện ở một phần scan ROI. Không bỏ qua
    // observation đơn này: dùng prior của frame trước để phân loại trái/phải;
    // nếu chưa có prior thì dùng tâm ảnh chỉ để thu thập điểm, chưa đủ điều
    // kiện tự tạo corridor.
    if (yellow_runs.size() == 1)
    {
      const int center = first_center;
      double previous_left = width * 0.25;
      double previous_right = width * 0.75;
      if (previous_boundary_corridor_valid_ &&
          !previous_left_boundary_poly_.empty() &&
          !previous_right_boundary_poly_.empty())
      {
        previous_left = evaluate_polynomial(
            previous_left_boundary_poly_, y);
        previous_right = evaluate_polynomial(
            previous_right_boundary_poly_, y);
      }
      const double previous_midpoint =
          0.5 * (previous_left + previous_right);
      if (std::abs(center - previous_left) <=
          std::abs(center - previous_right))
      {
        append_boundary_observation(
            y, center, left_observation_rows, left_boundary_x);
      }
      else if (center >= previous_midpoint || center >= width / 2)
      {
        append_boundary_observation(
            y, center, right_observation_rows, right_boundary_x);
      }
      else
      {
        append_boundary_observation(
            y, center, left_observation_rows, left_boundary_x);
      }
    }
  }

  const int min_boundary_observations = std::max(
      3, boundary_recovery_min_observations_);
  auto fit_boundary = [&](const std::vector<double> &rows,
                          const std::vector<double> &xs)
      -> std::vector<double>
  {
    if (rows.size() < static_cast<size_t>(min_boundary_observations))
      return {};
    const int degree = rows.size() >= 5 ? 2 : 1;
    return robust_boundary_fit(
        rows, xs, degree, min_boundary_observations);
  };

  const std::vector<double> left_boundary_poly =
      fit_boundary(left_observation_rows, left_boundary_x);
  const std::vector<double> right_boundary_poly =
      fit_boundary(right_observation_rows, right_boundary_x);

  // Khi cả hai biên vàng đã được fit đáng tin cậy, hình học corridor là
  // thông tin mạnh hơn độ sáng của mặt đường. Đặc biệt, một nửa mặt đường bị
  // chá sáng vẫn phải được xem là lane hợp lệ. Tạo mask đen tổng hợp cho toàn
  // bộ corridor để scan_row lấy midpoint giữa hai biên thay vì phụ thuộc vào
  // Otsu toàn ROI.
  bool use_boundary_corridor = false;
  std::string corridor_mode = "OTSU_FALLBACK";
  cv::Mat boundary_corridor;
  auto build_boundary_corridor = [&](const std::vector<double> &left_poly,
                                    const std::vector<double> &right_poly,
                                    cv::Mat &corridor)
      -> bool
  {
    if (left_poly.empty() || right_poly.empty())
      return false;

    corridor = cv::Mat::zeros(dark_roi.size(), CV_8UC1);
    int valid_corridor_rows = 0;
    for (int y = 0; y < dark_roi.rows; ++y)
    {
      const double left_value = evaluate_polynomial(left_poly, y);
      const double right_value = evaluate_polynomial(right_poly, y);
      if (!std::isfinite(left_value) || !std::isfinite(right_value))
        continue;
      // Không clamp một polynomial extrapolation sai vào hai mép ảnh. Nếu
      // làm vậy, một hàng có thể biến thành corridor gần như toàn chiều rộng
      // và scan_row sẽ lấy pixel đen ngoài lane.
      if (left_value < boundary_left || right_value > boundary_right ||
          right_value <= left_value)
        continue;

      int inner_left = static_cast<int>(std::round(left_value)) +
          boundary_margin_px;
      int inner_right = static_cast<int>(std::round(right_value)) -
          boundary_margin_px;

      inner_left = std::max(
          boundary_left,
          std::min(boundary_right - 1, inner_left));
      inner_right = std::max(
          boundary_left + 1,
          std::min(boundary_right, inner_right));
      if (inner_right <= inner_left ||
          inner_right - inner_left < min_boundary_gap_px)
        continue;

      corridor.row(y)
          .colRange(inner_left, inner_right)
          .setTo(255);
      ++valid_corridor_rows;
    }
    return valid_corridor_rows >= min_valid_bands_;
  };

  if (boundary_corridor_enabled_)
  {
    // Trường hợp bình thường: cả hai biên còn đủ điểm trong frame hiện tại.
    if (build_boundary_corridor(
            left_boundary_poly, right_boundary_poly, boundary_corridor))
    {
      use_boundary_corridor = true;
      corridor_mode = "FULL_CORRIDOR";
      previous_boundary_corridor_valid_ = true;
      previous_left_boundary_poly_ = left_boundary_poly;
      previous_right_boundary_poly_ = right_boundary_poly;
      previous_boundary_stamp_ = this->get_clock()->now();
    }
    else if (boundary_recovery_enabled_ &&
             previous_boundary_corridor_valid_ &&
             !previous_left_boundary_poly_.empty() &&
             !previous_right_boundary_poly_.empty())
    {
      const double age_ms =
          (this->get_clock()->now() - previous_boundary_stamp_).seconds() *
          1000.0;
      const bool previous_is_fresh =
          age_ms >= 0.0 && age_ms <= std::max(0, boundary_recovery_max_age_ms_);

      if (previous_is_fresh && !left_boundary_poly.empty() &&
          right_boundary_poly.empty())
      {
        // Biên phải bị mất: giữ độ rộng corridor của frame tốt trước đó.
        cv::Mat recovered;
        recovered = cv::Mat::zeros(dark_roi.size(), CV_8UC1);
        int valid_rows = 0;
        for (int y = 0; y < dark_roi.rows; ++y)
        {
          const double left_value =
              evaluate_polynomial(left_boundary_poly, y);
          const double previous_width =
              evaluate_polynomial(previous_right_boundary_poly_, y) -
              evaluate_polynomial(previous_left_boundary_poly_, y);
          if (!std::isfinite(left_value) || !std::isfinite(previous_width))
            continue;
          const int left = static_cast<int>(std::round(left_value));
          const int right = static_cast<int>(std::round(left_value + previous_width));
          if (left < boundary_left || right > boundary_right || right <= left)
            continue;
          int inner_left = left + boundary_margin_px;
          int inner_right = right - boundary_margin_px;
          inner_left = std::max(
              boundary_left, std::min(boundary_right - 1, inner_left));
          inner_right = std::max(
              boundary_left + 1, std::min(boundary_right, inner_right));
          if (inner_right <= inner_left ||
              inner_right - inner_left < min_boundary_gap_px)
            continue;
          recovered.row(y).colRange(inner_left, inner_right).setTo(255);
          ++valid_rows;
        }
        if (valid_rows >= min_valid_bands_)
        {
          boundary_corridor = recovered;
          use_boundary_corridor = true;
          corridor_mode = "RECOVER_RIGHT";
        }
      }
      else if (previous_is_fresh && left_boundary_poly.empty() &&
               !right_boundary_poly.empty())
      {
        // Biên trái bị mất: giữ độ rộng corridor của frame tốt trước đó.
        cv::Mat recovered = cv::Mat::zeros(dark_roi.size(), CV_8UC1);
        int valid_rows = 0;
        for (int y = 0; y < dark_roi.rows; ++y)
        {
          const double right_value =
              evaluate_polynomial(right_boundary_poly, y);
          const double previous_width =
              evaluate_polynomial(previous_right_boundary_poly_, y) -
              evaluate_polynomial(previous_left_boundary_poly_, y);
          if (!std::isfinite(right_value) || !std::isfinite(previous_width))
            continue;
          const int right = static_cast<int>(std::round(right_value));
          const int left = static_cast<int>(std::round(right_value - previous_width));
          if (left < boundary_left || right > boundary_right || right <= left)
            continue;
          int inner_left = left + boundary_margin_px;
          int inner_right = right - boundary_margin_px;
          inner_left = std::max(
              boundary_left, std::min(boundary_right - 1, inner_left));
          inner_right = std::max(
              boundary_left + 1, std::min(boundary_right, inner_right));
          if (inner_right <= inner_left ||
              inner_right - inner_left < min_boundary_gap_px)
            continue;
          recovered.row(y).colRange(inner_left, inner_right).setTo(255);
          ++valid_rows;
        }
        if (valid_rows >= min_valid_bands_)
        {
          boundary_corridor = recovered;
          use_boundary_corridor = true;
          corridor_mode = "RECOVER_LEFT";
        }
      }
    }
  }

  if (use_boundary_corridor)
  {
    // dark_roi=255 nghĩa là pixel được đưa vào scan mask đen ở cuối hàm.
    dark_roi = boundary_corridor;
  }
  else
  {
    // Fallback tương thích: chỉ dùng threshold Otsu khi chưa đủ hai biên để
    // xác định corridor. Không tự fabricate lane khi hình học không đủ tin cậy.
    for (int y = 0; y < dark_roi.rows; ++y)
    {
      const int corridor_left_limit = scan_left;
      const int corridor_right_limit = scan_right;
      int inner_left = corridor_left_limit;
      int inner_right = corridor_right_limit;

      if (row_left_boundary[y] >= 0 &&
          row_right_boundary[y] > row_left_boundary[y])
      {
        inner_left = row_left_boundary[y] + boundary_margin_px;
        inner_right = row_right_boundary[y] - boundary_margin_px;
      }

      inner_left = std::max(
          corridor_left_limit,
          std::min(corridor_right_limit - 1, inner_left));
      inner_right = std::max(
          corridor_left_limit + 1,
          std::min(corridor_right_limit, inner_right));
      if (inner_right <= inner_left)
        continue;

      unsigned char *dark_row = dark_roi.ptr<unsigned char>(y);
      std::fill(dark_row, dark_row + inner_left, 0);
      std::fill(dark_row + inner_right, dark_row + width, 0);

      int first_dark = -1;
      int last_dark = -1;
      for (int x = inner_left; x < inner_right; ++x)
      {
        if (dark_row[x] == 0)
          continue;
        if (first_dark < 0)
          first_dark = x;
        last_dark = x;
      }
      if (first_dark >= 0 && last_dark > first_dark)
        std::fill(dark_row + first_dark, dark_row + last_dark + 1, 255);
    }
  }

  cv::Mat mask_roi = scan_mask(roi_rect);
  mask_roi.setTo(255);
  mask_roi.setTo(0, dark_roi);

  // Publish các stage trung gian để web viewer cho biết lỗi xuất hiện ở bước
  // màu, threshold, corridor hay scan mask. Các mask mono8 giữ nguyên kích
  // thước ảnh để dễ đối chiếu cùng một tọa độ pixel.
  auto publish_mono = [&](const cv::Mat &image,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher)
  {
    if (image.empty() || image.type() != CV_8UC1 || !publisher)
      return;
    sensor_msgs::msg::Image output;
    output.header = msg->header;
    output.height = static_cast<uint32_t>(image.rows);
    output.width = static_cast<uint32_t>(image.cols);
    output.encoding = "mono8";
    output.is_bigendian = false;
    output.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols);
    output.data.assign(image.datastart, image.dataend);
    publisher->publish(output);
  };

  auto publish_color = [&](const cv::Mat &image,
                           const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher)
  {
    if (image.empty() || image.type() != CV_8UC3 || !publisher)
      return;
    sensor_msgs::msg::Image output;
    output.header = msg->header;
    output.height = static_cast<uint32_t>(image.rows);
    output.width = static_cast<uint32_t>(image.cols);
    output.encoding = "bgr8";
    output.is_bigendian = false;
    output.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
    output.data.assign(image.datastart, image.dataend);
    publisher->publish(output);
  };

  cv::Mat dark_full = cv::Mat::zeros(height, width, CV_8UC1);
  dark_mask_before_corridor.copyTo(dark_full(roi_rect));
  cv::Mat corridor_full = cv::Mat::zeros(height, width, CV_8UC1);
  if (use_boundary_corridor)
    boundary_corridor.copyTo(corridor_full(roi_rect));
  else
    dark_roi.copyTo(corridor_full(roi_rect));

  cv::Mat boundary_overlay;
  if (msg->encoding == "rgb8")
    cv::cvtColor(raw, boundary_overlay, cv::COLOR_RGB2BGR);
  else
    boundary_overlay = raw.clone();

  auto draw_polyline = [&](const std::vector<double> &poly,
                           const cv::Scalar &color)
  {
    if (poly.empty())
      return;
    std::vector<cv::Point> points;
    for (int y = 0; y < dark_roi.rows; ++y)
    {
      const double value = evaluate_polynomial(poly, y);
      if (!std::isfinite(value))
        continue;
      const int x = static_cast<int>(std::round(value));
      if (x < 0 || x >= width)
        continue;
      points.emplace_back(x, scan_top + y);
    }
    if (points.size() >= 2)
      cv::polylines(boundary_overlay, points, false, color, 2, cv::LINE_AA);
  };

  // Các điểm vàng quan sát được: xanh lá = observation hiện tại.
  for (size_t i = 0; i < left_observation_rows.size(); ++i)
  {
    cv::circle(
        boundary_overlay,
        cv::Point(
            static_cast<int>(std::round(left_boundary_x[i])),
            scan_top + static_cast<int>(std::round(left_observation_rows[i]))),
        3, cv::Scalar(0, 255, 0), -1);
  }
  for (size_t i = 0; i < right_observation_rows.size(); ++i)
  {
    cv::circle(
        boundary_overlay,
        cv::Point(
            static_cast<int>(std::round(right_boundary_x[i])),
            scan_top + static_cast<int>(std::round(right_observation_rows[i]))),
        3, cv::Scalar(0, 255, 0), -1);
  }
  draw_polyline(left_boundary_poly, cv::Scalar(0, 255, 0));
  draw_polyline(right_boundary_poly, cv::Scalar(0, 255, 0));

  // Xanh dương = mép corridor cuối cùng mà scan mask sử dụng. Điều này cũng
  // hiển thị được mép suy ra trong nhánh RECOVER_LEFT/RECOVER_RIGHT.
  if (use_boundary_corridor)
  {
    std::vector<cv::Point> left_edge;
    std::vector<cv::Point> right_edge;
    for (int y = 0; y < boundary_corridor.rows; ++y)
    {
      const unsigned char *row = boundary_corridor.ptr<unsigned char>(y);
      int first = -1;
      int last = -1;
      for (int x = 0; x < boundary_corridor.cols; ++x)
      {
        if (row[x] == 0)
          continue;
        if (first < 0)
          first = x;
        last = x;
      }
      if (first >= 0 && last > first)
      {
        left_edge.emplace_back(first, scan_top + y);
        right_edge.emplace_back(last, scan_top + y);
      }
    }
    if (left_edge.size() >= 2)
      cv::polylines(
          boundary_overlay, left_edge, false, cv::Scalar(255, 0, 0), 2,
          cv::LINE_AA);
    if (right_edge.size() >= 2)
      cv::polylines(
          boundary_overlay, right_edge, false, cv::Scalar(255, 0, 0), 2,
          cv::LINE_AA);
  }

  cv::putText(
      boundary_overlay, "mode=" + corridor_mode, cv::Point(10, 28),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

  publish_mono(yellow_mask, pub_debug_yellow_);
  publish_mono(dark_full, pub_debug_dark_);
  publish_mono(corridor_full, pub_debug_corridor_);
  publish_mono(scan_mask, pub_debug_scan_mask_);
  publish_color(boundary_overlay, pub_debug_boundary_);

  std_msgs::msg::String mode_msg;
  mode_msg.data = corridor_mode;
  pub_debug_mode_->publish(mode_msg);
  return scan_mask;
}

void ImageProcessor::publish_line_health(const std::string &status, double confidence)
{
  double c = std::max(0.0, std::min(1.0, confidence));
  std_msgs::msg::String s;
  s.data = status;
  pub_status_->publish(s);
  std_msgs::msg::Float64 cf;
  cf.data = c;
  pub_confidence_->publish(cf);
}

int calculate_span(int height, double top_ratio, double bot_ratio)
{
  int scan_top = static_cast<int>(height * top_ratio);
  int scan_bot = static_cast<int>(height * bot_ratio);
  return scan_bot - scan_top;
}

// ── Private method: publish_debug_image ───────────────────────────────────
void ImageProcessor::publish_debug_image(
    const sensor_msgs::msg::Image::SharedPtr src,
    const cv::Mat &scan_mask,
    const std::vector<int> &rows,
    const std::vector<double> &offsets,
    const std::vector<bool> &valid_bands,
    double image_center)
{
  auto debug = clone_image(src);

  const int width = static_cast<int>(src->width);
  const int step = static_cast<int>(src->step);
  const int channels = step / width;
  unsigned char *d = debug->data.data();

  draw_all_bands(
      d, scan_mask, src, step, channels, width,
      rows, offsets, valid_bands, image_center);
  pub_debug_->publish(*debug);
}

void ImageProcessor::draw_all_bands(
    unsigned char *d,
    const cv::Mat &scan_mask,
    const sensor_msgs::msg::Image::SharedPtr src,
    int step, int channels, int width,
    const std::vector<int> &rows,
    const std::vector<double> &offsets,
    const std::vector<bool> &valid_bands,
    double image_center)
{
  const uint8_t color[3] = {0, 255, 0}; // xanh lá
  for (int i = 0; i < N; i++)
  {
    if (!valid_bands[i])
      continue;
    int row_center = rows[i];
    int centroid_x = static_cast<int>(offsets[i] + image_center);

    // Tô 1 dải ngang 3 px xung quanh scan line
    for (int r = row_center - 1; r <= row_center + 1; r++)
    {
      if (r < 0 || r >= static_cast<int>(src->height))
        continue;
      int row_off = r * step;
      for (int x = 0; x < width; x++)
      {
        int px = row_off + x * channels;
        int brightness = scan_mask.at<unsigned char>(r, x);
        if (brightness < black_threshold_)
        {
          d[px] = color[0];
          d[px + 1] = color[1];
          d[px + 2] = color[2];
        }
      }
    }
    // Đánh dấu centroid (midpoint) bằng chấm trắng 5x5
    for (int dr = -2; dr <= 2; dr++)
      for (int dc = -2; dc <= 2; dc++)
      {
        int r = row_center + dr;
        int x = centroid_x + dc;
        if (r < 0 || r >= static_cast<int>(src->height))
          continue;
        if (x < 0 || x >= width)
          continue;
        int px = r * step + x * channels;
        d[px] = 0;
        d[px + 1] = 0;
        d[px + 2] = 0;
      }
  }
}

sensor_msgs::msg::Image::SharedPtr clone_image(const sensor_msgs::msg::Image::SharedPtr src)
{
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->header = src->header;
  img->width = src->width;
  img->height = src->height;
  img->step = src->step;
  img->encoding = src->encoding;
  img->data = src->data;
  return img;
}
