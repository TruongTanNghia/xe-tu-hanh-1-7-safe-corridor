#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

using std::placeholders::_1;

class CameraSubscriber : public rclcpp::Node {
public:
    CameraSubscriber() : Node("camera_subscriber") {
        cv::namedWindow("Camera | Debug 2x2", cv::WINDOW_NORMAL);
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/front_camera/image_raw", 10, std::bind(&CameraSubscriber::camera_callback, this, _1)
        );
        sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/line/debug_image", 10, std::bind(&CameraSubscriber::debug_callback, this, _1)
        );
        sub_fitting_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/line/fitting_image", 10, std::bind(&CameraSubscriber::fitting_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "CameraSubscriber node started | ROI scan band: %.0f%%-%.0f%%",
                     roi_top_ * 100.0, roi_bot_ * 100.0);
    }

private:
    cv::Mat last_camera_image_;
    cv::Mat last_debug_image_;
    cv::Mat last_fitting_image_;

    // ROI scan band (same as image_processor scan_top_ratio / scan_bot_ratio)
    double roi_top_ = 0.50;
    double roi_bot_ = 0.90;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_camera_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (camera): %s", e.what());
        }
    }

    void debug_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_debug_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (debug): %s", e.what());
        }
    }

    void fitting_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_fitting_image_ = cv_ptr->image.clone();
            show_combined();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (fitting): %s", e.what());
        }
    }

    cv::Mat crop_roi(const cv::Mat& src) {
        int y_top = static_cast<int>(src.rows * roi_top_);
        int y_bot = static_cast<int>(src.rows * roi_bot_);
        cv::Mat cropped = src(cv::Rect(0, y_top, src.cols, y_bot - y_top)).clone();
        return cropped;
    }

    void show_combined() {
        const int W = 320, H = 240;
        cv::Mat panel1, panel2, panel3, panel4;

        // Panel 1: Raw Camera
        if (!last_camera_image_.empty()) {
            cv::resize(last_camera_image_, panel1, cv::Size(W, H));
        } else {
            panel1 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel1, "Waiting camera...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,0), 1);
        }

        // Panel 2: ROI Crop (scan band region from raw camera)
        if (!last_camera_image_.empty()) {
            cv::Mat roi = crop_roi(last_camera_image_);
            cv::resize(roi, panel2, cv::Size(W, H));
        } else {
            panel2 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel2, "Waiting ROI...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,0), 1);
        }

        // Panel 3: Debug (scan) — was old Panel 2
        if (!last_debug_image_.empty()) {
            cv::resize(last_debug_image_, panel3, cv::Size(W, H));
            if (panel3.channels() == 1) cv::cvtColor(panel3, panel3, cv::COLOR_GRAY2BGR);
        } else {
            panel3 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel3, "Waiting debug_image...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
        }

        // Panel 4: Poly Fitting — was old Panel 3
        if (!last_fitting_image_.empty()) {
            cv::resize(last_fitting_image_, panel4, cv::Size(W, H));
            if (panel4.channels() == 1) cv::cvtColor(panel4, panel4, cv::COLOR_GRAY2BGR);
        } else {
            panel4 = cv::Mat(H, W, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(panel4, "Waiting fitting_image...", cv::Point(5, H/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
        }

        // Labels
        cv::putText(panel1, "Raw Camera",       cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
        cv::putText(panel2, "ROI Crop (scan)",   cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,128,0), 1);
        cv::putText(panel3, "Debug (scan)",      cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0),   1);
        cv::putText(panel4, "Poly Fitting",      cv::Point(5, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,140,255), 1);

        // Ghép 2x2: 640 x 480
        cv::Mat top, bot, combined;
        cv::hconcat(panel1, panel2, top);
        cv::hconcat(panel3, panel4, bot);
        cv::vconcat(top, bot, combined);
        cv::imshow("Camera | Debug 2x2", combined);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_debug_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fitting_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
