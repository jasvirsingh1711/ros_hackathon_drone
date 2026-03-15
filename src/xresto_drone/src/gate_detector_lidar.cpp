#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class GateDetectorLidar : public rclcpp::Node {
public:
    GateDetectorLidar() : Node("gate_detector") {
        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/drone/front_camera/image_raw", 10,
            std::bind(&GateDetectorLidar::rgb_callback, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/drone/front_camera/depth/image_raw", 10,
            std::bind(&GateDetectorLidar::depth_callback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/drone/altitude", 10,
            std::bind(&GateDetectorLidar::lidar_callback, this, std::placeholders::_1));
        
        // Publishers
        gate_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drone/gate_detection", 10);
        
        RCLCPP_INFO(this->get_logger(), "Gate Detector with LiDAR + Color Analysis initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gate_pub_;
    
    cv::Mat latest_depth_;
    cv::Mat latest_rgb_;
    float latest_lidar_distance_ = 10.0;
    bool has_rgb_ = false;
    bool has_depth_ = false;
    
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            latest_depth_ = cv_ptr->image.clone();
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth exception: %s", e.what());
        }
    }
    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.size() > 0 && !std::isnan(msg->ranges[0]) && !std::isinf(msg->ranges[0])) {
            latest_lidar_distance_ = msg->ranges[0];
        }
    }
    
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            latest_rgb_ = cv_ptr->image.clone();
            has_rgb_ = true;
            
            int height = latest_rgb_.rows;
            int width = latest_rgb_.cols;
            
            if (!has_depth_ || latest_depth_.empty()) {
                return;
            }
            
            // Analyze center region of depth image to detect gate opening
            int center_x = width / 2;
            int center_y = height / 2;
            int roi_size = 80;  // Region of interest size
            
            int roi_x_min = std::max(0, center_x - roi_size);
            int roi_x_max = std::min(width, center_x + roi_size);
            int roi_y_min = std::max(0, center_y - roi_size);
            int roi_y_max = std::min(height, center_y + roi_size);
            
            cv::Rect roi_rect(roi_x_min, roi_y_min, roi_x_max - roi_x_min, roi_y_max - roi_y_min);
            cv::Mat depth_roi = latest_depth_(roi_rect);
            cv::Mat rgb_roi = latest_rgb_(roi_rect);
            
            // Compute statistics on depth in ROI
            double min_depth, max_depth, mean_depth = 10.0;
            cv::minMaxLoc(depth_roi, &min_depth, &max_depth);
            
            // Count valid pixels
            int valid_pixels = 0;
            double sum_depth = 0;
            for (int y = 0; y < depth_roi.rows; y++) {
                for (int x = 0; x < depth_roi.cols; x++) {
                    float d = depth_roi.at<float>(y, x);
                    if (!std::isnan(d) && !std::isinf(d) && d > 0.1) {
                        sum_depth += d;
                        valid_pixels++;
                    }
                }
            }
            if (valid_pixels > 0) {
                mean_depth = sum_depth / valid_pixels;
            }
            
            // Detect gate: gate opening appears as lower depth (hole) compared to surroundings
            float background_depth = latest_lidar_distance_;
            float depth_threshold = background_depth * 0.7;  // Gate is < 70% of background distance
            
            // Find dark pixels (gate interior) vs bright pixels (frame)
            cv::Mat hsv;
            cv::cvtColor(rgb_roi, hsv, cv::COLOR_BGR2HSV);
            
            // Look for dark areas (low V in HSV) - gate interior tends to be darker
            cv::Mat dark_mask;
            cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 100), dark_mask);
            
            // Combine: gate opening has BOTH low depth AND dark color
            cv::Mat gate_candidate;
            for (int y = 0; y < depth_roi.rows; y++) {
                for (int x = 0; x < depth_roi.cols; x++) {
                    float d = depth_roi.at<float>(y, x);
                    uchar dark = dark_mask.at<uchar>(y, x);
                    
                    if ((std::isnan(d) || std::isinf(d) || d > depth_threshold) && dark > 100) {
                        // Potential gate opening
                        if (gate_candidate.empty()) {
                            gate_candidate.create(depth_roi.size(), CV_8UC1);
                            gate_candidate.setTo(0);
                        }
                        gate_candidate.at<uchar>(y, x) = 255;
                    }
                }
            }
            
            // Find contours of potential gates
            if (!gate_candidate.empty()) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(gate_candidate.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
                // Find largest contour (likely the gate)
                double max_area = 0;
                std::vector<cv::Point> best_contour;
                for (const auto& contour : contours) {
                    double area = cv::contourArea(contour);
                    if (area > max_area && area > 100) {  // Minimum 100 pixels
                        max_area = area;
                        best_contour = contour;
                    }
                }
                
                if (!best_contour.empty()) {
                    cv::Moments m = cv::moments(best_contour);
                    int gate_center_x_roi = m.m10 / m.m00;
                    int gate_center_y_roi = m.m01 / m.m00;
                    
                    // Convert back to full image coordinates
                    int gate_center_x = gate_center_x_roi + roi_x_min;
                    int gate_center_y = gate_center_y_roi + roi_y_min;
                    
                    // Publish gate detection: [center_x_norm, center_y_norm, distance_to_gate]
                    // Normalize to -1 to 1 range from image center
                    float norm_x = (gate_center_x - center_x) / (float)center_x;
                    float norm_y = (gate_center_y - center_y) / (float)center_y;
                    
                    auto gate_msg = std_msgs::msg::Float64MultiArray();
                    gate_msg.data = {norm_x, norm_y, mean_depth};
                    gate_pub_->publish(gate_msg);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "Gate detected! Center: (%.2f, %.2f), Distance: %.2f m, Contour Area: %.0f px",
                        norm_x, norm_y, mean_depth, max_area);
                    
                    // Draw on display
                    cv::circle(latest_rgb_, cv::Point(gate_center_x, gate_center_y), 15, cv::Scalar(0, 255, 0), 3);
                    cv::putText(latest_rgb_, cv::format("Gate Distance: %.2f m", mean_depth),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                }
            }
            
            cv::putText(latest_rgb_, cv::format("LiDAR Alt: %.2f m", latest_lidar_distance_),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
            
            cv::imshow("Gate Detection - LiDAR + Color", latest_rgb_);
            cv::waitKey(1);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RGB exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GateDetectorLidar>());
    rclcpp::shutdown();
    return 0;
}
