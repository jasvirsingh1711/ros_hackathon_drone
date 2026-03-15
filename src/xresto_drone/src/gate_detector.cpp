#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class GateDetector : public rclcpp::Node {
public:
    GateDetector() : Node("gate_detector") {
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/drone/front_camera/image_raw", 10, std::bind(&GateDetector::rgb_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/drone/front_camera/depth/image_raw", 10, std::bind(&GateDetector::depth_callback, this, std::placeholders::_1));

        gate_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drone/gate_detection", 10);
        
        RCLCPP_INFO(this->get_logger(), "Gate Detector - Edge-based detection active");
    }

private:
    cv::Mat latest_depth_;

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            latest_depth_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth exception: %s", e.what());
        }
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            int width = frame.cols;
            int height = frame.rows;

            if (latest_depth_.empty() || latest_depth_.rows != height) {
                return;
            }

            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // IMPORTANT: Mask out bottom of image to exclude drone body/legs
            // Only look at top 70% of image where actual gates are
            cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
            cv::rectangle(mask, cv::Point(0, 0), cv::Point(width, height * 0.7), cv::Scalar(255), -1);
            
            // Blur for noise reduction
            cv::Mat blurred;
            cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);
            blurred = blurred.mul(mask / 255.0);  // Apply mask to blurred image

            // Edge detection - works on ANY color gate
            cv::Mat edges;
            cv::Canny(blurred, edges, 50, 150);

            // Morphological closing to connect edges
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);
            cv::dilate(edges, edges, kernel, cv::Point(-1, -1), 1);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(edges.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int gate_count = 0;
            float best_score = 0;
            cv::Rect best_gate;

            // Analyze each contour
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                
                if (area < 1000 || area > 150000) {
                    continue;  // Filter by size - gates are large, drone parts are tiny
                }

                cv::Rect rect = cv::boundingRect(contour);
                if (rect.area() < 100) continue;
                
                // REJECT detections too close to bottom (drone body)
                if (rect.y + rect.height > height * 0.75) {
                    continue;  // Ignore bottom 25% of image
                }
                
                // Aspect ratio filter for gate-like shapes
                float aspect = (float)rect.width / (float)rect.height;
                if (aspect < 0.3 || aspect > 3.0) {
                    continue;
                }

                // Score based on contour properties
                float contour_area = (float)area;
                float rect_area = (float)rect.area();
                float fill_ratio = contour_area / rect_area;
                float score = contour_area * fill_ratio;

                // Check depth in gate region for valid distance
                float gate_depth = 0;
                int valid_pixels = 0;
                for (int y = std::max(0, rect.y); y < std::min(height, rect.y + rect.height); y++) {
                    for (int x = std::max(0, rect.x); x < std::min(width, rect.x + rect.width); x++) {
                        float d = latest_depth_.at<float>(y, x);
                        if (!std::isnan(d) && !std::isinf(d) && d > 0.2 && d < 50.0) {
                            gate_depth += d;
                            valid_pixels++;
                        }
                    }
                }
                if (valid_pixels > 0) gate_depth /= valid_pixels;

                // Update best gate
                if (score > best_score && valid_pixels > 50) {
                    best_score = score;
                    best_gate = rect;
                    gate_count = 1;
                }

                // Draw all candidates
                cv::rectangle(frame, rect, cv::Scalar(100, 100, 100), 1);
            }

            // Publish and visualize detection
            if (gate_count > 0) {
                // Convert to Float64MultiArray format: [center_x, center_y, gate_size]
                auto detection = std_msgs::msg::Float64MultiArray();
                float center_x = best_gate.x + best_gate.width / 2.0;
                float center_y = best_gate.y + best_gate.height / 2.0;
                float gate_size = (float)(best_gate.width * best_gate.height);
                
                detection.data.push_back(center_x);
                detection.data.push_back(center_y);
                detection.data.push_back(gate_size);
                gate_pub_->publish(detection);

                cv::rectangle(frame, best_gate, cv::Scalar( 0, 255, 0), 3);
                cv::putText(frame, "GATE DETECTED", cv::Point(best_gate.x, best_gate.y - 15),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }

            // Debug info
            cv::putText(frame, "Edges: " + std::to_string(cv::countNonZero(edges)) + " px", 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 1);
            cv::putText(frame, "Contours: " + std::to_string(contours.size()), 
                       cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 1);

            cv::imshow("Gate Detection (Any Color)", frame);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RGB exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gate_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GateDetector>());
    rclcpp::shutdown();
    return 0;
}