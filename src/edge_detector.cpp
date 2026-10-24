#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

class EdgeDetector : public rclcpp::Node {
public:
  EdgeDetector() : Node("edge_detector") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
      std::bind(&EdgeDetector::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_edges", 10);
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 1. ROS -> OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // 2. Processamento com OpenCV
    cv::Mat gray, blurred, edges;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 100, 200);

    // 3. OpenCV -> ROS e publica
    auto out = cv_bridge::CvImage(msg->header, "mono8", edges).toImageMsg();
    pub_->publish(*out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeDetector>());
  rclcpp::shutdown();
  return 0;
}
