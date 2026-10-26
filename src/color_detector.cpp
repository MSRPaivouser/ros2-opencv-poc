#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>

// No de PERCEPCAO: acha a bola vermelha na imagem da camera do robo.
// Publica em /target (geometry_msgs/Point):
//   x = deslocamento horizontal da bola (-1 esquerda, 0 centro, +1 direita)
//   y = fracao da imagem ocupada pela bola (quanto maior, mais perto)
//   z = 1 se achou a bola, 0 se nao achou
class ColorDetector : public rclcpp::Node {
public:
  ColorDetector() : Node("color_detector") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10,
      std::bind(&ColorDetector::callback, this, std::placeholders::_1));
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("target", 10);
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
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

    // 2. Segmentacao por cor: vermelho tem duas faixas de matiz no HSV
    cv::Mat hsv, mask_low, mask_high, mask;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 150, 60), cv::Scalar(6, 255, 255), mask_low);
    cv::inRange(hsv, cv::Scalar(174, 150, 60), cv::Scalar(180, 255, 255), mask_high);
    cv::bitwise_or(mask_low, mask_high, mask);

    // 3. Centro e tamanho da regiao vermelha
    cv::Moments m = cv::moments(mask, true);
    geometry_msgs::msg::Point target;
    cv::Mat debug = cv_ptr->image.clone();

    if (m.m00 > 20.0) {
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;
      double half_width = mask.cols / 2.0;
      target.x = (cx - half_width) / half_width;
      target.y = m.m00 / (static_cast<double>(mask.cols) * mask.rows);
      target.z = 1.0;
      cv::circle(debug, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                 10, cv::Scalar(0, 255, 0), 2);
    } else {
      target.x = 0.0;
      target.y = 0.0;
      target.z = 0.0;
    }

    // 4. Publica o alvo e uma imagem de depuracao
    target_pub_->publish(target);
    auto out = cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg();
    debug_pub_->publish(*out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetector>());
  rclcpp::shutdown();
  return 0;
}
