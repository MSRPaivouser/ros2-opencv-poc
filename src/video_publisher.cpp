#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node {
public:
  VideoPublisher() : Node("video_publisher") {
    this->declare_parameter<std::string>("video_path", "video.mp4");
    std::string path = this->get_parameter("video_path").as_string();

    cap_.open(path);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Nao consegui abrir o video: %s", path.c_str());
      return;
    }

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = this->create_wall_timer(
      33ms, std::bind(&VideoPublisher::publish_frame, this));
  }

private:
  void publish_frame() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      // Acabou o video: volta para o inicio
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      return;
    }
    std_msgs::msg::Header header;
    header.stamp = this->now();
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    pub_->publish(*msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}
