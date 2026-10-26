#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

// No de CONTROLE (decisao): le /target e decide como mover o robo.
//   - Nao viu a bola: gira no lugar procurando.
//   - Viu a bola: vira em direcao a ela e anda ate ficar perto.
//   - Chegou perto: para.
// A atuacao e o comando de velocidade publicado em /cmd_vel.
class TargetFollower : public rclcpp::Node {
public:
  TargetFollower() : Node("target_follower") {
    this->declare_parameter<double>("kp_angular", 1.0);
    this->declare_parameter<double>("linear_speed", 0.5);
    this->declare_parameter<double>("search_speed", 0.4);
    this->declare_parameter<double>("stop_area", 0.12);

    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "target", 10,
      std::bind(&TargetFollower::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    double kp = this->get_parameter("kp_angular").as_double();
    double linear_speed = this->get_parameter("linear_speed").as_double();
    double search_speed = this->get_parameter("search_speed").as_double();
    double stop_area = this->get_parameter("stop_area").as_double();

    geometry_msgs::msg::Twist cmd;

    if (msg->z < 0.5) {
      // Nao achou: gira procurando
      cmd.angular.z = search_speed;
    } else {
      // Achou: vira para o lado em que a bola aparece (x > 0 = direita = girar negativo)
      cmd.angular.z = -kp * msg->x;
      // Anda ate a bola ocupar a fracao stop_area da imagem
      if (msg->y < stop_area) {
        cmd.linear.x = linear_speed;
      }
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetFollower>());
  rclcpp::shutdown();
  return 0;
}
