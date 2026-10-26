#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

// No de CONTROLE com desvio de obstaculos.
//   Entradas:  /target (onde a camera ve a bola) e /scan (distancias do lidar)
//   Saida:     /cmd_vel (velocidade das rodas)
//
// Regras:
//   1. Bola visivel: o objetivo e ir na direcao dela.
//   2. Bola nao visivel: gira procurando e, se nao achar, explora andando em frente.
//   3. Em qualquer caso, o robo escolhe o rumo livre mais proximo do objetivo,
//      usando o lidar para nao bater nas paredes (tecnica "seguir a lacuna").
//   4. Bola perto (ocupa area suficiente da imagem): para.
class Navigator : public rclcpp::Node {
public:
  Navigator() : Node("navigator") {
    this->declare_parameter<double>("kp", 1.2);
    this->declare_parameter<double>("linear_speed", 0.3);
    this->declare_parameter<double>("explore_speed", 0.25);
    this->declare_parameter<double>("search_speed", 0.6);
    this->declare_parameter<double>("turn_speed", 0.6);
    this->declare_parameter<double>("max_angular", 0.8);
    this->declare_parameter<double>("stop_area", 0.04);
    this->declare_parameter<double>("d_look", 0.75);
    this->declare_parameter<double>("r_safe", 0.30);
    this->declare_parameter<double>("d_stop", 0.35);
    this->declare_parameter<double>("d_slow", 0.9);
    this->declare_parameter<double>("lost_time", 1.5);
    this->declare_parameter<double>("search_period", 12.0);
    this->declare_parameter<double>("explore_period", 6.0);
    this->declare_parameter<double>("camera_fov", 1.3962634);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Navigator::on_scan, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "target", 10,
      std::bind(&Navigator::on_target, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    mode_start_ = this->now();
    timer_ = this->create_wall_timer(100ms, std::bind(&Navigator::control_loop, this));
  }

private:
  enum class Mode { SEARCH, EXPLORE };

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }

  void on_target(const geometry_msgs::msg::Point::SharedPtr msg) {
    target_ = *msg;
    target_stamp_ = this->now();
    have_target_ = true;
  }

  double param(const std::string & name) { return this->get_parameter(name).as_double(); }

  void publish(double linear, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  // Escolhe, entre os rumos de -90 a +90 graus, o livre mais proximo do objetivo.
  // Cada ponto do lidar a menos de d_look "fecha" os rumos dentro de asin(r_safe / distancia).
  bool pick_heading(const std::vector<double> & angles, const std::vector<double> & ranges,
                    double goal, double & chosen) {
    const double d_look = param("d_look");
    const double r_safe = param("r_safe");
    bool found = false;
    double best_cost = 1e9;
    for (int deg = -90; deg <= 90; deg += 3) {
      double th = deg * M_PI / 180.0;
      bool ok = true;
      for (size_t i = 0; i < angles.size(); ++i) {
        double d = ranges[i];
        if (d < d_look) {
          double half = std::asin(std::min(1.0, r_safe / std::max(d, 1e-3)));
          if (std::fabs(th - angles[i]) < half) {
            ok = false;
            break;
          }
        }
      }
      if (ok) {
        double cost = std::fabs(th - goal) + 0.01 * std::fabs(th);
        if (cost < best_cost) {
          best_cost = cost;
          chosen = th;
          found = true;
        }
      }
    }
    return found;
  }

  void control_loop() {
    if (!scan_) {
      return;  // sem lidar ainda: nao se move
    }
    const auto now = this->now();

    // --- leitura do alvo (camera) ---
    bool found = false;
    double x = 0.0, area = 0.0;
    if (have_target_ && (now - target_stamp_).seconds() < 0.5 && target_.z > 0.5) {
      found = true;
      x = target_.x;
      area = target_.y;
    }

    // --- leitura do lidar ---
    std::vector<double> angles, ranges;
    const double max_range = 8.0;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      double r = scan_->ranges[i];
      if (std::isnan(r)) {
        continue;
      }
      if (std::isinf(r)) {
        r = (r > 0) ? max_range : scan_->range_min;
      }
      r = std::min(std::max(r, static_cast<double>(scan_->range_min)), max_range);
      angles.push_back(scan_->angle_min + i * scan_->angle_increment);
      ranges.push_back(r);
    }
    if (angles.empty()) {
      return;
    }
    double front = max_range;
    for (size_t i = 0; i < angles.size(); ++i) {
      if (std::fabs(angles[i]) < 20.0 * M_PI / 180.0) {
        front = std::min(front, ranges[i]);
      }
    }

    // --- objetivo ---
    double goal = 0.0;
    bool approaching = false;

    if (found) {
      last_seen_ = now;
      have_seen_ = true;
      last_x_ = x;
      mode_ = Mode::SEARCH;
      mode_start_ = now;
      if (area >= param("stop_area")) {
        publish(0.0, 0.0);  // chegou perto da bola
        return;
      }
      goal = -std::atan(x * std::tan(param("camera_fov") / 2.0));
      approaching = true;
    } else if (have_seen_ && (now - last_seen_).seconds() < param("lost_time")) {
      // acabou de perder a bola de vista (quina de parede, por exemplo): vira para o lado onde ela estava
      double side = (last_x_ != 0.0) ? last_x_ : 1.0;
      goal = -std::copysign(40.0 * M_PI / 180.0, side);
    } else {
      if (mode_ == Mode::SEARCH) {
        if ((now - mode_start_).seconds() > param("search_period")) {
          mode_ = Mode::EXPLORE;
          mode_start_ = now;
        } else {
          publish(0.0, -param("search_speed"));  // gira no lugar procurando
          return;
        }
      }
      if (mode_ == Mode::EXPLORE) {
        if ((now - mode_start_).seconds() > param("explore_period")) {
          mode_ = Mode::SEARCH;
          mode_start_ = now;
          publish(0.0, -param("search_speed"));
          return;
        }
      }
      goal = 0.0;  // explorando: segue em frente desviando das paredes
    }

    // --- escolha do rumo livre mais proximo do objetivo ---
    double th = 0.0;
    if (!pick_heading(angles, ranges, goal, th)) {
      // tudo bloqueado: gira para o lado com mais espaco
      double left = 0.0, right = 0.0;
      for (size_t i = 0; i < angles.size(); ++i) {
        if (angles[i] > 0) {
          left += ranges[i];
        } else {
          right += ranges[i];
        }
      }
      publish(0.0, (left > right) ? param("turn_speed") : -param("turn_speed"));
      return;
    }

    double ang = std::max(-param("max_angular"), std::min(param("max_angular"), param("kp") * th));
    double lin = approaching ? param("linear_speed") : param("explore_speed");
    if (std::fabs(th) > 50.0 * M_PI / 180.0) {
      lin = 0.0;  // rumo muito lateral: gira antes de andar
    } else {
      lin *= std::max(0.3, std::cos(th));
    }
    double k = (front - param("d_stop")) / (param("d_slow") - param("d_stop"));
    lin *= std::max(0.0, std::min(1.0, k));  // desacelera perto de paredes

    publish(lin, ang);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::Point target_;
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  bool have_target_ = false;

  rclcpp::Time last_seen_{0, 0, RCL_ROS_TIME};
  bool have_seen_ = false;
  double last_x_ = 0.0;

  Mode mode_ = Mode::SEARCH;
  rclcpp::Time mode_start_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
