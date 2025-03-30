#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class RobotControl : public rclcpp::Node
{
public:
RobotControl() : Node("control_node")
  {
    // Инициализация подписчиков и издателя
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "base_scan", 10, std::bind(&RobotControl::laserCallback, this, _1));
    
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "base_pose_ground_truth", 10, std::bind(&RobotControl::poseCallback, this, _1));
    
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Таймер с частотой 10 Гц (каждые 0.1 секунды)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&RobotControl::timerCallback, this));
    
    // Оптимальные коэффициенты PID после подбора
    // kp_ = 1.2;  // пропорциональный коэффициент
    // ki_ = 0.05; // интегральный коэффициент
    // kd_ = 0.2;  // дифференциальный коэффициент

    kp_ = 40;  // пропорциональный коэффициент
    ki_ = 0.05; // интегральный коэффициент
    kd_ = 0.2; 
    
    RCLCPP_INFO(this->get_logger(), "Control node has been started");
  }

private:
  // Флаги и переменные состояния
  bool obstacle_ = false;
  double err_ = 0;
  double int_err_ = 0;
  double old_err_ = 0;
  const double range_ = 1.0;  // желаемое расстояние до стенки
  
  // Коэффициенты PID
  double kp_;
  double ki_;
  double kd_;
  
  // Подписчики, издатель и таймер
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * Обработчик данных лазерного дальномера
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Laser msg: %f", msg->scan_time);
    const double kMinRange = 0.9;
    obstacle_ = false;
    double curr_range = msg->ranges[0];
    
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      if (msg->ranges[i] < curr_range)
      {
        curr_range = msg->ranges[i];
      }
      // Проверяем, нет ли вблизи робота препятствия
      if (msg->ranges[i] < kMinRange)
      {
        obstacle_ = true;
        RCLCPP_WARN(this->get_logger(), "OBSTACLE!!!");
        break;
      }
    }
    err_ = range_ - curr_range; // ошибка регулирования
  }

  /**
   * Обработчик данных о положении робота
   */
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double theta = 2 * atan2(msg->pose.pose.orientation.z, 
                            msg->pose.pose.orientation.w);
    RCLCPP_DEBUG(this->get_logger(), "Pose msg: x = %f, y = %f, theta = %f",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 theta);
  }

  /**
   * Обработчик таймера (вызывается каждые 0.1 секунды)
   */
  void timerCallback()
  {
    static int counter = 0;
    counter++;
    RCLCPP_DEBUG(this->get_logger(), "on timer %d", counter);
    
    auto cmd = geometry_msgs::msg::Twist();
    
    if (!obstacle_)
    {
      int_err_ += err_; // интегральная составляющая
      double dif_err = err_ - old_err_; // дифференциальная составляющая
      old_err_ = err_;
      
      RCLCPP_INFO(this->get_logger(), "Go forward");
      cmd.linear.x = 0.5;
      
      // ПИД-регулирование с подобранными коэффициентами
      cmd.angular.z = kp_ * err_ + ki_ * int_err_ + kd_ * dif_err;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Spin around!");
      cmd.linear.x = 0;
      cmd.angular.z = 0.5;
    }
    
    pub_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}