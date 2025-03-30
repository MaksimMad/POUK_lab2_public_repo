#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
// using namespace std::placeholders::_1;

class RobotControl : public rclcpp::Node
{
public:
    RobotControl()
    : Node("robot_control")
    {
        // Создаем издателя для команды движения
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 10, std::bind(&RobotControl::laserCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_pose_ground_truth", 10, std::bind(&RobotControl::poseCallback, this, std::placeholders::_1));
        
        // Таймер для периодической отправки команд
        timer_ = this->create_wall_timer(
            500ms, std::bind(&RobotControl::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Simple driver node started");
    }

private:
    void timer_callback()
    {
        auto cmd = geometry_msgs::msg::Twist();

        // message.linear.x = 0.5;  // Постоянная скорость вперед 0.5 м/с
        // message.angular.z = 0.0; // Без поворота
        
        if (!obstacle)
        {
            //ROS_INFO_STREAM("go forward");
            cmd.linear.x = 0.5;
            cmd.angular.z = 0;
        }
        else
        {
            //ROS_WARN_STREAM("Spin around!");
            cmd.linear.x = 0;
            cmd.angular.z = 0.5;
        }


        publisher_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Publishing command: linear.x=%.2f", cmd.linear.x);
    }

    //void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr  msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Laser msg: %f", msg->scan_time);

        const double kMinRange = 0.5;
        obstacle = false;
        //проверим нет ли вблизи робота препятствия
        for (size_t i = 0; i<msg->ranges.size(); i++)
        {
            if (msg->ranges[i] < kMinRange)
            {
                obstacle = true;
                RCLCPP_WARN(this->get_logger(), "OBSTACLE!!!");
                break;
            }
        }
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr  msg)
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Pose msg: x = %f y = %f theta = %f", 
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    }
    
    //rclcpp::TimerBase::SharedPtr timer_;
    bool obstacle;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControl>());
    rclcpp::shutdown();
    return 0;
}