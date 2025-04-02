#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

const static double rng = 1;

class RobotControl : public rclcpp::Node
{
public:

bool obstacle = false;
double err = 0;
double int_err = 0;
double old_err = 0;
RobotControl() : Node("robot_control")
    {
        // Инициализация Subscription, Publisher и Timer
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 10,
            std::bind(&RobotControl::laser_callback, this, std::placeholders::_1));
        poseSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "base_pose_ground_truth", 10,
            std::bind(&RobotControl::pose_callback, this, std::placeholders::_1));
        cmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&RobotControl::timer_callback, this));
    }

private:
    /**
    * Функция, которая будет вызвана
    * при получении данных от лазерного дальномера
    * параметр функции msg - ссылка на полученное сообщение
    */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser msg: " << msg->scan_time);

        const double kMinRange = 0.9;
        obstacle = false;
        double curr_range = msg->ranges[0];
        for (size_t i = 0; i<msg->ranges.size(); i++)
        {
            if (msg->ranges[i] < curr_range)
            {
                curr_range = msg->ranges[i];
            }
            //проверим нет ли вблизи робота препятствия
            if (msg->ranges[i] < kMinRange)
	        {
		        obstacle = true;
                RCLCPP_WARN_STREAM(this->get_logger(), "OBSTACLE!!!");
		        break;
	        }
        }
        err = rng - curr_range; //ошибка
    }
    

    /**
    * Функция, которая будет вызвана при
    * получении сообщения с текущем положением робота
    * параметр функции msg - ссылка на полученное сообщение
    */
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Pose msg: x = " << msg->pose.pose.position.x<<
            " y = " << msg->pose.pose.position.y <<
            " theta = " << 2*atan2(msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w));
    }
    
    /*
    * функция обработчик таймера
    * параметр функции - структура, описывающая событие таймера, здесь не используется
    */
    void timer_callback()
    {  
	    static int counter = 0;
	    counter++;
        RCLCPP_DEBUG_STREAM(this->get_logger(), "on timer "<<counter);
        //сообщение с помощью которого задается
	    //управление угловой и линейной скоростью
	    auto cmd = geometry_msgs::msg::Twist();
        //если вблизи нет препятствия то задаем команды
        if (!obstacle)
	    {
            int_err += err; //интегральная ошибка
            double dif_err = err - old_err; //дифференциальная ошибкаold_err = err;
            old_err = err;
            RCLCPP_INFO_STREAM(this->get_logger(), "go forward");
            cmd.linear.x = 0.5;
            cmd.angular.z = 13*err + 0.3*int_err + 6.5*dif_err; //ПИД-регулирование
	    }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Spin around!");
            cmd.linear.x = 0;
            cmd.angular.z = 0.5;
        }
        //отправляем (публикуем) команду
	    cmdPub_->publish(cmd);
    }

    // Subscription, Publisher и Timer
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr poseSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControl>());
    rclcpp::shutdown();
    return 0;
}