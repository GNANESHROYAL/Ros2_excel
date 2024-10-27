#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <fstream>
#include <ctime>

std::ofstream logfile("/home/gnanesh/ws/src/sensor_processor/logs/sensor_processor_log.txt", std::ios_base::app);

void log(const std::string &message) {
    std::time_t now = std::time(0);
    std::tm *local_time = std::localtime(&now);
    logfile << "[" << (local_time->tm_year + 1900) << "-"
            << (local_time->tm_mon + 1) << "-"
            << local_time->tm_mday << " "
            << local_time->tm_hour << ":"
            << local_time->tm_min << ":"
            << local_time->tm_sec << "] "
            << message << std::endl;
    logfile.flush();
}

class SensorProcessor : public rclcpp::Node {
public:
    SensorProcessor() : Node("sensor_processor_node"), obstacle_detected(false), manual_stop(false) {
        obstacle_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "obstacle", 10, std::bind(&SensorProcessor::obstacle_callback, this, std::placeholders::_1)
        );
        accelerometer_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "accelerometer", 10, std::bind(&SensorProcessor::accelerometer_callback, this, std::placeholders::_1)
        );
        gyro_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "gyro", 10, std::bind(&SensorProcessor::gyro_callback, this, std::placeholders::_1)
        );
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "command_topic", 10, std::bind(&SensorProcessor::command_callback, this, std::placeholders::_1)
        );
    }

private:
    void obstacle_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        obstacle_data = msg->data;
        obstacle_detected = (obstacle_data == 1);
        if (obstacle_detected) {
            log("OBSTACLE DETECTED");
            RCLCPP_INFO(this->get_logger(), "OBSTACLE DETECTED");
        }
        check_for_stopped();
    }

    void accelerometer_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() > 1) {
            accelerometer_data = {msg->data.begin() + 1, msg->data.end()};
            log("Received accelerometer data.");
            RCLCPP_INFO(this->get_logger(), "Received accelerometer data. %f,%f,%f",msg->data[1],msg->data[2],msg->data[3]);
        } else {
            log("Invalid accelerometer data.");
        }
    }

    void gyro_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() > 1) {
            gyro_data= {msg->data.begin() + 1, msg->data.end()};
            log("Received gyro data.");
            RCLCPP_INFO(this->get_logger(), "Received gyro data. %f,%f,%f",msg->data[1],msg->data[2],msg->data[3]);
        } else {
            log("Invalid gyro data.");
        }
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        manual_stop = (msg->data == "M");
        if (manual_stop) {
            log("BRAKE applied by manual command.");
            RCLCPP_INFO(this->get_logger(), "BRAKE");
            check_for_stopped();
        } else if (msg->data == "W") {
            log("Forward command received.");
            RCLCPP_INFO(this->get_logger(), "Forward");
        }
    }

    void check_for_stopped() {
        if (obstacle_detected && manual_stop) {
            log("STOPPED due to obstacle and manual stop.");
            RCLCPP_INFO(this->get_logger(), "STOPPED");
        }
    }

    bool obstacle_detected;
    bool manual_stop;
    int obstacle_data;
    std::vector<float> accelerometer_data;
    std::vector<float> gyro_data;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr accelerometer_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessor>());
    rclcpp::shutdown();
    return 0;
}
