#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

std::ofstream logfile("/home/gnanesh/ws/src/sensor_processor/logs/obstacle_log.txt", std::ios_base::app);

void log(const std::string& message) {
    std::time_t now = std::time(0);
    std::tm* local_time = std::localtime(&now);
    logfile << "[" << (local_time->tm_year + 1900) << "-"
            << (local_time->tm_mon + 1) << "-"
            << local_time->tm_mday << " "
            << local_time->tm_hour << ":"
            << local_time->tm_min << ":"
            << local_time->tm_sec << "] "
            << message << std::endl;
}
class ObstaclePublisher : public rclcpp::Node {
public:
    ObstaclePublisher() : Node("obstacle_publisher"), first_row_skipped_(false), stop_publishing_(false) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("obstacle", 10);
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "command_topic", 10, std::bind(&ObstaclePublisher::command_callback, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ObstaclePublisher::publish_data, this)
        );
        file_.open("/home/gnanesh/ws/src/sensor_processor/src/obstacle.csv");
        if (!file_.is_open()) {
            log("[LOG] Can't open obstacle.csv file! ");
            rclcpp::shutdown();
        } else {
            log(" [LOG] Opened obstacle.csv file. " );
        }
    }

private:
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "M") {
            stop_publishing_ = true;
            log(" [LOG] Publishing stopped as 'M' recieved. ");
        } else if (msg->data == "W") {
            stop_publishing_ = false;
            log(" [LOG] Publishing resumed as 'W' recieved. ");
        }
    }

    void publish_data() {
        if (stop_publishing_) {
            log(" [LOG] Publishing is paused. ");
            return;
        }

        std::string line;
        if (!first_row_skipped_) {
            std::getline(file_, line);
            first_row_skipped_ = true;
            log(" [LOG] Skipped the first row of the CSV file. ");
            return;
        }

        if (std::getline(file_, line)) {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                try {
                    int value = std::stoi(cell);  
                    auto message = std_msgs::msg::Int32();
                    message.data = value;
                    log("Publishing obstacle data");
                    publisher_->publish(message);

                } catch (const std::invalid_argument& e) {
                    log("Non-integer value found, skipping cell");
                    continue;
                }
            }
        } else {
            log("Reached end of CSV file, restarting from the beginning.");
            file_.clear();  
            file_.seekg(0); 
            first_row_skipped_ = false; 
        }
    }

    bool first_row_skipped_;
    bool stop_publishing_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;  // Subscriber for command_topic
    rclcpp::TimerBase::SharedPtr timer_;
    std::ifstream file_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstaclePublisher>());
    rclcpp::shutdown();
    return 0;
}
