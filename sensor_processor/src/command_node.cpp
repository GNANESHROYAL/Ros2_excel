#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <ctime>
#include <fstream>

std::ofstream logfile("/home/gnanesh/ws/src/sensor_processor/logs/command_log.txt", std::ios_base::app);

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
class CommandNode : public rclcpp::Node {
public:
    CommandNode() : Node("command_node") {
        command_pub_ = this->create_publisher<std_msgs::msg::String>("command_topic", 10);
    }

    void handleUserInput() {
        std::string input;
        while (rclcpp::ok()) {
            std::cout << "Enter command (W, S, A, D, M): ";
            std::getline(std::cin, input);
            
            std_msgs::msg::String command_msg;
            command_msg.data = input;

            if (input == "W") {
                command_pub_->publish(command_msg);
                std::cout << "Forward" << std::endl;
                log("Published command W");
            } else if (input == "S") {
                command_pub_->publish(command_msg);
                std::cout << "Reverse" << std::endl;
                log("Published command S");
            } else if (input == "A") {
                command_pub_->publish(command_msg);
                std::cout << "Left" << std::endl;
                log("Published command A");
            } else if (input == "D") {
                command_pub_->publish(command_msg);
                std::cout << "Right" << std::endl;
                log("Published command D");
            } else if (input == "M") {
                command_pub_->publish(command_msg);
                std::cout << "BRAKE" << std::endl;
                log("Published command M");
            } else if (!input.empty()) {
                std::cout << "Command Sent: " << input << std::endl;
                
            }
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandNode>();

    while (rclcpp::ok()) {
        node->handleUserInput();
    }

    rclcpp::shutdown();
    return 0;
}
