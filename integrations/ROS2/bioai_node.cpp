/*
 * BIOAI ROS 2 NODE (Industrial Grade)
 * Dependencies: rclcpp, std_msgs
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

// Importiere unseren C++ Wrapper
#include "../../wrappers/cpp/BioAI.hpp"

using std::placeholders::_1;

class BioAINode : public rclcpp::Node {
public:
    BioAINode() : Node("bioai_brain"), brain_(nullptr) {
        // Parameter: Lizenzschlüssel
        this->declare_parameter("license_key", (int64_t)12345);
        int64_t key = this->get_parameter("license_key").as_int();

        try {
            // Brain initialisieren
            brain_ = std::make_unique<BioAI::Agent>((uint64_t)key);
            
            // Instinkt: Sicherheit (Beispiel)
            uint64_t T_OBSTACLE = BioAI::Agent::CreateToken("Obstacle", BioAI::Cluster::Object);
            uint64_t T_STOP     = BioAI::Agent::CreateToken("Stop",     BioAI::Cluster::Action);
            brain_->ForceInstinct(T_OBSTACLE, T_STOP, 1.0f);

            RCLCPP_INFO(this->get_logger(), "BioAI Core initialized. Key: %ld", key);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to init BioAI: %s", e.what());
            throw;
        }

        // Publisher: Die Entscheidung der KI
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("bioai/action", 10);

        // Subscriber: Sensoren (String-Befehle werden zu Tokens gehasht)
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sensors/data", 10, std::bind(&BioAINode::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!brain_) return;

        // 1. Input Mapping: String -> TokenID
        // In der Praxis würde man hier komplexe Sensordaten mappen.
        // Hier: "Wall" -> Token(Wall)
        uint64_t inputToken = BioAI::Agent::CreateToken(msg->data, BioAI::Cluster::Object);
        
        // 2. Denken (O(1))
        std::vector<uint64_t> inputs = { inputToken };
        uint64_t decision = brain_->Think(inputs);

        // 3. Handeln (Publish)
        auto message = std_msgs::msg::UInt64();
        message.data = decision;
        
        // Log für Debugging
        RCLCPP_INFO(this->get_logger(), "Input: '%s' -> Action Token: %lu", msg->data.c_str(), decision);
        
        publisher_->publish(message);
    }

    std::unique_ptr<BioAI::Agent> brain_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BioAINode>());
    rclcpp::shutdown();
    return 0;
}
