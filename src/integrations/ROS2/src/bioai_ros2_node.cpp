#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include "BioAI.hpp" // Dein C++ Wrapper

using namespace BioAI;

class BioAIRos2Bridge : public rclcpp::Node {
public:
    BioAIRos2Bridge() : Node("bioai_bridge_node") {
        // 1. Parameter für Pfade deklarieren
        std::string key_path = this->declare_parameter("key_path", "config/key.json");
        int mode = this->declare_parameter("mode", 0); // 0=Lernen, 1=Produktion

        // 2. BioAI Instanz über RAII-Wrapper laden
        try {
            brain_ = std::make_unique<BioBrainInstance>(key_path);
            brain_->setMode(mode);
            RCLCPP_INFO(this->get_logger(), "BioAI Engine erfolgreich gestartet.");
        }
        catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Fehler beim Starten von BioAI: %s", e.what());
            return;
        }

        // 3. ROS Schnittstellen
        // Input: Liste von TokenIDs (Sensordaten)
        sub_inputs_ = this->create_subscription<std_msgs::msg::UInt64MultiArray>(
            "bioai/inputs", 10, std::bind(&BioAIRos2Bridge::input_callback, this, std::placeholders::_1));

        // Feedback: Belohnung/Bestrafung für die letzte Aktion
        sub_feedback_ = this->create_subscription<std_msgs::msg::Float32>(
            "bioai/feedback", 10, std::bind(&BioAIRos2Bridge::feedback_callback, this, std::placeholders::_1));

        // Output: Die gewählte Aktion der Engine
        pub_action_ = this->create_publisher<std_msgs::msg::UInt64>("bioai/action", 10);
    }

private:
    void input_callback(const std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
        // Direkter Aufruf der Engine-Logik
        last_action_ = brain_->update(msg->data);

        auto out_msg = std_msgs::msg::UInt64();
        out_msg.data = last_action_;
        pub_action_->publish(out_msg);
    }

    void feedback_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (last_action_ != 0) {
            // Reinforcement Learning anwenden
            brain_->feedback(msg->data, last_action_);
        }
    }

    std::unique_ptr<BioBrainInstance> brain_;
    uint64_t last_action_ = 0;
    rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr sub_inputs_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_feedback_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_action_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BioAIRos2Bridge>());
    rclcpp::shutdown();
    return 0;
}