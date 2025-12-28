# BioAI ROS 2 Integration 🤖

**Version:** 0.7.6 (Industrial Closed Feature)

**Supported Distros:** Humble, Iron, Jazzy

**Tier:** Ultra / SmartHome (Recommended for Robotics)

---

## 1. Overview

This guide describes how to integrate the **BioAI Core Engine** into a **ROS 2 Node**. The objective is to convert sensor data (Topics) into TokenIDs, process them through the brain, and publish control commands in real-time ().

### Architecture & Data Flow

* **Input:** `sensors/data` (String or SensorRaw)  `BioAI::CreateToken`
* **Processing:** `BioAI::Agent::Think()` (Native C-Core,  complexity)
* **Output:** `bioai/action` (UInt64 TokenID)  Robot Controller / Actuator Node

---

## 2. Package Setup

Create a new ROS 2 package in your workspace (`src/`):

```bash
ros2 pkg create --build-type ament_cmake bioai_ros_node --dependencies rclcpp std_msgs

```

### File Structure

Your project directory should be organized as follows:

```text
bioai_ros_node/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── BioAI.hpp          <-- The C++ Wrapper Header
├── lib/
│   └── libbioai.so        <-- The Core Binary (Ultra or SmartHome)
└── src/
    └── bioai_node.cpp      <-- The Node Source Code

```

> **IMPORTANT:** Rename your specific edition (e.g., `BioAI_SmartHome.so`) to `libbioai.so` so the linker can find it correctly.

---

## 3. Build Configuration (`CMakeLists.txt`)

To ensure ROS finds and links the proprietary library, modify your `CMakeLists.txt` using the template below:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bioai_ros_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include Directories (for BioAI.hpp)
include_directories(include)

# Executable
add_executable(bioai_node src/bioai_node.cpp)
ament_target_dependencies(bioai_node rclcpp std_msgs)

# --- BIOAI LINKING START ---
# Locates libbioai.so in the local lib/ folder
find_library(BIOAI_LIB bioai PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib" NO_DEFAULT_PATH)

if(NOT BIOAI_LIB)
  message(FATAL_ERROR "libbioai.so not found in lib/ folder!")
endif()

target_link_libraries(bioai_node ${BIOAI_LIB})
# --- BIOAI LINKING END ---

# Install Rules
install(TARGETS
  bioai_node
  DESTINATION lib/${PROJECT_NAME})

# The .so file must be installed for runtime access
install(FILES "lib/libbioai.so"
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

---

## 4. The Node Code (`src/bioai_node.cpp`)

This optimized C++ node is designed for production use, ensuring memory safety through `std::unique_ptr`.

```cpp
/*
 * BIOAI ROS 2 NODE (Industrial Grade)
 * Dependencies: rclcpp, std_msgs, BioAI Native Core
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

// Header-Only Wrapper
#include "BioAI.hpp"

using std::placeholders::_1;

class BioAINode : public rclcpp::Node {
public:
    BioAINode() : Node("bioai_brain"), brain_(nullptr) {
        // Parameter: License key (can be set via launch file)
        this->declare_parameter("license_key", (int64_t)0xCAFEBABE);
        int64_t key = this->get_parameter("license_key").as_int();

        try {
            // Initialize Brain using the linked libbioai.so
            brain_ = std::make_unique<BioAI::Agent>((uint64_t)key);
            
            // Hard-Coded Safety Instincts
            uint64_t T_OBSTACLE = BioAI::Agent::CreateToken("Obstacle", BioAI::Cluster::Object);
            uint64_t T_STOP     = BioAI::Agent::CreateToken("Stop",     BioAI::Cluster::Action);
            
            brain_->ForceInstinct(T_OBSTACLE, T_STOP, 1.0f);

            RCLCPP_INFO(this->get_logger(), "BioAI Core initialized. Key: %lX", key);
            RCLCPP_INFO(this->get_logger(), "Safety Reflex [Obstacle -> Stop] active.");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: BioAI Initialization failed: %s", e.what());
            exit(1); // Force shutdown in production if brain fails
        }

        // Publisher: The AI's decision
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("bioai/action", 10);

        // Subscriber: Raw Sensor Data
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sensors/data", 10, std::bind(&BioAINode::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!brain_) return;

        // 1. Input Mapping: String -> TokenID
        uint64_t inputToken = BioAI::Agent::CreateToken(msg->data, BioAI::Cluster::Object);
        
        // 2. Inference (O(1) constant time)
        std::vector<uint64_t> inputs = { inputToken };
        uint64_t decision = brain_->Think(inputs);

        // 3. Publish Decision
        auto message = std_msgs::msg::UInt64();
        message.data = decision;
        
        RCLCPP_DEBUG(this->get_logger(), "Input: '%s' -> Action: %lu", msg->data.c_str(), decision);
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

```

---

## 5. Running the Node

1. **Build:**
```bash
colcon build --packages-select bioai_ros_node

```


2. **Source:**
```bash
source install/setup.bash

```


3. **Run:**
```bash
# Ensure LD_LIBRARY_PATH includes the installation folder so the .so is found
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/bioai_ros_node/lib/bioai_ros_node
ros2 run bioai_ros_node bioai_node

```



---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

