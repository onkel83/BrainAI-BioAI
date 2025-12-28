# BioAI ROS 2 Integration 🤖

**Version:** 0.5.5 (Industrial Beta)
**Supported Distros:** Humble, Iron, Jazzy
**Tier:** Ultra / SmartHome (Recommended for Robots)

---

## 1. Overview

Dieser Guide beschreibt, wie Sie die **BioAI Core Engine** in einen **ROS 2 Node** integrieren.
Das Ziel ist es, Sensor-Daten (Topics) direkt in TokenIDs zu wandeln, durch das Gehirn zu leiten und Steuerbefehle in Echtzeit (< 1ms) zu publizieren.

### Architektur
* **Input:** `sensors/data` (String oder SensorRaw) -> `BioAI::CreateToken`
* **Processing:** `BioAI::Agent::Think()` (C-Core, O(1))
* **Output:** `bioai/action` (UInt64 Token) -> Robot Controller

---

## 2. Package Setup

Erstellen Sie ein neues ROS 2 Paket in Ihrem Workspace (`src/`):

```bash
ros2 pkg create --build-type ament_cmake bioai_ros_node --dependencies rclcpp std_msgs
````

### Dateistruktur

Ihre Ordnerstruktur sollte so aussehen:

```text
bioai_ros_node/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── BioAI.hpp          <-- Der C++ Wrapper Header
├── lib/
│   └── libbioai.so        <-- Die Core Binary (z.B. Ultra oder SmartHome)
└── src/
    └── bioai_node.cpp     <-- Der Node Code
```

> **WICHTIG:** Benennen Sie Ihre Edition (z.B. `BioAI_SmartHome.so`) in `libbioai.so` um, damit der Linker sie findet.

-----

## 3\. Build Configuration (CMakeLists.txt)

Damit ROS die proprietäre Bibliothek findet und linkt, muss die `CMakeLists.txt` angepasst werden. Kopieren Sie dieses Template:

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

# Include Directories (für BioAI.hpp)
include_directories(include)

# Executable
add_executable(bioai_node src/bioai_node.cpp)
ament_target_dependencies(bioai_node rclcpp std_msgs)

# --- BIOAI LINKING START ---
# Findet die libbioai.so im lokalen lib/ Ordner
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

# Die .so Datei muss zur Laufzeit auch installiert werden
install(FILES "lib/libbioai.so"
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

-----

## 4\. The Node Code (`src/bioai_node.cpp`)

Dies ist der optimierte C++ Node für den produktiven Einsatz.

```cpp
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

// Header-Only Wrapper
#include "../include/BioAI.hpp"

using std::placeholders::_1;

class BioAINode : public rclcpp::Node {
public:
    BioAINode() : Node("bioai_brain"), brain_(nullptr) {
        // Parameter: Lizenzschlüssel (kann via launch-file gesetzt werden)
        this->declare_parameter("license_key", (int64_t)0xCAFEBABE);
        int64_t key = this->get_parameter("license_key").as_int();

        try {
            // Brain initialisieren
            // Nutzt die gelinkte libbioai.so (Ultra/IoT/SmartHome)
            brain_ = std::make_unique<BioAI::Agent>((uint64_t)key);
            
            // Instinkt: Hard-Coded Safety Rules
            uint64_t T_OBSTACLE = BioAI::Agent::CreateToken("Obstacle", BioAI::Cluster::Object);
            uint64_t T_STOP     = BioAI::Agent::CreateToken("Stop",     BioAI::Cluster::Action);
            
            brain_->ForceInstinct(T_OBSTACLE, T_STOP, 1.0f);

            RCLCPP_INFO(this->get_logger(), "BioAI Core initialized. Key: %lX", key);
            RCLCPP_INFO(this->get_logger(), "Safety Instinct [Obstacle -> Stop] active.");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to init BioAI: %s", e.what());
            // In Production: Node shutdown erzwingen
            exit(1); 
        }

        // Publisher: Die Entscheidung der KI
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("bioai/action", 10);

        // Subscriber: Sensoren
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sensors/data", 10, std::bind(&BioAINode::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!brain_) return;

        // 1. Input Mapping: String -> TokenID
        // Hier wird der String gehasht. Das Ergebnis ist deterministisch.
        uint64_t inputToken = BioAI::Agent::CreateToken(msg->data, BioAI::Cluster::Object);
        
        // 2. Denken (O(1))
        std::vector<uint64_t> inputs = { inputToken };
        uint64_t decision = brain_->Think(inputs);

        // 3. Handeln (Publish)
        auto message = std_msgs::msg::UInt64();
        message.data = decision;
        
        // Debug Log (nur im Debug Level, um Performance zu sparen)
        RCLCPP_DEBUG(this->get_logger(), "In: '%s' -> Out: %lu", msg->data.c_str(), decision);
        
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

-----

## 5\. Running the Node

1.  **Build:**

    ```bash
    colcon build --packages-select bioai_ros_node
    ```

2.  **Source:**

    ```bash
    source install/setup.bash
    ```

3.  **Run:**

    ```bash
    # LD_LIBRARY_PATH muss gesetzt sein, damit die .so gefunden wird
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/bioai_ros_node/lib/bioai_ros_node
    ros2 run bioai_ros_node bioai_node
    ```

-----

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.