# 📘 BioAI ROS 2 Integration Guide (0.7.6)

**Status:** Industrial Release (Stable)

**Unterstützte Distributionen:** Humble, Iron, Jazzy

**Architektur:** Neuro-Symbolic / Sparse Associative Memory (SAM)

---

## 1. Übersicht & Konzept

Dieser Guide beschreibt die Implementierung eines produktiven ROS 2 Knotens unter Verwendung der **BioAI Core Engine**. Im Gegensatz zu klassischen Deep-Learning-Ansätzen arbeitet BioAI mit **TokenIDs** und bietet deterministische Antwortzeiten von .

### Datenfluss im Robotik-Kontext

* **Wahrnehmung:** Sensordaten (LaserScan, Battery, etc.) werden in einem Translation-Layer in **TokenIDs** übersetzt.
* **Verarbeitung:** Der BioAI-Kern berechnet basierend auf Langzeitgedächtnis (LTM), aktuellen Plänen und Reflexen die optimale Aktion.
* **Aktion:** Die resultierende Aktions-ID wird zurück in ROS 2 Steuerbefehle gewandelt.

---

## 2. Paket-Konfiguration

### Dateistruktur

Erstellen Sie ein Standard-ROS 2 Paket und integrieren Sie die proprietären BioAI-Komponenten wie folgt:

```text
bioai_ros2_package/
├── CMakeLists.txt          <-- Angepasst für Native Linking
├── package.xml             <-- Abhängigkeiten (rclcpp, std_msgs)
├── include/
│   ├── BioAI.hpp           <-- Der offizielle C++ Wrapper
│   └── BioAI_Interface.h   <-- Native API Definitionen
├── lib/
│   └── libbioai_core.so    <-- Die Binärdatei (Tier: Ultra/SmartHome)
└── src/
    └── bioai_node.cpp      <-- Der Integrations-Knoten

```

### `package.xml` (Auszug)

Stellen Sie sicher, dass die Abhängigkeiten korrekt definiert sind:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>

```

---

## 3. Native Build-Konfiguration (CMake)

Verwenden Sie `IMPORTED_LOCATION`, um die vorkompilierte `libbioai_core.so` sicher einzubinden:

```cmake
# --- BIOAI NATIVE LINKING ---
add_library(bioai_lib SHARED IMPORTED)
set_target_properties(bioai_lib PROPERTIES 
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libbioai_core.so"
)

add_executable(bioai_node src/bioai_node.cpp)
target_link_libraries(bioai_node
    rclcpp::rclcpp
    std_msgs::std_msgs
    bioai_lib
)

```

---

## 4. Implementierung des Knotens (`src/bioai_node.cpp`)

Dieser Knoten nutzt den **RAII-Wrapper** (`BioBrainInstance`), um Speicherlecks zu verhindern und die Integrität des Kerns zu gewährleisten.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "BioAI.hpp" // C++ Wrapper v0.7.6

using namespace BioAI;

class BioAIBridgeNode : public rclcpp::Node {
public:
    BioAIBridgeNode() : Node("bioai_bridge") {
        // 1. Initialisierung mit key.json (RAII)
        std::string key_path = this->declare_parameter("key_path", "config/key.json");
        brain_ = std::make_unique<BioBrainInstance>(key_path);

        // 2. Modus-Einstellung (1 = Produktion/Echtzeit-Sicher)
        brain_->setMode(1);

        // 3. Subscriber: Eingaben (Wahrnehmungen)
        sub_inputs_ = this->create_subscription<std_msgs::msg::UInt64MultiArray>(
            "bioai/inputs", 10, [this](const std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
                // Inferenz auslösen
                uint64_t decision = brain_->update(msg->data);
                
                // Ergebnis publizieren
                auto out_msg = std_msgs::msg::UInt64();
                out_msg.data = decision;
                pub_action_->publish(out_msg);
                last_action_ = decision;
            });

        // 4. Subscriber: Feedback (Reinforcement Learning)
        sub_feedback_ = this->create_subscription<std_msgs::msg::Float32>(
            "bioai/feedback", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (last_action_ != 0) brain_->feedback(msg->data, last_action_);
            });

        pub_action_ = this->create_publisher<std_msgs::msg::UInt64>("bioai/action", 10);
        RCLCPP_INFO(this->get_logger(), "BioAI Node v0.7.6 started.");
    }

private:
    std::unique_ptr<BioBrainInstance> brain_;
    uint64_t last_action_ = 0;
    rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr sub_inputs_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_feedback_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_action_;
};

```

---

## 5. Wichtige Laufzeithinweise

### Cluster-Konformität

Senden Sie Inputs immer mit den korrekten Bit-Masken, um die kognitive Trennung im Kern zu wahren:

| Cluster | Maske (Hex) | Bedeutung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | **Das Ding / Der Zustand** | Sensordaten, Temperatur, Objekterkennung. |
| **ACTION** | `0x2000...` | **Das Tun** | Motorsteuerung, Datenbank-Eintrag, Not-Aus. |
| **TIME** | `0x3000...` | **Das Wann** | Zeitstempel, Intervalle, Schichtpläne. |
| **LOGIC** | `0x4000...` | **Die Regel** | Wenn-Dann-Verknüpfungen, logische Gatter. |
| **SELF** | `0x5000...` | **Das Ich** | Interne Zustände, Batteriestand, Zielerreichung. |

> **Wichtig:** Ein **Reflex** (unbrechbare Regel) nutzt die Sub-Maske `0x4010...`. Ein Token mit dieser Maske und einem Gewicht  unterbricht sofort alle anderen Denkprozesse.

### Echtzeit & Determinismus

Stellen Sie sicher, dass der Knoten im Produktionsmodus (`setMode(1)`) läuft. In diesem Zustand sind alle Speicherallokationen gesperrt, was die Engine immun gegen Jitter durch Memory-Management macht.

### Lizenz & Salting

Der Lizenzschlüssel wird nur einmal beim Start über die `key.json` geladen. Der C++ Wrapper kümmert sich intern um das **Salting** der Gewichte, sodass die Kommunikation über die ROS 2 Topics immer mit den echten, "entsalzten" TokenIDs erfolgt.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
