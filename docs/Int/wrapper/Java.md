# BioAI Java JNA Integration ☕

**Version:** 0.7.6 (Industrial Closed Feature)

**Platform:** Java 8+, Android API 24+

**Technology:** JNA (Java Native Access)

**Backend:** Ultra / SmartHome / IoT (Interchangeable)

---

## 1. Overview

The BioAI Java Wrapper provides a **high-performance bridge** to the native C-engine. Unlike legacy JNI approaches, this wrapper utilizes **JNA** to access native memory directly, minimizing overhead and complexity.

### Key Features

* **AutoCloseable:** Full support for `try-with-resources`. Prevents memory leaks automatically by handling native deallocation.
* **Zero-Copy Serialization:** Efficiently save and load brain states via native byte streams.
* **Thread-Safe:** Synchronized vocabulary management for multi-threaded applications.
* **Backend-Agnostic:** No need to recompile your Java code when switching between "IoT" and "Ultra" engines.

---

## 2. Installation & Setup

### Step A: Dependencies (Maven / Gradle)

You need the `JNA` (Java Native Access) library in your classpath.

**Maven:**

```xml
<dependency>
    <groupId>net.java.dev.jna</groupId>
    <artifactId>jna</artifactId>
    <version>5.13.0</version>
</dependency>

```

### Step B: Native Library

Java looks for the library in the `java.library.path`.

1. Pick your edition (e.g., `BioAI_Ultra.dll` or `BioAI_IoT.so`).
2. **Rename the file:**
* Windows: `bioai.dll`
* Linux/Android: `libbioai.so`


3. Place the file in your project root or set the path explicitly:
`-Djava.library.path=/path/to/libs`

---

## 3. Quick Start

This example demonstrates the complete lifecycle of an autonomous BioAI agent.

```java
import com.brainai.bioai.BioAI;
import com.brainai.bioai.BioAI.BioMode;

public class App {
    public static void main(String[] args) {
        // License Key / Seed for determinism
        long licenseKey = 0x12345678L;

        // 1. "try-with-resources" ensures native memory is freed automatically
        try (BioAI brain = new BioAI(licenseKey)) {
            
            System.out.println("BioAI Engine initialized.");

            // 2. Define Vocabulary (Names -> 64-bit Hashes)
            long sensorHeat = BioAI.createToken("SENSOR_HEAT", BioAI.CLUSTER_OBJECT);
            long actionFan  = BioAI.createToken("ACTION_FAN",  BioAI.CLUSTER_ACTION);

            // 3. Inject Instinct (Safety Rule)
            // "If HEAT detected, then activate FAN" (Weight 1.0 = Immutable Law)
            brain.forceInstinct(sensorHeat, actionFan, 1.0f);

            // 4. Process Perception (Inference in O(1) time)
            long decision = brain.think(sensorHeat);

            // 5. Execute Action
            if (decision == actionFan) {
                System.out.println("Decision: FAN ON (Correct Behavior)");
                // Provide positive reinforcement to consolidate experience
                brain.learn(1.0f, decision);
            } else {
                System.out.println("Decision: UNKNOWN");
            }

            // 6. Optional: Persistence
            brain.saveToFile("brain_backup.bin");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

```

---

## 4. Advanced Concepts

### Memory Management (CRITICAL)

Since BioAI operates in the **unmanaged heap** (outside the Java Virtual Machine), the Garbage Collector cannot reclaim its memory.

* **Always** use `try-with-resources` (as shown in the example above).
* Alternatively, call `brain.close()` manually when the object is no longer needed.

### Production Mode (ISO Compliance)

For certified industrial environments, you can "freeze" the neural structure to prevent runtime deviations.

```java
// Completely disables malloc() in the C-Core.
// Maximum stability for 24/7 industrial operation.
brain.setMode(BioAI.BioMode.PRODUCTION);

```

### Simulation ("Imagination")

Before executing a critical action, the agent can simulate potential consequences based on its causal history.

```java
// Simulates 5 steps into the future
long futureOutcome = brain.simulate(perceptionArray, 5);

```

---

## 5. API Reference

| Java Method | Native Equivalent | Description |
| --- | --- | --- |
| `think(long...)` | `API_Update` | Processes inputs in **** time. |
| `simulate(..., depth)` | `API_Simulate` | Recursive causality check. |
| `learn(reward, action)` | `API_Feedback` | Hebbian Learning on the recent trace. |
| `forceInstinct(...)` | `API_Teach` | Writes immutable rules into the LTM. |
| `inspect(...)` | `API_Inspect` | Debugging of synaptic weights. |
| `saveToFile(path)` | `API_Serialize` | Exports the entire state to binary. |

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
