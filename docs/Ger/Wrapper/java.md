# BioAI Java JNA Integration ☕

**Version:** 0.7.6
**Platform:** Java 8+, Android API 24+
**Technology:** JNA (Java Native Access)
**Backend:** Ultra / SmartHome / IoT (Austauschbar)

---

## 1. Übersicht

Der BioAI Java Wrapper bietet eine **High-Performance Bridge** zur nativen C-Engine. Anders als veraltete JNI-Ansätze nutzt dieser Wrapper **JNA**, um direkt auf den Speicher zuzugreifen.

### Features
* **AutoCloseable:** Volle Unterstützung für `try-with-resources`. Speicherlecks werden automatisch verhindert.
* **Zero-Copy Serialization:** Effizientes Speichern und Laden von Gehirnen über native Byte-Streams.
* **Thread-Safe:** Das Vokabular-Management ist synchronisiert.
* **Backend-Agnostic:** Der Java-Code muss nicht neu kompiliert werden, wenn Sie von der "IoT" zur "Ultra" Engine wechseln.

---

## 2. Installation & Setup

### Schritt A: Abhängigkeiten (Maven / Gradle)
Sie benötigen `JNA` (Java Native Access) in Ihrem Classpath.

**Maven:**
```xml
<dependency>
    <groupId>net.java.dev.jna</groupId>
    <artifactId>jna</artifactId>
    <version>5.13.0</version>
</dependency>
````

### Schritt B: Native Library

Java sucht nach der Bibliothek im `java.library.path`.

1.  Wählen Sie Ihre Edition (z.B. `BioAI_Ultra.dll` oder `BioAI_IoT.so`).
2.  **Benennen Sie die Datei um:**
      * Windows: `bioai.dll`
      * Linux/Android: `libbioai.so`
3.  Platzieren Sie die Datei im Root Ihres Projekts oder setzen Sie den Pfad:
    `-Djava.library.path=/path/to/libs`

-----

## 3\. Quick Start

Dieses Beispiel zeigt den kompletten Lebenszyklus eines Agenten.

```java
import com.brainai.bioai.BioAI;
import com.brainai.bioai.BioAI.BioMode;

public class App {
    public static void main(String[] args) {
        // Seed für Determinismus (0x1234...)
        long licenseKey = 0x12345678L;

        // 1. "try-with-resources" garantiert Speicherfreigabe (native free)
        try (BioAI brain = new BioAI(licenseKey)) {
            
            System.out.println("BioAI Engine initialized.");

            // 2. Vokabular definieren (Namen -> Hash)
            long sensorHeat = BioAI.createToken("SENSOR_HEAT", BioAI.CLUSTER_OBJECT);
            long actionFan  = BioAI.createToken("ACTION_FAN",  BioAI.CLUSTER_ACTION);

            // 3. Instinkt injizieren (Safety Rule)
            // "Wenn HITZE, dann LÜFTER AN" (Gewicht 1.0 = Gesetz)
            brain.forceInstinct(sensorHeat, actionFan, 1.0f);

            // 4. Input verarbeiten (Denken)
            long decision = brain.think(sensorHeat);

            // 5. Handeln
            if (decision == actionFan) {
                System.out.println("Decision: FAN ON (Correct)");
                // Belohnung geben (Verstärkung)
                brain.learn(1.0f, decision);
            } else {
                System.out.println("Decision: UNKNOWN");
            }

            // Optional: Speichern
            brain.saveToFile("brain_backup.bin");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
```

-----

## 4\. Advanced Concepts

### Memory Management (WICHTIG\!)

Da BioAI im **unmanaged Heap** (außerhalb der Java VM) läuft, greift der Garbage Collector hier nicht direkt.

  * Nutzen Sie immer `try-with-resources` (wie im Beispiel oben).
  * Oder rufen Sie manuell `brain.close()` auf, wenn das Objekt nicht mehr benötigt wird.

### Production Mode (ISO Compliance)

Für zertifizierte Umgebungen können Sie das Lernen einfrieren.

```java
// Deaktiviert malloc() im C-Core komplett.
// Maximale Stabilität für 24/7 Betrieb.
brain.setMode(BioAI.BioMode.PRODUCTION);
```

### Simulation ("Imagination")

Bevor eine kritische Aktion ausgeführt wird, kann der Agent die Konsequenzen simulieren.

```java
// Simuliert 5 Schritte in die Zukunft
long futureOutcome = brain.simulate(inputs, 5);
```

-----

## 5\. API Referenz

| Java Methode | Native Entsprechung | Beschreibung |
| :--- | :--- | :--- |
| `think(long...)` | `API_Update` | Verarbeitet Inputs in **O(1)**. |
| `simulate(..., depth)` | `API_Simulate` | Rekursive Kausalitäts-Prüfung. |
| `learn(reward, action)` | `API_Feedback` | Hebbian Learning auf Trace. |
| `forceInstinct(...)` | `API_Teach` | Schreibt Regeln ins LTM. |
| `inspect(...)` | `API_Inspect` | Debugging von Synapsen-Gewichten. |
| `saveToFile(path)` | `API_Serialize` | Speichert den kompletten Zustand. |
| `loadFromFile(path)` | `API_Deserialize` | Lädt einen Zustand wiederher. |

-----

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.6 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
