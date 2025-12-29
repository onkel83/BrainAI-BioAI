# 🛸 BioAI Technical Manual: Autonomous Drone Controller (0.7.6)

Dieses Handbuch beschreibt die Implementierung des **BioDroneController**, einer autonomen Flugsteuerungslogik, die Sensordaten in Echtzeit verarbeitet und sicherheitskritische Entscheidungen in **garantierter  Zeit** trifft. Die Architektur ist darauf ausgelegt, Hardware-Schäden durch "Safety-First"-Reflexe zu verhindern, selbst wenn die Haupt-Navigation ausfällt.

---

## 1. Die Flug-Ontologie (Vocab-Dump) 🟦🟥

In der BioAI-Umwelt werden physische Zustände auf 64-Bit **TokenIDs** abgebildet. Diese Struktur ermöglicht es dem Kern, komplexe Sensor-Szenarien ohne den Overhead klassischer If-Else-Kaskaden zu bewerten.

### Wahrnehmung (Inputs / OBJECT Cluster)

| Token | Hex-ID (Beispiel) | Beschreibung | Schwellenwert (Logic) |
| --- | --- | --- | --- |
| `T_DIST_FRONT_CLOSE` | `0x1000...0001` | Hindernis im Nahbereich erkannt. | < 1.5m |
| `T_BATTERY_LOW` | `0x1000...0002` | Kritischer Akkuladestand. | < 15% |
| `T_GPS_LOST` | `0x1000...0003` | Verlust des Satelliten-Fix. | Boolean False |

### Reaktionen (Actions & Reflexes)

| Token | Hex-ID (Beispiel) | Cluster | MAVLink / Hardware Kommando |
| --- | --- | --- | --- |
| `T_MOVE_FORWARD` | `0x2000...0001` | **ACTION** | `SET_POSITION_TARGET` (Vel: X+). |
| `T_HOVER` | `0x2000...0002` | **ACTION** | Geschwindigkeit auf Null setzen. |
| `T_EMERGENCY_LAND` | `0x4010...0001` | **REFLEX** | `CMD_NAV_LAND` (Sofortiger Abstieg). |
| `T_RTL` | `0x4010...0002` | **REFLEX** | `CMD_NAV_RETURN_TO_LAUNCH`. |

---

## 2. Implementierung: BioDroneController.py

```python
import time
from bioai import BioBrainInstance

class BioDroneController:
    """
    Python-Implementierung der autonomen Drohnensteuerung (v0.7.6).
    Mappt Sensor-Daten auf neuro-symbolische Reflexe.
    """

    # --- TOKEN DEFINITIONEN ---
    T_DIST_FRONT_CLOSE = 0x1000000000000001
    T_BATTERY_LOW      = 0x1000000000000002
    T_GPS_LOST         = 0x1000000000000003

    T_MOVE_FORWARD     = 0x2000000000000001
    T_HOVER            = 0x2000000000000002
    
    T_EMERGENCY_LAND   = 0x4010000000000001 # Höchste Priorität
    T_RTL              = 0x4010000000000002 # Sicherheits-Bias

    def __init__(self, json_path: str, dll_path: str):
        # 1. Initialisierung des Gehirns über den Wrapper
        self._brain = BioBrainInstance(json_path, dll_path)
        self._initialize_instincts()

    def _initialize_instincts(self):
        """Injektion von Sicherheits-Reflexen (Ebene 1)."""
        self._brain.set_mode(0) # Training

        # Reflex: Akku leer -> Sofort Landen (Gewicht 1.0 = Unverhandelbar)
        self._brain.teach(self.T_BATTERY_LOW, self.T_EMERGENCY_LAND, 1.0)
        
        # Reflex: Hindernis voraus -> Hover
        self._brain.teach(self.T_DIST_FRONT_CLOSE, self.T_HOVER, 1.0)

        # Bias: GPS weg -> Return to Launch (Gewicht 0.8)
        self._brain.teach(self.T_GPS_LOST, self.T_RTL, 0.8)

        self._brain.set_mode(1) # Produktion
        print("[DRONE] Fluglogik-Kernel stabilisiert und versiegelt.")

    def update_flight_logic(self, dist_front: float, batt_pct: float, has_gps: bool):
        """Hauptschleife zur Flugsteuerung."""
        active_inputs = []

        # 1. Sensor-Abstraktion
        if dist_front < 1.5: active_inputs.append(self.T_DIST_FRONT_CLOSE)
        if batt_pct < 15.0: active_inputs.append(self.T_BATTERY_LOW)
        if not has_gps:      active_inputs.append(self.T_GPS_LOST)

        # 2. Denken (O(1) Inferenz)
        decision = self._brain.update(active_inputs)

        # 3. Kommando-Ausführung
        self._execute_command(decision)

    def _execute_command(self, action_token: int):
        if action_token == self.T_EMERGENCY_LAND:
            print("[MAVLINK] !! EMERGENCY_LAND !! (Akkuschutz)")
        elif action_token == self.T_HOVER:
            print("[MAVLINK] SET_POSITION_TARGET (STOP)")
        elif action_token == self.T_RTL:
            print("[MAVLINK] CMD_NAV_RETURN_TO_LAUNCH (GPS-Failsafe)")
        elif action_token == self.T_MOVE_FORWARD:
            print("[MAVLINK] SET_POSITION_TARGET (FORW: 2.0m/s)")
        else:
            print("[MAVLINK] NO_ACTION (Wait for inputs)")

    def shutdown(self):
        self._brain.close()

```

---

## 3. Sicherheits-Architektur (ISS Standard)

Die Drohnensteuerung nutzt die **vierstufige Entscheidungshierarchie**, um Flugunfälle zu vermeiden:

1. **Safety-Interrupts**: Wenn `T_BATTERY_LOW` auftritt, erzeugt der Kern einen Interrupt. Da dieses Token mit dem Reflex `T_EMERGENCY_LAND` (Gewicht 1.0) verknüpft ist, wird jede andere Bewegung (z. B. Vorwärtsflug) sofort unterdrückt.
2. **Deterministische Latenz**: Unabhängig davon, ob 3 oder 30 Sensoren überwacht werden, erfolgt die Entscheidung immer in der gleichen Zeitspanne. Dies ist für Flugregler (Flight-Controller), die oft im 400Hz Takt arbeiten, lebensnotwendig.
3. **Black-Box Integrity**: Die gelernten Verhaltensmuster sind mathematisch verschlüsselt (Weight Salting). Ein unbefugtes Ändern der Fluglogik ("Hacking the Brain") ist ohne die `key.json` physikalisch nicht möglich.

---

## 4. Performance & Validierung

Im Rahmen der Build-Validierung (`Core_Tests.c`) wurde dieses Modell einem Stresstest unterzogen:

* **Kollisions-Vermeidung**: Erfolgreiches Auslösen des `T_HOVER` Reflexes bei Hindernis-Input innerhalb von < 1ms.
* **Kompaktheit**: Das gesamte Drohnen-Gehirn belegt weniger als 64 KB RAM (SmartHome Tier), was ideal für integrierte Flight-Control-Boards ist.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
