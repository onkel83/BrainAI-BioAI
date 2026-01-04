# ProKey SDK 
### True Hardware Entropy. Zero Friction.

**Version:** 0.1.0  
**License:** Commercial / Free for Non-Profit  
**Website:** [https://github.com/onkel83/brainai]

---

## 📖 Über ProKey

**ProKey** ist eine hochleistungsfähige, hardware-nahe Engine zur Generierung von echter Entropie und kryptografisch sicheren Keys. 

Im Gegensatz zu herkömmlichen Lösungen, die oft nur auf deterministischen Algorithmen basieren, greift ProKey direkt auf die **physikalischen Eigenschaften der Hardware** zu. Wir bieten eine einheitliche API, egal ob Sie auf einem High-End Server, einem Android-Tablet oder einem Embedded-System arbeiten.

### 🚀 Warum ProKey? Die realistische Lösung.

Warum sollten Sie ProKey statt `rand()`, `System.Random` oder komplexen Krypto-Libraries nutzen?

| Feature | Standard Libs (rand/Random) | OpenSSL / CryptoAPI | **ProKey SDK** |
| :--- | :--- | :--- | :--- |
| **Quelle** | Software (Mathematik) | OS-Pool (Software/Mixed) | **Pure Hardware** (CPU/ADC/Jitter) |
| **Sicherheit** | Niedrig (Vorhersagbar) | Hoch | **Maximum** (Physikalische Entropie) |
| **Integration** | Einfach | Komplex / Viel Code | **Plug & Play** (1 Zeile Code) |
| **Portabilität**| Gut | Variiert je OS | **Universal** (Embedded bis Server) |
| **Performance**| Sehr hoch | Mittel (Overhead) | **Hardware-Speed** (Zero-Copy) |

**Das Problem:** Software-Zufallsgeneratoren sind deterministisch. Wenn man den "Seed" kennt, kennt man alle zukünftigen Zahlen.
**Die ProKey Lösung:** Wir nutzen Quanteneffekte (Intel Secure Key/RDRAND), thermisches Rauschen (ADC) und Timing-Jitter, um Unvorhersehbarkeit auf physikalischer Ebene zu garantieren.

---

## 🛠 Unterstützte Sprachen & Wrapper

ProKey ist "Language Agnostic". Der Core ist in hocheffizientem C99 geschrieben. Wir liefern offizielle, gepflegte Wrapper für die wichtigsten Enterprise-Sprachen mit:

### 

[Image of C programming language logo]
 **Native C**
Der Kern des SDKs. Direkter Zugriff ohne Overhead.
* **Ideal für:** Treiber, Kernel-Module, Embedded Systems.
* **API:** `prokey.h`

###  **C++ (Modern)**
Ein "Header-Only" Wrapper mit RAII-Support, Exceptions und Streams.
* **Ideal für:** High-Performance Applications, Game Engines.
* **API:** `ProKey.hpp`

###  **C# / .NET**
Vollständige Integration in das .NET Ökosystem (Core, Framework, 5+). Exceptions statt Error-Codes.
* **Ideal für:** Enterprise Backend, Desktop Tools, Unity.
* **API:** `ProKey.SDK` Namespace

### 

[Image of Python logo]
 **Python**
Schlanker Wrapper basierend auf `ctypes`. Keine Pip-Installation nötig.
* **Ideal für:** Data Science, Scripting, Rapid Prototyping.
* **API:** `prokey.py`

### 

[Image of Java logo]
 **Java**
JNA-basierter Wrapper. Läuft überall dort, wo eine JVM läuft.
* **Ideal für:** Android Apps, Enterprise Server (Spring/Jakarta).
* **API:** `com.prokey.sdk`

---

## ⚙️ Under the Hood: Architektur

ProKey erkennt zur Laufzeit die CPU-Architektur und wählt automatisch die sicherste Entropiequelle:

1.  **x86_64 (Server/Desktop):** Direkter Zugriff auf die `RDRAND` Instruktion der CPU (Intel Secure Key). Umgeht das Betriebssystem für maximale Geschwindigkeit und Sicherheit.
2.  **AVR (Embedded):** Nutzung des thermischen Rauschens am Analog-Digital-Wandler (Floating Pin ADC Noise).
3.  **ARM / Generic:** Nutzung von "SRAM Remanence" (Speicher-Startzustände) kombiniert mit "Execution Jitter" (Ausnutzung von Clock-Drift und Interrupt-Latenzen).

---

## 📦 SDK Inhalt

```text
ProKey_SDK/
├── bin/                 # Kompilierte Binaries (DLL / SO)
│   ├── prokey.dll       # Windows x64
│   ├── libprokey.so     # Linux x64
│   └── libprokey_and.so # Android ARM64
├── lib/                 # Linker Libraries (Windows .lib)
├── include/             # C/C++ Header
├── bindings/            # Wrapper für andere Sprachen
│   ├── csharp/
│   ├── java/
│   └── python/
├── LICENSE.txt          # Lizenzbestimmungen
└── README.md            # Diese Datei

```

---

## ⚖️ Lizenzmodell

ProKey ist **proprietäre Software**. Der Quellcode des Cores ist geschlossen ("Closed Source").

> **"You make money, you pay"**

1. **Private & Non-Profit Nutzung:** Kostenlos. Sie dürfen das SDK frei in Open-Source-Projekten, für Bildung oder private Bastelprojekte nutzen.
2. **Kommerzielle Nutzung:** Sobald Sie ProKey in einem Produkt verwenden, mit dem Umsatz erzielt wird (direkt oder indirekt), ist eine **kommerzielle Lizenz** erforderlich.

Bitte kontaktieren Sie uns unter `koehne83@googlemail.com` für Lizenzanfragen.

---

© 2025 BrainAI Developers. All Rights Reserved.
