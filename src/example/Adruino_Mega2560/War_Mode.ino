#include "BioAI_Adruino.h"

// --- KONFIGURATION ---
#define NEURON_COUNT 50  // Startwert für Stabilität (erhöhe auf 125 wenn RAM hält)
#define SERIAL_SPEED 115200

// Globale Instanzen
BioAI::Agent* agent;
uint64_t inputs[NEURON_COUNT];
uint64_t actions[NEURON_COUNT];

void setup() {
    Serial.begin(SERIAL_SPEED);
    while (!Serial); 
    
    Serial.println(F("\n--- BioAI v0.7.5 WAR-MODE BOOT ---"));

    // 1. Agent initialisieren (Heap-Allokation)
    agent = new BioAI::Agent(12345);
    Serial.println(F("Core geladen."));

    // 2. Neuronen & Reflexe "impfen"
    Serial.print(F("Initialisiere "));
    Serial.print(NEURON_COUNT);
    Serial.println(F(" Reflex-Pfade..."));

    for (int i = 0; i < NEURON_COUNT; i++) {
        // Effiziente Token-Erzeugung ohne großen String-Overhead
        char buf[8];
        itoa(i, buf, 10);
        
        inputs[i]  = BioAI::Agent::CreateToken(buf, BioAI::Cluster::Object);
        actions[i] = BioAI::Agent::CreateToken(buf, BioAI::Cluster::Action);

        // Direkte Kausal-Verknüpfung im Logic-Cluster
        agent->ForceInstinct(inputs[i], actions[i], 1.0f);
        
        // Fortschrittsanzeige alle 10 Neuronen
        if (i % 10 == 0) { Serial.print(F(".")); Serial.flush(); }
    }

    // 3. Modus auf Industrial Stable (Production)
    agent->SetMode(BioAI::BioMode::Production);

    Serial.println(F("\nSystem bereit. O(1) Modus aktiv."));
}

void loop() {
    // Hochpräzise Zeitmessung der Denk-Latenz
    uint32_t t1 = micros();

    // Der O(1) Denkprozess verarbeitet den gesamten Vektor
    uint64_t response = agent->Think(inputs, NEURON_COUNT);

    uint32_t t2 = micros();

    // Analyse & Output
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        uint32_t duration = t2 - t1;
        
        Serial.print(F("[WAR MODE] Latenz: "));
        Serial.print(duration);
        Serial.print(F(" us | Aktion: 0x"));
        Serial.print((uint32_t)(response >> 32), HEX); // Obere 32 Bit
        Serial.println((uint32_t)response, HEX);       // Untere 32 Bit
        
        lastPrint = millis();
    }
}