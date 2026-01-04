import asyncio
from asyncua import ua, Server
from bioai import BioBrainInstance

class BioOpcSortingServer:
    # --- 1. TOKEN DEFINITIONEN (Die 6 Schritte der Sortierung) ---
    # ACTIONS: Prozessschritte
    T_SCAN_BARCODE  = 0x2000000000000001
    T_WEIGHT_CHECK  = 0x2000000000000002
    T_SIZE_CHECK    = 0x2000000000000003
    T_ROUTE_A       = 0x2000000000000004
    T_ROUTE_B       = 0x2000000000000005
    T_LOG_EXIT      = 0x2000000000000006

    # OBJECTS & REFLEXES
    T_HAND_DETECTED  = 0x1000000000000001 # Sensor-Input
    T_EMERGENCY_STOP = 0x4010000000000001 # Hard-Reflex (Safety)

    def __init__(self, key_path: str, dll_path: str):
        # Initialisierung des BioAI Kerns
        self._brain = BioBrainInstance(key_path, dll_path)
        self._initialize_bio_logic()
        
        # OPC UA Server Setup
        self.server = Server()
        self.input_node = None
        self.output_node = None

    def _initialize_bio_logic(self):
        """Injektion der industriellen Sicherheits-Logik."""
        self._brain.set_mode(0) # Training

        # Ebene 1: Instinkte - Wenn Hand erkannt, dann SOFORT-STOPP
        # Gewicht 1.0 = Höchste Priorität (Safety First)
        self._brain.teach(self.T_HAND_DETECTED, self.T_EMERGENCY_STOP, 1.0)

        # Produktion einfrieren für maximale Stabilität
        self._brain.set_mode(1)
        
        # 6-Schritt-Sequenz als Standard-Prozess laden
        # In v0.7.6 wird dies über die Inferenz-Logik gesteuert.
        print("[BioAI] Sorting-Kernel stabilisiert und im Production Mode.")

    async def init_server(self):
        """Initialisiert den OPC UA Adressraum."""
        await self.server.init()
        self.server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
        
        # Namespace erstellen
        uri = "http://brainai.io/bioai/sorting"
        idx = await self.server.register_namespace(uri)

        # Objekt-Ordner für die SPS-Kommunikation
        root = await self.server.nodes.objects.add_folder(idx, "SortingAI")

        # --- INPUT NODE (SPS schreibt hierher) ---
        self.input_node = await root.add_variable(idx, "PLC_Sensor_Input", 0, ua.VariantType.UInt64)
        await self.input_node.set_writable() # SPS darf schreiben

        # --- OUTPUT NODE (SPS liest von hier) ---
        self.output_node = await root.add_variable(idx, "AI_Action_Output", 0, ua.VariantType.UInt64)
        
        print("[OPC UA] Server gestartet. Endpunkt: opc.tcp://localhost:4840")

    async def run(self):
        """Überwachungsschleife: Verarbeitet SPS-Inputs in Echtzeit."""
        async with self.server:
            while True:
                # 1. Wert von der SPS lesen
                plc_token = await self.input_node.get_value()
                
                if plc_token != 0:
                    # 2. BioAI 'Think' Zyklus in O(1)
                    # Wandelt SPS-Signal in optimale nächste Aktion um
                    action = self._brain.update([plc_token])

                    # 3. Ergebnis für die SPS bereitstellen
                    await self.output_node.set_value(ua.Variant(action, ua.VariantType.UInt64))

                    if action == self.T_EMERGENCY_STOP:
                        print("!!! [OPC UA] REFLEX STOPP AUSGELÖST !!!")

                await asyncio.sleep(0.01) # 10ms Zykluszeit

    def shutdown(self):
        self._brain.close()

# --- Hauptprogramm ---
if __name__ == "__main__":
    controller = BioOpcSortingServer("bin/key.json", "bin/BioAI_ULTRA.dll")
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(controller.init_server())
        loop.run_until_complete(controller.run())
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        print("[System] BioAI OPC UA Server beendet.")