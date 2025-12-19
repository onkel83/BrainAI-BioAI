using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using BrainAI.BioAI; // Dein Wrapper Namespace

namespace IndustrialHMI
{
    class Program
    {
        // --- KONFIGURATION ---
        const string BRAIN_FILE = "factory_master.brain";
        const string LOG_FILE = "machine.log";
        const string VOCAB_FILE = "vocab_master.txt";
        const int TICK_MS = 500; // Simulations-Geschwindigkeit

        // --- GLOBALS ---
        static BioBrain brain;
        static bool running = true;
        static Dictionary<ulong, string> tokenMap = new Dictionary<ulong, string>();

        // Simulation State (Die "echte Welt")
        static int beltPosition = 0;
        static bool hasWorkpiece = true;
        static bool isProcessed = false;
        static bool isVibrating = false;
        static bool isEmergency = false;

        // Tokens
        static ulong S_EMPTY, S_OBJECT, S_RAW, S_DONE, S_VIB, S_EMG;
        static ulong A_MOVE, A_STAMP, A_STOP, A_MAINT, A_RESET;

        static void Main(string[] args)
        {
            SetupConsole();
            Log("SYS", "SYSTEM START - Loading Factory Simulation...");

            try
            {
                // 1. Initialisierung & Instinkte laden
                InitBioAI();
                BioClusters.DumpVocabulary(VOCAB_FILE);

                while (running)
                {
                    HandleInput();

                    // 2. SENSOREN LESEN
                    var sensors = GetSensors();
                    string sensorLog = string.Join("+", sensors.Select(GetName));

                    // 3. THINK (Die KI entscheidet)
                    ulong actToken = brain.Think(sensors.ToArray());
                    string actName = GetName(actToken);

                    // 4. PREDICTION & CONFIDENCE (Optional zur Anzeige)
                    ulong predToken = brain.Simulate(sensors.ToArray(), 1);
                    float confidence = (sensors.Count > 0) ? brain.Inspect(sensors[0], actToken) : 0f;

                    Log("CORE", $"In:[{sensorLog}] -> Act:{actName} (Conf: {confidence:P0})");

                    // 5. SOP (Notfall-Plan bei Vibration)
                    if (isVibrating && brain.GetPlanStep() == -1)
                    {
                        // Lädt festen Plan: STOP -> WARTUNG -> RESET
                        brain.LoadPlan(new ulong[] { A_STOP, A_MAINT, A_RESET }, true);
                    }

                    // 6. PHYSIK AUSFÜHREN & LERNEN
                    // Die KI bekommt sofort Feedback für ihre Aktion
                    float reward = RunPhysics(actToken);
                    brain.Learn(reward, actToken);

                    // 7. VISUALISIERUNG
                    RenderHMI(actName, GetName(predToken), confidence, reward);
                    Thread.Sleep(TICK_MS);
                }

                SaveAndExit();
            }
            catch (Exception ex)
            {
                // Falls doch was schiefgeht: Roten Bildschirm zeigen
                Console.BackgroundColor = ConsoleColor.DarkRed;
                Console.ForegroundColor = ConsoleColor.White;
                Console.Clear();
                Console.WriteLine("\n!!! CRITICAL ERROR !!!");
                Console.WriteLine(ex.Message);
                Log("FATAL", ex.ToString());
                Console.ReadKey();
            }
        }

        static void InitBioAI()
        {
            // Brain immer frisch starten für die Demo (keine Altlasten)
            if (File.Exists(BRAIN_FILE)) File.Delete(BRAIN_FILE);

            brain = new BioBrain(0xCAFE); // Neuer Seed
            Log("SYS", "BioBrain initialized.");

            // Tokens registrieren (Mapping für Anzeige)
            S_EMPTY = Reg("S_EMPTY", BioClusters.OBJECT);
            S_OBJECT = Reg("S_OBJECT", BioClusters.OBJECT);
            S_RAW = Reg("S_POS_2_RAW", BioClusters.OBJECT);
            S_DONE = Reg("S_POS_2_DONE", BioClusters.OBJECT);
            S_VIB = Reg("S_VIBRATION", BioClusters.OBJECT);
            S_EMG = Reg("S_EMERGENCY", BioClusters.OBJECT);

            A_MOVE = Reg("ACT_MOVE", BioClusters.ACTION);
            A_STAMP = Reg("ACT_STAMP", BioClusters.ACTION);
            A_STOP = Reg("ACT_STOP", BioClusters.ACTION);
            A_MAINT = Reg("ACT_MAINT", BioClusters.ACTION);
            A_RESET = Reg("ACT_RESET", BioClusters.ACTION);

            // === DAS BASISSET (REFLEXE) ===
            // Das sorgt dafür, dass die KI sofort funktioniert.

            // Regel 1: Rohling an Position 2 -> STANZEN
            brain.ForceInstinct(S_RAW, A_STAMP, 1.0f);

            // Regel 2: Fertiges Teil an Position 2 -> WEGFAHREN
            brain.ForceInstinct(S_DONE, A_MOVE, 1.0f);

            // Regel 3: Sonstiges Objekt -> FÖRDERN
            brain.ForceInstinct(S_OBJECT, A_MOVE, 1.0f);

            // Regel 4: Leerlauf -> FÖRDERN (Band läuft weiter)
            brain.ForceInstinct(S_EMPTY, A_MOVE, 1.0f);

            // Regel 5: Notfälle
            brain.ForceInstinct(S_VIB, A_MAINT, 1.0f);
            brain.ForceInstinct(S_EMG, A_STOP, 1.0f);

            Log("SYS", "Basisset injected successfully.");
        }

        // --- PHYSIK-SIMULATION ---
        static float RunPhysics(ulong action)
        {
            float r = 0.0f;

            // 1. Not-Aus
            if (isEmergency) return (action == A_STOP) ? 1.0f : -5.0f;

            // 2. SOP (Wartungsmodus)
            if (brain.GetPlanStep() != -1)
            {
                if (action == A_RESET) isVibrating = false;
                return 0.5f;
            }

            // 3. Förderband-Logik
            if (action == A_MOVE)
            {
                // Sicherheitsregel: Rohling darf nicht ungestanzt weiterfahren!
                if (beltPosition == 2 && !isProcessed)
                {
                    r = -2.0f; // Strafe
                }
                else
                {
                    beltPosition++;
                    // Zyklus-Ende: Band ist rund
                    if (beltPosition > 4)
                    {
                        beltPosition = 0;
                        hasWorkpiece = true; // Neues Teil kommt
                        isProcessed = false; // Ist wieder roh
                        r = 1.0f; // Belohnung für fertigen Zyklus
                    }
                    else
                    {
                        r = 0.1f; // Kleines Lob für Fluss
                    }
                }
            }
            else if (action == A_STAMP)
            {
                // Stanzen nur sinnvoll an Pos 2 bei Rohling
                if (beltPosition == 2 && !isProcessed)
                {
                    isProcessed = true;
                    r = 2.0f; // Großes Lob
                }
                else
                {
                    r = -1.0f; // Luft stanzen oder doppelt stanzen ist schlecht
                }
            }
            else if (action == A_STOP)
            {
                r = -0.1f; // Grundloses Stoppen kostet Geld
            }
            else
            {
                r = -0.1f; // Unbekannte Aktion
            }
            return r;
        }

        static List<ulong> GetSensors()
        {
            var list = new List<ulong>();
            if (isEmergency) list.Add(S_EMG);
            else if (isVibrating) list.Add(S_VIB);
            else
            {
                if (!hasWorkpiece) list.Add(S_EMPTY);
                else
                {
                    // WICHTIG: Sensor unterscheidet an Pos 2 zwischen RAW und DONE
                    if (beltPosition == 2) list.Add(isProcessed ? S_DONE : S_RAW);
                    else list.Add(S_OBJECT);
                }
            }
            return list;
        }

        static void RenderHMI(string act, string pred, float conf, float rew)
        {
            Console.SetCursorPosition(0, 0);
            Header(" BIOAI INDUSTRIAL CORE (RUNNING) ");

            Console.Write(" STATE: ");
            if (isEmergency) ColorPrint("EMERGENCY", ConsoleColor.Red);
            else if (isVibrating) ColorPrint("MALFUNCTION", ConsoleColor.Yellow);
            else ColorPrint("RUNNING", ConsoleColor.Green);

            Console.WriteLine($" | ACT: {act,-10} (Conf: {conf:P0})");
            Console.WriteLine(new string('-', 60));

            Console.Write(" IN [ ");
            for (int i = 0; i < 5; i++)
            {
                if (i == beltPosition && hasWorkpiece)
                {
                    if (isProcessed) ColorPrint("(OK) ", ConsoleColor.Green); // Fertig
                    else ColorPrint("(##) ", ConsoleColor.White); // Rohling
                }
                else Console.Write(" ..  ");
            }
            Console.WriteLine("] OUT");

            string arm = (act == "ACT_STAMP") ? "  ↓↓  " : "      ";
            Console.WriteLine($"      {arm}");
            Console.WriteLine($"      [ROBOT]   Rew: {rew:+0.0}");

            Console.WriteLine(new string('-', 60));
            Console.WriteLine("[V]ib (Störung) | [N]ot-Aus | [Q]uit");
        }

        // --- HELPER FUNKTIONEN ---
        static ulong Reg(string n, ulong c)
        {
            ulong id = BioClusters.CreateToken(n, c);
            if (!tokenMap.ContainsKey(id)) tokenMap[id] = n;
            return id;
        }
        static string GetName(ulong id) => tokenMap.ContainsKey(id) ? tokenMap[id] : $"UNK_{id:X}";

        static void Log(string t, string m)
        {
            try { File.AppendAllText(LOG_FILE, $"{DateTime.Now:HH:mm:ss}\t{t}\t{m}\n"); } catch { }
        }

        static void ColorPrint(string t, ConsoleColor c)
        {
            Console.ForegroundColor = c; Console.Write(t); Console.ResetColor();
        }

        static void Header(string t)
        {
            Console.BackgroundColor = ConsoleColor.DarkBlue;
            Console.ForegroundColor = ConsoleColor.White;
            Console.WriteLine(t.PadRight(60));
            Console.ResetColor();
        }

        static void HandleInput()
        {
            if (Console.KeyAvailable)
            {
                var k = Console.ReadKey(true).Key;
                if (k == ConsoleKey.Q) running = false;
                if (k == ConsoleKey.V) isVibrating = !isVibrating;
                if (k == ConsoleKey.N) isEmergency = !isEmergency;
            }
        }

        static void SetupConsole()
        {
            Console.Clear();
            Console.ForegroundColor = ConsoleColor.Gray;
        }

        static void SaveAndExit()
        {
            if (brain != null)
            {
                var d = brain.Serialize();
                if (d != null) File.WriteAllBytes(BRAIN_FILE, d);
                brain.Dispose();
            }
        }
    }
}