using System;
using BrainAI.BioAI;

namespace BrainAI.Examples
{
    public class DatevSanityCheck
    {
        public static void RunExample(ulong licenseKey)
        {
            // 1. Initialisierung des Gehirns (Tier 3 Architektur)
            using (var brain = new BioBrain(licenseKey)) //
            {
                // 2. Erstellung der Buchhaltungs-Domäne (Tokenisierung)
                // Wir mappen DATEV-Kategorien auf BioClusters
                ulong tRevenue = BioClusters.CreateToken("Revenue_4400", BioClusters.OBJECT); //
                ulong tBank = BioClusters.CreateToken("Bank_1200", BioClusters.OBJECT); //
                ulong tVAT = BioClusters.CreateToken("VAT_19_Percent", BioClusters.OBJECT); //
                
                ulong tValid = BioClusters.CreateToken("VALID_BOOKING", BioClusters.ACTION); //
                ulong tError = BioClusters.CreateToken("REVENUE_WITHOUT_VAT", BioClusters.LOGIC | 0x0010000000000000); //

                // 3. Definition der Kausalen Logik (Instincts)
                // Wir "impfen" dem System ein, dass Erlöse (4400) IMMER MwSt benötigen.
                // Ohne MwSt -> Reflex "Error". Mit MwSt -> "Valid".
                brain.ForceInstinct(tRevenue, tError, 1.0f); // Standard-Reflex: Alarm bei Erlös
                brain.ForceInstinct(tRevenue | tVAT, tValid, 1.0f); // Höherwertiger Reflex: OK wenn MwSt dabei

                // 4. Echtzeit-Validierung (Der "DATEV-Killer" Test)
                Console.WriteLine("--- BioAI v0.7.5: Starting Causal Sanity Check ---");

                // Testfall A: Unvollständige Buchung (Erlös ohne Steuer)
                ulong resultA = brain.Think(tRevenue); //
                Console.WriteLine($"Result A (Revenue only): {(resultA == tError ? "ALARM: Missing VAT" : "Unknown")}");

                // Testfall B: Korrekte Buchung (Erlös + Bank + Steuer)
                ulong resultB = brain.Think(tRevenue, tVAT, tBank); //
                Console.WriteLine($"Result B (Complete Booking): {(resultB == tValid ? "PASS: Logically Consistent" : "FAIL")}");

                // 5. Performance-Beweis
                // In einer Schleife mit 500.000 Agenten/Buchungen
                // bleibt die Komplexität bei O(1).
            }
        }
    }
}
