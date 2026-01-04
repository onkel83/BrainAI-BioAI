# 🛡️ BioAI Sovereign Security & Identity Guide

Dieses Verzeichnis enthält die fertigen Rechenkerne (Binaries) und Ihre individuellen Sicherheitsschlüssel. Mit der Version **0.7.6** hat BioAI das Konzept klassischer Seriennummern durch mathematische Integrität ersetzt.

## 1. Wo finde ich meinen Schlüssel?

Nach jedem Build-Vorgang generiert das System automatisch zwei Dateien im `bin`-Ordner, die Ihre Identität enthalten:

* **`key.json`**: Die maschinenlesbare Version des Schlüssels. Wrapper (C#, Java, Python etc.) lesen diesen Schlüssel automatisch ein, um das Gehirn zu initialisieren.
* **`License_Key.txt`**: Eine menschenlesbare Kopie des Schlüssels zur Dokumentation oder für manuelle Konfigurationen.

## 2. Warum gibt es keine „Seriennummer“ mehr?

Klassische Seriennummern dienen meist nur der Freischaltung einer Software über einen Server. BioAI verfolgt einen sichereren, dezentralen Weg: **Sovereign Security**.

1. **Mathematische Verankerung**: Der Schlüssel wird während des Builds direkt in den C-Code (`BioAI_Key.h`) injiziert. Er ist kein „Passwort“, sondern ein fester Bestandteil der Berechnungslogik.
2. **Daten-Integrität (Salting)**: Alle gelernten neuronalen Gewichte werden mit diesem Schlüssel mathematisch „gesalzen“. Ohne den exakt passenden Schlüssel in der `key.json` liefert die Engine zwar Ergebnisse, diese sind jedoch mathematisch verfälscht und unbrauchbar.
3. **Cloud-Unabhängigkeit**: Da die Sicherheit in der Mathematik der Daten liegt, benötigt BioAI keinen Lizenzserver. Das System bleibt auch in isolierten Netzen (Air-Gapped) oder auf Bare-Metal-Hardware sicher.

## 3. Die „Lock and Key“ Logik

Betrachten Sie das System als ein Schloss, das während des Kompilierens um einen spezifischen Schlüssel herum gebaut wurde.

* **Das Schloss**: Die DLL/SO im `bin`-Ordner.
* **Der Schlüssel**: Die `key.json`.
* **Der Schutz**: Ein mit Schlüssel A trainiertes Modell kann niemals mit einem Gehirn von Schlüssel B geladen werden.

---

### ⚠️ Wichtige Sicherheitshinweise

* **Verlust des Schlüssels**: Wenn Sie die `key.json` verlieren und keine Kopie der `License_Key.txt` haben, können Sie Ihre gespeicherten Gehirn-Zustände (`.bin` Dateien) nicht wiederherstellen.
* **Geheimhaltung**: Teilen Sie niemals Ihre `key.json` oder die Binaries zusammen mit dem Schlüssel öffentlich. Jeder, der Zugriff auf beide Komponenten hat, kann Ihr trainiertes Modell de-salten und analysieren.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.