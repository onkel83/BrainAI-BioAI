---
name: Bug report
about: Create a report to help us improve
title: ''
labels: bug
assignees: onkel83

---

---
name: 🐛 Bug Report
about: Create a report to help us improve BioAI
title: "[BUG] "
labels: bug
assignees: ''
---

## 🚨 Security Warning
**Is this a security vulnerability?** (e.g., Buffer Overflow, Reflex Bypass)
* [ ] YES -> **STOP!** Do not post here. See `SECURITY.md` and email [koehne83@googlemail.com].
* [ ] NO -> Continue below.

---

## 💻 Environment Setup (Required)
**Hardware / Platform:**
 - Device: [e.g. Fire HD 10, Raspberry Pi 4, ESP32]
 - OS: [e.g. Android 11, Ubuntu 22.04, Windows 10]
 - CPU Architecture: [e.g. ARM64, x64]

**BioAI Version:**
 - Core Version: [e.g. v0.0.2 Alpha]
 - Wrapper Language: [e.g. C# (MAUI), Python, C++]

---

## 🧩 Component
Which part of the stack is failing?
- [ ] **Wrapper / SDK** (Source Code issues, API bindings)
- [ ] **Native Core** (Binary behavior, `libbioai.so` / `bioai.dll`)
- [ ] **Tools** (Installer, Vocabulary Dumper)

---

## 📝 Description
A clear and concise description of what the bug is.

**Expected Behavior:**
What did you expect to happen? (e.g., "The agent should trigger the reflex A_STOP when T_HEAT is active.")

**Actual Behavior:**
What actually happened? (e.g., "The agent continued A_MOVE, and Inspect() showed a weight of 0.0.")

---

## 🔄 Reproduction Steps
Please provide a minimal code snippet to reproduce the issue.

```csharp
// Example Code (C#)
var brain = new BioBrain(KEY);
ulong input = BioClusters.CreateToken("Test", BioClusters.OBJECT);
// ... triggers the crash
````

-----

## 🕵️‍♂️ Logs & Evidence

If applicable, add logs or screenshots.

  * **Stack Trace:** (If it crashed/segfaulted)
  * **Inspect Output:** If it's a logic error, what does `brain.Inspect(input, action)` return?
  * **Vocabulary Dump:** (Optional)

-----

## ⚖️ Disclaimer

I confirm that I have checked the documentation and that this is not a setup error (e.g., missing `libbioai.so` in the correct folder).

```
