
# Contributing to BioAI

Thank you for your interest in BioAI! 🧠

BioAI consists of two parts with strictly different contribution rules to protect the integrity and safety of the system.

## 🛑 PART A: The BioAI Core (Restricted)

The C-Core (`bioai_"Version".dll/so` and headers) is **Proprietary Technology** and the intellectual property of **BrainAI**.

* **Status:** **READ-ONLY**.
* **Policy:** We do **NOT** accept Pull Requests, modifications, or "optimizations" for the Core logic.
* **Why?** The Core is certified for specific safety standards (Determinism, Memory Safety, O(1)). Any external modification could break the strict industrial safety guarantees.
* **Bugs:** If you suspect a bug in the Core, please open an Issue describing the behavior. Do **not** try to fix it yourself in the source code.

**Any Pull Request attempting to modify the C-Core logic will be closed immediately.**

---

## 🟢 PART B: Wrappers & Tools (Open for Contribution)

The Wrappers (C#, Python, C++, Java, etc.) and Demo Apps are **Open Source** (MIT License). We welcome community contributions here to make BioAI accessible on more platforms.

### How to Contribute to Wrappers:

1.  **New Platforms:** You want to run BioAI on Rust, Go, or Swift? Feel free to create a new wrapper that interfaces with the compiled library.
2.  **Usability:** Improve the Python pip package or the NuGet integration.
3.  **Examples:** Create demo projects (e.g., for Arduino or Unity) that use the *existing* Core binary.

### Contribution Guidelines

1.  **Fork & Branch:** Create a feature branch for your wrapper/tool.
2.  **No Core Changes:** Ensure your PR does not touch the `libs/` directory.
3.  **Documentation:** If you add a wrapper, please include a `README` on how to use it.

---

## 🐛 Reporting Issues

If you find a bug, please use the Issue Tracker:

* **Category:** [Wrapper] or [Core-Behavior]
* **Description:** Clear description of the error.
* **Reproduction:** Code snippet using the public API.

---

## ⚖️ License Agreement

By submitting a Pull Request to the Wrappers or Tools, you agree that your contribution is licensed under the **MIT License**.
The BioAI Core remains strictly under the **Proprietary License** of BrainAI.

---

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.6 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.