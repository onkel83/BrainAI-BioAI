# Security Policy

BioAI is designed for **safety-critical edge applications** (Industrial IoT, Robotics, Smart Grids). Therefore, we treat security vulnerabilities not just as software bugs, but as potential physical safety risks.

## 🛡️ Supported Versions

We currently support security updates for the following versions of the BioAI Core and Wrappers:

| Version | Supported | Notes |
| :--- | :--- | :--- |
| **v0.7.5 (Industrial Closed Feature)** | ✅ | Current Stable Release |
| < 0.7.5 | ❌ | Deprecated. Do not use in production. |

---

## 🚨 Reporting a Vulnerability

**Do NOT open a public GitHub Issue for security vulnerabilities.**
If you have discovered a vulnerability that could compromise the integrity, safety, or availability of a system running BioAI, please report it privately.

### How to Report
Please send an email to the Lead Architect:
* **Email:** [koehne83@googlemail.com](mailto:koehne83@googlemail.com)
* **Subject:** `[SECURITY] BioAI Vulnerability Report`

### What to Include
Please provide as much detail as possible:
1.  **Component:** Is it in the *Open Source Wrapper* (C#/Python/Java/JS) or the *Proprietary C-Core* (`bioai.dll` / `.so`)?
2.  **Severity:** Can it bypass the *Safety Reflex Layer*? Does it cause a crash (DoS)? Can it inject false Tokens?
3.  **Proof of Concept:** A minimal code snippet, a malicious brain-dump file, or a description to reproduce the issue.

### Our Response Process
1.  **Acknowledgment:** We will acknowledge your report within 48 hours.
2.  **Verification:** We will verify the vulnerability internally.
3.  **Patching:**
    * **Wrappers:** We will push a fix to the public repo immediately.
    * **Core:** We will patch the proprietary binary and release a new version (e.g., v0.5.2).
4.  **Disclosure:** Once the patch is available to customers/users, we will credit you (if desired) in the release notes.

---

## 🔒 Specific Security Scope

### 1. The C-Core (Binary)
The Core operates in **Fixed Structure Mode** (No `malloc`/`free` during runtime) to prevent memory corruption attacks.

* **Critical:** Any method to trigger a **Buffer Overflow** or **Memory Leak** in the Core (especially via `API_Deserialize`) is considered a Critical Severity issue.
* **Critical:** Any method to bypass a **ForceInstinct (Reflex)** is considered a Critical Safety Violation.
* **High:** Breaking the **O(1) Real-Time Guarantee** (e.g., by forcing the engine into an infinite loop or excessive calculation time) constitutes a Denial-of-Service (DoS) against the physical control loop.

### 2. The Wrappers (Source)
The wrappers handle the interface between the OS and the Core.

* **High:** Vulnerabilities that allow **Token Injection** (spoofing sensor data) via the API boundaries.
* **Medium:** DLL-Hijacking vulnerabilities in the library loading mechanism.

---

## ⚠️ Disclaimer on "Safety" vs. "Security"

BioAI distinguishes between **Safety** (preventing harm to the environment) and **Security** (preventing malicious access).
However, in our architecture, a *Security* breach (e.g., modifying the LTM weights via an exploit) immediately becomes a *Safety* risk (Robot ignoring a stop signal).

**We treat all Security reports with the highest priority.**

---

**BrainAI Security Team**
