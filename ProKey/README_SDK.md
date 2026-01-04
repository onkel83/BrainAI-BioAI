# ProKey SDK - Integration Guide

Thank you for choosing **ProKey**. This SDK provides hardware-near entropy generation and key streaming capabilities.

## 1. Requirements

* **Windows:** Visual Studio 2015 or later (x64)
* **Linux:** GCC / Clang (x64)
* **Android:** Clang (ARM64/aarch64)

## 2. Integration

### C++ (Recommended)
Include the header from the `include/` folder and link against the library in `lib/` (Windows) or `bin/` (Linux).

```cpp
#include "include/ProKey.hpp"

int main() {
    try {
        // Generate and print key
        prokey::Generator::toStream(std::cout);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}

```

### C (Native)

Use `include/prokey.h`.

---

## 3. How to Compile & Link

### Windows (Visual Studio)

You need `prokey.lib` (Linker) and `prokey.dll` (Runtime).

1. Add `include/` to your **Include Directories**.
2. Add `lib/` to your **Library Directories**.
3. Add `prokey.lib` to **Linker -> Input -> Additional Dependencies**.
4. **Important:** Copy `bin/prokey.dll` to the same folder where your `.exe` is located.

### Linux (GCC)

You need `libprokey.so`.

```bash
gcc main.c -I./include -L./bin -lprokey -Wl,-rpath=./bin -o myapp

```

* `-L./bin`: Tells the linker where to find the library.
* `-lprokey`: Links against `libprokey.so`.
* `-Wl,-rpath=./bin`: Tells the executable to look in `./bin` at runtime.

---

## 4. Licensing

**This is commercial software.**
If you generate revenue with software using ProKey, you must acquire a license.
See `LISENSE.md` for details.
