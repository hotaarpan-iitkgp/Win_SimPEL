# Win_SimPEL - High-Performance Windows C++ Circuit Simulator & Interactive Schematic Editor

**Win_SimPEL** is a standalone, ultra-fast C++ Windows desktop application for power electronics schematic editing, signal routing, and real-time circuit simulation.

---

## Key Features

- ⚡ **Native C++ Performance**: Built with Dear ImGui, DirectX 11 / OpenGL, and CMake for high FPS rendering and instant interaction.
- 📐 **100% Orthogonal Routing Engine**: Automatic $90^\circ$ right-angle wire routing with port-axis stubs, 4-way orthogonal segment dragging, and dynamic collinear path simplification.
- ⚡ **Control & Power Domain Isolation**: Enforces domain separation between control signal lines (`#38bdf8` cyan) and electrical power lines (`#00e678` green).
- 🔄 **Bidirectional Web Tool JSON Compatibility**: Full schema compatibility (`w1`, `w7`, `W1`, `"type": "junction"`, `"wireId"`) with the SimPEL Web application.
- 🎛️ **Dynamic Component Configuration**: Supports configurable multi-pin math blocks (`SUM_RECT`, `SUM_ROUND`, `PRODUCT_RECT`), Pulse Generators, PID Controllers, MOSFETs, and custom C-Script execution blocks.
- 🔍 **Mouse-Centered Focal Point Zoom**: Smooth, mouse-anchored canvas zooming ($0.15\times$ to $6.0\times$) and pan navigation.
- 🌊 **Real-Time Particle Flow Animation**: Live particle current/signal flow overlays reflecting net domain states.

---

## Building from Source

### Prerequisites
- Visual Studio 2022 / MSVC (with C++ Desktop Workload)
- CMake 3.20+

### Build Instructions
Run the automated build script:
```cmd
build.bat
```
Or build manually via CMake:
```cmd
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

The compiled binary will be generated at:
`build/circuitsim_pro_win.exe`

---

## Repository
- **GitHub**: [https://github.com/hotaarpan-iitkgp/Win_SimPEL.git](https://github.com/hotaarpan-iitkgp/Win_SimPEL.git)
