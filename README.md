# EV Thermal & Battery Control System

A software-defined simulation of an electric vehicle battery thermal management 
system, built in C++ with Python visualization. Modeled after real automotive 
control system architecture.

## Project Overview

Modern EVs rely on sophisticated thermal management to keep battery packs within 
safe operating temperatures. This project simulates that system end-to-end:

- **Sensor layer** — simulates battery temp, ambient temp, SoC, vehicle speed
- **Control algorithm** — state-machine-based thermal controller (NORMAL / COOLING / HEATING / FAULT / SAFE_SHUTDOWN)
- **CAN communication** — simulated CAN bus message passing between controllers
- **Diagnostics** — fault detection, DTC logging, fault injection testing
- **Visualization** — real-time Python dashboard of system state

## Architecture

Sensor Layer → Control Algorithm → Actuator Layer
↕
CAN Bus (simulated)
↕
Diagnostics & DTC Logger
## Tech Stack

| Tool | Purpose |
|------|---------|
| C++17 | Core control logic, simulation engine |
| MSVC / CMake | Build system |
| Google Test | Unit + integration testing |
| Python 3 + Pandas + Plotly | Visualization dashboard |
| Git | Version control |

## Build Instructions

```powershell
# Clone the repo
git clone https://github.com/simbarashemamvura1/EVThermalControl.git
cd EVThermalControl

# Build (Developer PowerShell for VS 2022)
mkdir build
cl /std:c++17 src/main.cpp /Fe:build/main.exe

# Run
.\build\main.exe
```

## Project Status

| Module | Status |
|--------|--------|
| Core data structs | ✅ Complete |
| Sensor simulator | 🔄 In progress |
| Thermal controller | ⬜ Planned |
| CAN simulation | ⬜ Planned |
| Diagnostics | ⬜ Planned |
| Python dashboard | ⬜ Planned |

## Why This Project

Built to demonstrate embedded controls engineering skills relevant to automotive 
software roles — specifically: C++ systems design, physics-based modeling, 
communication protocols, diagnostics strategies, and SIL-style testing.
