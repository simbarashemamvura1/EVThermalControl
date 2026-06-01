# EV Thermal Battery Control System

A software-defined EV battery thermal management system built in C++ with a live Python dashboard. Designed to demonstrate embedded control system architecture, sensor simulation, CAN bus communication, diagnostics, and real-time visualization — modeled after production automotive software patterns.

---

## Overview

This project simulates the thermal management system of an electric vehicle battery pack. It was built as a portfolio project targeting embedded controls engineering roles in the automotive industry, specifically demonstrating competency in areas directly relevant to battery management systems (BMS) development.

The C++ simulation engine runs continuously, processing sensor data through a state machine controller, logging CAN bus traffic, and streaming live JSON data to a Python dashboard over a TCP socket. The dashboard renders a real-time car visualization, live graphs, and a fault event log — all driven by the actual C++ control logic.

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│              C++ Simulation Engine                   │
│                                                      │
│  SensorSimulator → ThermalController → CANBus        │
│         ↓                ↓               ↓           │
│  DiagnosticsManager   ActuatorCmd    CSV Logs         │
│         ↓                                            │
│  SocketServer (port 9000) ← CommandServer (9001)     │
└──────────────────────┬──────────────────────────────┘
                       │ TCP JSON stream
┌──────────────────────▼──────────────────────────────┐
│              Python Dashboard (Dash/Flask)            │
│                                                      │
│  Socket Reader → State → /live-state endpoint        │
│       ↓              ↓          ↓                    │
│  Graphs         Stat Cards   ev_scene.html (iframe)  │
│  Fault Log      DTC Alert    Car Simulation (Canvas)  │
└─────────────────────────────────────────────────────┘
```

---

## C++ Modules

### SensorSimulator
Generates physics-based sensor readings every simulation tick. Models battery temperature, state of charge, voltage, and current with configurable Gaussian noise to mimic real sensor imprecision. Heat generation scales with vehicle speed and motor load. Thermal lag is modelled so temperature changes are gradual, not instantaneous.

### ThermalController
A five-state finite state machine that reads sensor data and outputs actuator commands every tick:

| State | Trigger | Action |
|---|---|---|
| `NORMAL` | Default | Fan off, pump off |
| `HEATING` | Battery temp < 10°C | Heater on |
| `COOLING` | Battery temp > 45°C | Fan proportional to temp, pump on |
| `FAULT` | Temp > 70°C or sensor dropout | Fan 100%, all alerts |
| `SAFE_SHUTDOWN` | Temp > 80°C | All actuators off, shutdown sequence |

Hysteresis thresholds prevent state chatter — for example, cooling does not deactivate the moment temperature drops by 1°C. The controller requires temperature to fall to 40°C before transitioning back to NORMAL from COOLING.

### CANBus
Simulates CAN bus communication with real frame structure — message IDs, DLC (data length code), and 8-byte payloads. Three frame types are encoded and transmitted each tick:

- `0x100` — Battery status (temperature, SoC, voltage, current)
- `0x101` — Thermal status (coolant temp, fan speed, pump/heater state)
- `0x200` — Actuator commands

All frames are logged to `build/can_log.csv` for post-run analysis.

### DiagnosticsManager
Manages Diagnostic Trouble Codes (DTCs) with full lifecycle management — raise, resolve, and clear. Maintains an active fault list and a complete fault history. Supports software-in-the-loop (SIL) fault injection for testing fault response paths. All DTC events are logged to `build/dtc_log.csv`.

Active DTC codes used in this simulation:

| Code | Severity | Description |
|---|---|---|
| `0x0001` | WARNING | Overtemperature warning |
| `0x0002` | CRITICAL | Battery temperature critically high |
| `0x0003` | WARNING | Battery undertemperature warning |
| `0x0004` | CRITICAL | Sensor reading out of physical bounds |
| `0x0005` | CRITICAL | Sensor dropout — data unreliable |
| `0x0006` | WARNING | State of charge low |
| `0x0007` | CRITICAL | State of charge critically low |
| `0x0008` | CRITICAL | System in SAFE SHUTDOWN — battery critically hot |

### SocketServer
TCP data server on port 9000. Broadcasts a JSON packet to the connected client every simulation tick (200ms). The JSON payload includes all sensor readings, actuator states, system state, active DTCs, real SoC, and the current scenario label.

### Command Server
TCP command receiver on port 9001. Accepts plain-text scenario commands from the Python dashboard (`cold`, `highway`, `fault`, `full`). On receipt, immediately stops any running scenario, resets all simulation state, and starts the new scenario in a background thread. The ThermalController state machine is reset to NORMAL instantly so the dashboard badge updates within one tick.

---

## Realistic Battery Physics

The simulation models a physically accurate state of charge drain with four separate components:

```cpp
float baseDrain   = 0.0008f;                                    // parasitic draw — always present
float driveDrain  = actualSpeed * 0.000055f;                    // speed-proportional motor load
float heatDrain   = (bT > 40.0f) ? (bT - 40.0f) * 0.00015f : 0.0f;  // internal resistance rises with temp
float coldDrain   = (bT < 10.0f) ? (10.0f - bT) * 0.0001f  : 0.0f;  // lithium-ion loses capacity when cold
float heaterDrain = heaterOn ? 0.003f : 0.0f;                  // cabin/battery heater draws significant power
```

This means:
- A parked vehicle with all systems idle drains negligibly
- A cold battery (-10°C) drains faster even at the same speed as a warm one
- A hot battery (above 40°C) drains faster due to increased internal resistance
- The heater is the single biggest drain during cold start scenarios
- High-speed driving (180 kph) produces the fastest discharge

---

## Unit Tests

22 unit tests across three test suites using Google Test:

**Sensor tests (6)** — Temperature rises with speed, SoC drains over time, SoC never goes negative, readings stay within physical bounds, higher speed produces more heat, voltage correlates with SoC.

**Controller tests (8)** — Normal conditions stay NORMAL, cooling activates above 45°C, heating activates below 10°C, fault state above 70°C, safe shutdown above 80°C, shutdown state is sticky, hysteresis prevents rapid switching, fan speed is proportional to temperature.

**Diagnostics tests (8)** — No faults in normal conditions, overtemp warning raises DTC, critical overtemp raises DTC, fault injection raises DTC, DTC resolves after duration, DTC resolves when condition clears, low SoC raises DTC, fault history grows correctly.

---

## Python Dashboard

Built with Dash and Plotly. Operates in two modes:

**C++ Engine Live** — Socket reader thread connects to port 9000 and streams real physics data from the C++ engine. Scenario buttons send commands to port 9001 which are processed by the C++ command server. All state transitions, DTCs, and sensor readings come from the actual C++ code.

**Python Fallback** — If the C++ engine is not running, a Python physics engine takes over automatically. Implements the same state machine logic, realistic drain model, and DTC generation so the dashboard remains fully functional for standalone demos.

The connection status is displayed in the header as a live badge — green **C++ Engine Live** or grey **Python Fallback**.

### Dashboard panels
- Real-time battery temperature graph with COOLING (45°C) and HEATING (10°C) threshold lines
- Fan speed graph showing proportional spin-up and smooth deceleration
- State of charge graph with Low SoC (15%) and Critical (5%) threshold lines — line turns red when SoC drops below 15%
- Fault event log showing all DTCs with tick timestamps
- Six sensor status indicators (Temp, SoC, Voltage, CAN Bus, Pump, Fan) that turn red during fault injection
- Active Diagnostic Codes alert box showing all currently active DTCs
- Battery Depleted alert that blocks scenario execution when SoC reaches 0%

---

## Car Scene

A Canvas2D WebGL car simulation rendered in an iframe. Polls `/live-state` every 300ms and smoothly interpolates toward the server values so animation remains fluid even at the 250ms dashboard update interval.

Features:
- Stationary sedan with world-scrolling parallax — road dashes, near trees, far trees, and clouds all scroll at different speeds proportional to vehicle speed
- Battery pack cutaway showing SoC fill bar (color shifts blue → teal → amber → red with temperature), temp and SoC% labels, and cell grid overlay
- Motor with heat-tinted glow and rotor animation
- Animated cooling pipe flow between battery and motor
- Four sensor dots (T, V, S, C) that pulse green in normal operation and flash red during fault injection
- Speed lines that appear behind the car above 50 kph and intensify with speed
- Engine vibration above 100 kph
- Sky tints red during FAULT and SAFE SHUTDOWN states
- Braking taillight brightens when decelerating
- Dead battery overlay message and red battery fill when SoC reaches 0%
- Low SoC warning label in the bottom-right corner below 15%

### Sound alerts
Six audio events fire on state transitions using the Web Audio API:

| Event | Sound | Trigger |
|---|---|---|
| Cooling activated | Rising twin tones | NORMAL → COOLING |
| Heating activated | Warm low tones | NORMAL → HEATING |
| Fault | Descending sawtooth | Any → FAULT |
| Safe shutdown | Long descending alarm | Any → SAFE SHUTDOWN |
| Low SoC | Triple beep | SoC crosses below 15% |
| Battery dead | Heavy descending tones | SoC reaches 0% |

---

## Simulation Scenarios

Four scenarios are available from the dashboard buttons. Each can be started at any time and will immediately reset the previous scenario:

**Cold Start** — Battery begins at -10°C. The heater activates, battery temperature rises gradually as the vehicle accelerates. Demonstrates HEATING state and cold-weather efficiency loss. The battery drains faster than normal during the heating phase.

**Highway Heat** — Vehicle accelerates to 180 kph. Battery temperature climbs past the cooling threshold, then the overtemp threshold, triggering FAULT and finally SAFE SHUTDOWN. Demonstrates the full thermal protection chain.

**Fault Injection** — Normal driving at 100 kph, then a sensor dropout is injected for 20 ticks. Sensor dots turn red, the system enters FAULT state, DTCs fire. After the injection window closes, the system recovers automatically.

**Full Auto Run** — Runs all three scenarios back to back: cold start warm-up, highway heat soak with cooling and shutdown, then fault injection and recovery. Designed for demo and interview recording.

---

## Project Structure

```
EVThermalControl/
├── CMakeLists.txt
├── README.md
├── src/
│   ├── types.h                   Core structs: BatteryState, ThermalState, SensorReading, CANFrame, DiagnosticCode
│   ├── SensorSimulator.h/.cpp    Physics-based sensor generation with Gaussian noise
│   ├── ThermalController.h/.cpp  Five-state FSM with hysteresis
│   ├── CANBus.h/.cpp             CAN frame encoding and CSV logging
│   ├── DiagnosticsManager.h/.cpp DTC lifecycle management and fault injection
│   ├── SocketServer.h/.cpp       TCP data server on port 9000
│   └── main.cpp                  Simulation loop, scenario runner, command server
├── tests/
│   ├── test_sensor.cpp           6 sensor simulation tests
│   ├── test_controller.cpp       8 state machine tests
│   └── test_diagnostics.cpp      8 diagnostics tests
├── scripts/
│   ├── dashboard.py              Dash dashboard with socket integration
│   └── assets/
│       └── ev_scene.html         Canvas2D car simulation with sound alerts
├── cmake-build/
│   └── Debug/
│       ├── EVThermalControl.exe
│       └── ev_tests.exe
└── build/
    ├── can_log.csv               CAN bus traffic log
    └── dtc_log.csv               DTC event log
```

---

## Build Instructions

**Requirements**
- Visual Studio 2022 with C++17 support
- CMake 3.14 or later
- Python 3.9+ with `dash`, `plotly`, `flask`

**Install Python dependencies**
```bash
pip install dash plotly flask
```

**Build C++ engine**
```powershell
# Open Developer PowerShell for VS 2022
cd EVThermalControl
mkdir cmake-build
cd cmake-build
cmake ..
cmake --build . --config Debug
```

**Run the system**
```powershell
# Terminal 1 — C++ simulation engine
cd EVThermalControl
.\cmake-build\Debug\EVThermalControl.exe

# Terminal 2 — Python dashboard
python scripts/dashboard.py
```

Open `http://127.0.0.1:8050` in your browser.

**Run unit tests**
```powershell
cd cmake-build\Debug
.\ev_tests.exe
```

---

## Technical Highlights

**Delta-time physics** — The simulation loop measures actual elapsed time between frames using `std::chrono`. All drain calculations and interpolations are multiplied by `dt * 60` so the simulation runs identically regardless of system load or tick timing.

**Atomic state sharing** — Target speed and temperature are stored as `std::atomic<float>` so the scenario runner thread and simulation loop can share state without mutex overhead on the hot path.

**Instant state reset** — When a new scenario starts, a `g_resetRequested` atomic flag tells the simulation loop to snap `actualBattTemp` to 25°C on the very next tick. This forces the ThermalController out of SAFE SHUTDOWN immediately, so the dashboard badge updates within 200ms of clicking a new scenario button.

**Dual socket architecture** — Port 9000 is a streaming data socket (C++ → Python, one JSON packet per tick). Port 9001 is a command socket (Python → C++, one scenario name per connection). This separation means data flow is never blocked by command processing.

**SIL fault injection** — The DiagnosticsManager's `injectSensorFault(n)` method simulates sensor dropout for `n` ticks. The simulation loop treats injected faults as invalid sensor data, the controller enters FAULT state, and DTCs are raised — exactly as they would be in a real ECU responding to a faulty CAN message.

---

## Interview Notes

This project was built to demonstrate practical knowledge of:

- Embedded C++ patterns — FSM design, atomic operations, thread safety, memory management without dynamic allocation on the hot path
- Automotive protocols — CAN bus frame structure, DLC encoding, message ID assignment following standard automotive conventions
- Diagnostics — DTC lifecycle (raise, resolve, clear), severity levels, SIL fault injection methodology
- Control theory — Hysteresis in threshold-based controllers, proportional actuator output, thermal lag modelling
- Software architecture — Clean separation between sensor layer, control layer, communication layer, and diagnostics layer
- Testing — Unit tests covering state transitions, boundary conditions, fault lifecycles, and sensor physics

The phrase "software-defined vehicle" in the subtitle refers specifically to the fact that all control decisions that would normally run on physical ECU hardware are here implemented in software, making the system testable, observable, and demoable without any hardware.

---

## GitHub

[https://github.com/simbarashemamvura1/EVThermalControl](https://github.com/simbarashemamvura1/EVThermalControl)