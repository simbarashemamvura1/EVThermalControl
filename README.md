# EV Thermal Battery Control System

A software-defined EV battery thermal management system built in C++ with a live Python dashboard. Designed to demonstrate embedded control system architecture, sensor simulation, CAN bus communication, diagnostics, and real-time visualization — modeled after production automotive software patterns.

---

## Overview

This project simulates the thermal management system of an electric vehicle battery pack. It was built as a portfolio project targeting embedded controls engineering roles in the automotive industry, specifically demonstrating competency in areas directly relevant to battery management systems (BMS) development.

The C++ simulation engine runs continuously, processing sensor data through a five-state finite state machine controller, logging CAN bus traffic, and streaming live JSON data to a Python dashboard over a TCP socket. The dashboard renders a real-time car visualization, live graphs, a fault event log, and a voice alert system — all driven by the actual C++ control logic.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  C++ Simulation Engine                       │
│                                                              │
│  SensorSimulator → ThermalController → CANBus               │
│         ↓                ↓                ↓                  │
│  DiagnosticsManager   ActuatorCmd     CSV Logs               │
│         ↓                                                    │
│  SocketServer (port 9000) ← CommandServer (port 9001)        │
└──────────────────────────┬──────────────────────────────────┘
                           │ TCP JSON stream (200ms tick)
┌──────────────────────────▼──────────────────────────────────┐
│              Python Dashboard (Dash / Flask)                  │
│                                                              │
│  Socket Reader → Shared State → /live-state endpoint         │
│       ↓                ↓               ↓                     │
│  Graphs           Stat Cards    /scene Flask route           │
│  Fault Log        DTC Alert     Canvas2D Car Simulation      │
│                                 + Web Audio Voice Alerts      │
└─────────────────────────────────────────────────────────────┘

Data flow:   C++ → port 9000 → Python (every 200ms)
Commands:    Python → port 9001 → C++ (on button click)
Scene:       Python /scene → iframe → Canvas2D (polls /live-state every 300ms)
```

---

## C++ Modules

### SensorSimulator
Generates physics-based sensor readings every simulation tick. Models battery temperature, state of charge, voltage, and current with configurable Gaussian noise to mimic real sensor imprecision. Heat generation scales with vehicle speed and motor load. Thermal lag is modelled so temperature changes are gradual rather than instantaneous.

### ThermalController
A five-state finite state machine that reads sensor data and outputs actuator commands every tick:

| State | Trigger | Action |
|---|---|---|
| `NORMAL` | Default | Fan off, pump off, heater off |
| `HEATING` | Battery temp < 10°C | Heater on |
| `COOLING` | Battery temp > 45°C | Fan proportional to temp, pump on |
| `FAULT` | Temp > 70°C or sensor dropout | Fan 100%, all alerts active |
| `SAFE_SHUTDOWN` | Temp > 80°C | All actuators off, shutdown sequence |

Hysteresis thresholds prevent state chatter — cooling does not deactivate until temperature falls to 40°C. A `reset()` method was added to allow the scenario runner to snap the controller back to NORMAL instantly when a new scenario starts, without waiting for temperature to drift down naturally.

### CANBus
Simulates CAN bus communication with real frame structure — message IDs, DLC (data length code), and 8-byte payloads. Three frame types are encoded and transmitted each tick:

- `0x100` — Battery status (temperature, SoC, voltage, current)
- `0x101` — Thermal status (coolant temp, fan speed, pump/heater state)
- `0x200` — Actuator commands

All frames are logged to `build/can_log.csv` for post-run analysis.

### DiagnosticsManager
Manages Diagnostic Trouble Codes (DTCs) with full lifecycle management — raise, resolve, and clear. Maintains an active fault list and a complete fault history. Supports software-in-the-loop (SIL) fault injection for testing fault response paths. All DTC events are logged to `build/dtc_log.csv`.

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
TCP data server on port 9000. Broadcasts a JSON packet every simulation tick (200ms) containing all sensor readings, actuator states, system state, active DTCs, real SoC, vehicle speed, and current scenario label.

### Command Server
TCP command receiver on port 9001. Accepts plain-text scenario commands (`cold`, `highway`, `fault`, `full`) from the Python dashboard. On receipt, immediately stops any running scenario, snaps simulation state to safe defaults using the `g_resetRequested` atomic flag, resets the ThermalController to NORMAL, and starts the new scenario in a detached thread.

---

## Realistic Battery Physics

The simulation models a physically accurate state of charge drain with five separate components:

```cpp
float baseDrain   = 0.0008f;
// Parasitic draw — BMS, sensors, 12V systems always active

float driveDrain  = actualSpeed * 0.000055f;
// Speed-proportional motor load — main drain during driving

float heatDrain   = (bT > 40.0f) ? (bT - 40.0f) * 0.00015f : 0.0f;
// Internal resistance rises with temperature above 40C
// Hot battery wastes more energy as heat — faster discharge

float coldDrain   = (bT < 10.0f) ? (10.0f - bT) * 0.0001f : 0.0f;
// Lithium-ion loses usable capacity when cold
// Cold battery has effectively smaller usable capacity

float heaterDrain = heaterOn ? 0.003f : 0.0f;
// Battery/cabin heater is the single largest load during cold start

float totalDrain  = (baseDrain + driveDrain + heatDrain
                     + coldDrain + heaterDrain) * deltaTime * 60.0f;
```

Practical consequences:
- Parked with all systems idle — drain is negligible
- Cold start at -10°C with heater — drains faster than 60 kph highway cruise
- Hot battery at 180 kph — two components active simultaneously, fastest discharge
- Recovering from fault — drain drops immediately as speed decreases

---

## Dead Battery Protection

At SoC ≤ 0.5% the following sequence triggers automatically:

1. C++ sets `g_targetSpeed = 0` and `g_stopScenario = true`
2. Terminal prints a BATTERY DEPLETED warning
3. JSON broadcasts `battery_dead: true`
4. Dashboard shows a red alert blocking all scenario buttons
5. Car scene shows DEPLETED label, red battery fill, and overlay message
6. Voice announces: *"Battery fully depleted. All drive systems offline."*
7. Simulation loop continues broadcasting dead state but does not simulate further

---

## Unit Tests

22 unit tests across three suites using Google Test (fetched automatically via CMake FetchContent):

**Sensor tests (6)** — Temperature rises with speed, SoC drains over time, SoC never goes negative, readings stay within physical bounds, higher speed produces more heat, voltage correlates with SoC.

**Controller tests (8)** — Normal conditions stay NORMAL, cooling activates above 45°C, heating activates below 10°C, fault state above 70°C, safe shutdown above 80°C, shutdown state is sticky, hysteresis prevents rapid switching, fan speed is proportional to temperature.

**Diagnostics tests (8)** — No faults in normal conditions, overtemp warning raises DTC, critical overtemp raises DTC, fault injection raises DTC, DTC resolves after duration, DTC resolves when condition clears, low SoC raises DTC, fault history grows correctly.

---

## Python Dashboard

Built with Dash and Plotly. Operates in two modes:

**C++ Engine Live** — Socket reader thread connects to port 9000 and streams real physics data from the C++ engine. Scenario buttons send commands to port 9001 processed by the C++ command server. All state transitions, DTCs, and sensor readings come from actual C++ code.

**Python Fallback** — If the C++ engine is not running, a Python physics engine takes over automatically. Implements the same state machine logic, realistic drain model, and DTC generation so the dashboard remains fully functional for standalone demos.

The connection status is displayed in the header as a live badge — green **C++ Engine Live** or grey **Python Fallback**.

### Dashboard panels
- Real-time battery temperature graph with COOLING (45°C) and HEATING (10°C) threshold lines
- Fan speed graph showing proportional spin-up and smooth deceleration
- State of charge graph with Low SoC (15%) and Critical (5%) threshold lines — line turns red below 15%
- Fault event log showing all DTCs with tick timestamps
- Six sensor status indicators (Temp, SoC, Voltage, CAN Bus, Pump, Fan) — turn red during fault injection
- Active Diagnostic Codes alert box showing all currently active DTCs
- Battery Depleted alert that blocks scenario execution when SoC reaches 0%

### Flask no-cache scene route
The car scene is served through a Flask route at `/scene` rather than as a static asset. This was necessary because Dash serves files from `/assets/` with aggressive browser caching headers that prevented updated JavaScript from loading. The Flask route sets `Cache-Control: no-store` so every page load fetches the latest version directly from disk.

```python
@app.server.route('/scene')
def serve_scene():
    with open('assets/ev_scene.html', 'r') as f:
        html_content = f.read()
    resp = Response(html_content, mimetype='text/html')
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
    return resp
```

---

## Car Scene

A Canvas2D car simulation served in an iframe. Polls `/live-state` every 300ms and smoothly interpolates toward server values so animation remains fluid at the 250ms dashboard update interval.

**Visual features:**
- Stationary sedan with world-scrolling parallax — road dashes, near trees, far trees, and clouds scroll at different speeds proportional to speed
- Battery pack cutaway showing SoC fill bar (color shifts blue → teal → amber → red), temp and SoC% labels, cell grid overlay
- Motor with heat-tinted glow and rotor animation proportional to speed
- Animated cooling pipe flow between battery and motor
- Four sensor dots (T, V, S, C) pulse green in normal operation, flash red during fault injection
- Speed lines appear above 50 kph and intensify with speed
- Engine vibration above 100 kph
- Sky tints red during FAULT and SAFE SHUTDOWN
- Braking taillight brightens when decelerating
- Dead battery overlay, red fill, DEPLETED label when SoC reaches 0%
- Low SoC warning label bottom-right below 15%

### Audio and Voice Alert System

Each event fires a beep first, then a spoken voice alert 350ms later — matching the automotive chime-then-announcement pattern used in real vehicles.

**Voice system design:**
- Female synthetic voice using `SpeechSynthesis` API (Microsoft Zira → Google UK English Female → Samantha → Victoria in order of preference)
- Rate: 0.88 (deliberate and clear), Pitch: 1.15 (crisp)
- FIFO speech queue — messages never interrupt each other mid-sentence
- `pushFront()` — inserts critical alert next in queue without cancelling current speech
- `pushBack()` — appends step announcements to end of queue
- 220ms gap between utterances for natural pacing

**Voice alerts:**

| Trigger | Beep | Voice |
|---|---|---|
| Scenario button click | Soft double chime | Scenario-specific introduction |
| Vehicle hits 100 kph | Soft chime | *"Vehicle accelerating to highway speed..."* |
| HEATING activated | Warm low tones | *"Battery below normal temperature threshold. Switching heater on..."* |
| Heater turns off | Positive chime | *"Switching off heater. Battery system stable."* |
| COOLING activated | Rising twin tones | *"Battery temperature increasing to critical level. Cooling fans now active..."* |
| FAULT (sensor dropout) | Triple sawtooth | *"Warning. Sensor dropout detected. BMS has lost reliable data..."* |
| Recovery from FAULT | Warm tones | *"Sensor data restored. System recovered. Resuming standard monitoring."* |
| SAFE SHUTDOWN | Long alarm | *"Critical alert. Temperature exceeded safe limits. Initiating controlled shutdown..."* |
| SoC hits 15% | Triple beep | *"Low battery warning. State of charge below 15 percent..."* |
| Battery dead | Heavy descending | *"Battery fully depleted. All drive systems offline..."* |
| Simulation complete | Rising chime | Scenario-specific completion message |

---

## Simulation Scenarios

Each scenario can be started at any time. Clicking a new button immediately resets the previous scenario — temperature snaps to 25°C, controller resets to NORMAL, and the new scenario begins within 400ms.

**Cold Start** — Battery begins at -10°C. Heater activates and battery temperature rises gradually as vehicle accelerates. Demonstrates HEATING state, heater drain effect on SoC, and cold-weather efficiency loss.

**Highway Heat** — Vehicle accelerates to 180 kph. Battery temperature climbs past the cooling threshold (45°C), then the fault threshold (70°C), triggering FAULT and finally SAFE SHUTDOWN at 80°C. Demonstrates the full thermal protection chain.

**Fault Injection** — Normal driving at 100 kph, then a sensor dropout is injected for 20 ticks. Sensor dots turn red, FAULT state activates, DTCs fire. After the injection window closes, the system recovers automatically and announces recovery.

**Full Auto Run** — Runs all three scenarios back to back: cold start warm-up, highway heat soak, then fault injection and recovery. Designed for demos and interview recording.

---

## Project Structure

```
EVThermalControl/
├── CMakeLists.txt
├── README.md
├── src/
│   ├── types.h                    Core structs: BatteryState, ThermalState,
│   │                              SensorReading, CANFrame, DiagnosticCode
│   ├── SensorSimulator.h/.cpp     Physics-based sensor generation with Gaussian noise
│   ├── ThermalController.h/.cpp   Five-state FSM with hysteresis + reset()
│   ├── CANBus.h/.cpp              CAN frame encoding and CSV logging
│   ├── DiagnosticsManager.h/.cpp  DTC lifecycle management and fault injection
│   ├── SocketServer.h/.cpp        TCP data server on port 9000
│   └── main.cpp                   Simulation loop, scenario runner, command server
├── tests/
│   ├── test_sensor.cpp            6 sensor simulation tests
│   ├── test_controller.cpp        8 state machine tests
│   └── test_diagnostics.cpp       8 diagnostics tests
├── scripts/
│   ├── dashboard.py               Dash dashboard with dual-socket integration
│   └── assets/
│       └── ev_scene.html          Canvas2D car simulation with voice alert system
├── cmake-build/
│   └── Debug/
│       ├── EVThermalControl.exe
│       └── ev_tests.exe
└── build/
    ├── can_log.csv                CAN bus traffic log
    └── dtc_log.csv                DTC event log
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

**Run the full system**
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

**Delta-time physics** — The simulation loop measures actual elapsed time between frames using `std::chrono`. All drain calculations and interpolations are multiplied by `dt * 60` so the simulation behaves identically regardless of system load or frame rate.

**Atomic state sharing** — Target speed and temperature are `std::atomic<float>` so the scenario runner thread and simulation loop share state without mutex overhead on the hot path. Only the scenario label string uses a mutex since `std::string` is not atomically copyable.

**Instant scenario reset** — `g_resetRequested` is an atomic flag checked at the top of every simulation tick. When set, `actualBattTemp` snaps to 25°C immediately — forcing the ThermalController out of SAFE SHUTDOWN on the very next tick. Combined with `controller.reset()`, the dashboard badge updates within 200ms of clicking any scenario button.

**Dual socket architecture** — Port 9000 streams data (C++ → Python, one JSON packet per tick). Port 9001 receives commands (Python → C++, one scenario name per connection). This separation ensures data flow is never blocked by command processing, and the two concerns are independently testable.

**SIL fault injection** — `DiagnosticsManager::injectSensorFault(n)` simulates sensor dropout for `n` ticks. The simulation loop marks sensor data as invalid, the controller enters FAULT state, and DTCs are raised — exactly as they would be in a real ECU responding to a faulty CAN message.

**No-cache scene serving** — The car scene is served through Flask with `Cache-Control: no-store` rather than as a Dash static asset. This was required because Dash's built-in static file server uses aggressive caching that prevented JavaScript changes from loading in the browser, even after hard refresh.

---

## Known Issues and Engineering Decisions

This section documents real problems encountered during development and how they were resolved. These are worth understanding if you are extending the project.

### MSVC Lambda Capture Error (C2589)

**Problem:** MSVC rejected lambda captures in thread creation inside `commandServer()` and `autoStartFull()`. The error `C2589: '(': illegal token on right side of '::'` appeared alongside `C2059: syntax error: ')'`. The same file had conflicts between `std::max` and the Windows `max` macro.

**Fix:**
- Added `#define NOMINMAX` as the very first line before all includes — prevents Windows headers from defining `min`/`max` as macros
- Replaced all lambdas with named free functions (`runColdScenario`, `runHighwayScenario`, etc.)
- Accessed `DiagnosticsManager` and `ThermalController` via global pointers (`g_diag`, `g_controller`) instead of reference captures
- Replaced all `std::max()` / `std::min()` calls with explicit conditional expressions

### SAFE SHUTDOWN Badge Sticky After Scenario Switch

**Problem:** After Highway Heat ended in SAFE SHUTDOWN, clicking a new scenario showed the badge updating slowly or not at all. The root cause was that `actualBattTemp` was still 82°C — so the controller immediately re-entered SAFE SHUTDOWN on the very next simulation tick, even after `controller.reset()` was called in the scenario runner thread.

**Fix:** Added `g_resetRequested` atomic flag. The scenario runner sets it when starting a new scenario. The simulation loop checks it at the top of every tick and if set, snaps `actualBattTemp` to 25°C immediately before running the controller. The controller then reads a normal temperature and stays in NORMAL. Badge updates within 200ms.

### Browser Iframe Caching

**Problem:** Changes to `ev_scene.html` (voice lines, physics tweaks, new features) were not reflecting in the dashboard even after restarting Python, hard-refreshing the browser with `Ctrl+Shift+R`, or adding `?v=N` query parameters to the iframe src. The browser was serving a cached version regardless.

**Root cause:** Dash serves files from `/assets/` with `Cache-Control: public, max-age=31536000` by default. Browsers respect this aggressively across sessions.

**Fix:** Replaced the static iframe src with a Flask route at `/scene` that reads the file from disk on every request and returns it with `Cache-Control: no-store`. Changes to `ev_scene.html` now appear on the next page load without any cache clearing.

### Speed Mismatch Between Scene and Dashboard

**Problem:** The speed shown on the car scene (bottom-right kph label) differed from the dashboard stat card by 10-30 kph, especially during acceleration and braking. The scene was displaying `ACT.spd` (smoothly interpolated local value) while the dashboard showed the exact C++ value.

**Fix:** Changed the scene speed readout to display `ACT._tSpd` (the exact server value received from `/live-state`) while keeping `ACT.spd` for wheel spin and road scroll animations. The display is now always exact while the physics feel remains smooth.

---

## Interview Notes

This project demonstrates practical competency in:

- **Embedded C++ patterns** — FSM design, atomic operations for lock-free inter-thread communication, thread safety with minimal mutex scope, RAII resource management
- **Automotive protocols** — CAN bus frame structure, DLC encoding, message ID assignment following standard automotive conventions (0x100-range for status, 0x200-range for commands)
- **Diagnostics** — DTC lifecycle (raise, resolve, clear), severity classification, SIL fault injection methodology matching ISO 26262 testing philosophy
- **Control theory** — Hysteresis in threshold-based controllers prevents state chatter, proportional actuator output, thermal lag modelling for realistic sensor response
- **Software architecture** — Clean separation between sensor layer, control layer, communication layer, and diagnostics layer — each module is independently unit-testable
- **Systems integration** — C++ engine wired to Python visualization layer via TCP sockets, with Python fallback physics for standalone operation
- **Real problem solving** — MSVC-specific build issues, browser caching, thread synchronization, atomic state management across multiple threads

The phrase *software-defined vehicle* refers to the fact that all control decisions that would normally run on physical ECU hardware are implemented in software — making the system testable, observable, and demonstrable without any hardware dependency.

---

## GitHub

[https://github.com/simbarashemamvura1/EVThermalControl](https://github.com/simbarashemamvura1/EVThermalControl)