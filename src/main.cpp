#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef int socklen_t;
#endif

#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include "SensorSimulator.h"
#include "ThermalController.h"
#include "CANBus.h"
#include "DiagnosticsManager.h"
#include "SocketServer.h"


// ============================================================
//  HELPERS
// ============================================================
const char* stateToString(ThermalControlState s) {
    switch(s) {
        case ThermalControlState::NORMAL:        return "NORMAL";
        case ThermalControlState::COOLING:       return "COOLING";
        case ThermalControlState::HEATING:       return "HEATING";
        case ThermalControlState::FAULT:         return "FAULT";
        case ThermalControlState::SAFE_SHUTDOWN: return "SAFE SHUTDOWN";
        default:                                 return "UNKNOWN";
    }
}

// Escape string for JSON
std::string jsonStr(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"')  out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out += c;
    }
    return out;
}

// Build JSON packet — includes ALL active DTCs as array
std::string buildJson(
    const SensorReading&      reading,
    const ActuatorCommand&    cmd,
    const DiagnosticsManager& diag,
    float vehicleSpeed,
    int   tick)
{
    std::ostringstream j;
    j << std::fixed << std::setprecision(2);

    // Build DTC array from ALL active faults
    std::string dtcArray = "[";
    const auto& active = diag.getActiveFaults();
    for (size_t i = 0; i < active.size(); i++) {
        std::ostringstream code;
        code << "DTC 0x" << std::hex << std::uppercase
             << active[i].code << std::dec
             << " — " << active[i].description;
        dtcArray += "\"" + jsonStr(code.str()) + "\"";
        if (i + 1 < active.size()) dtcArray += ",";
    }
    dtcArray += "]";

    // Primary DTC string (most recent active)
    std::string dtc = "";
    if (!active.empty()) {
        std::ostringstream code;
        code << "DTC 0x" << std::hex << std::uppercase
             << active.back().code << std::dec
             << " — " << active.back().description;
        dtc = jsonStr(code.str());
    }

    j << "{"
      << "\"tick\":"       << tick                                      << ","
      << "\"spd\":"        << vehicleSpeed                              << ","
      << "\"bT\":"         << reading.battery.temperature               << ","
      << "\"soc\":"        << (reading.battery.stateOfCharge * 100.0f)  << ","
      << "\"voltage\":"    << reading.battery.voltage                   << ","
      << "\"current\":"    << reading.battery.current                   << ","
      << "\"coolant\":"    << reading.thermal.coolantTemperature        << ","
      << "\"ambient\":"    << reading.thermal.ambientTemperature        << ","
      << "\"fan\":"        << (cmd.fanSpeedPercent * 100.0f)            << ","
      << "\"pump\":"       << (cmd.pumpActive   ? "true" : "false")     << ","
      << "\"heater\":"     << (cmd.heaterActive ? "true" : "false")     << ","
      << "\"sys\":\""      << stateToString(cmd.state) << "\""          << ","
      << "\"fault\":"      << (diag.isFaultInjected() ? "true":"false") << ","
      << "\"faultCount\":" << (int)diag.getFaultHistory().size()        << ","
      << "\"dtc\":\""      << dtc << "\""                               << ","
      << "\"dtcs\":"       << dtcArray
      << "}";

    return j.str();
}

// ============================================================
//  SCENARIO DEFINITIONS
// ============================================================
struct ScenarioStep {
    float duration;
    float speed;
    float battTemp;      // -999 = inject fault
    std::string label;
};

using Scenario = std::vector<ScenarioStep>;

std::map<std::string, Scenario> SCENARIOS = {
    {"cold", {
        {5.0f,  0.0f,  -10.0f, "Cold Start — Battery at -10 C"},
        {6.0f,  40.0f,  -4.0f, "Cold Start — Heater active, warming"},
        {6.0f,  70.0f,   8.0f, "Cold Start — Approaching normal range"},
        {5.0f,  90.0f,  19.0f, "Cold Start — Normal temperature reached"},
    }},
    {"highway", {
        {5.0f,  60.0f,  24.0f, "Highway — Cruise at 60 kph"},
        {6.0f, 120.0f,  36.0f, "Highway — Accelerating to 120 kph"},
        {6.0f, 180.0f,  47.0f, "Highway — Cooling system activated"},
        {4.0f, 180.0f,  64.0f, "Highway — Overtemp warning"},
        {4.0f, 180.0f,  74.0f, "Highway — Fault state, full cooling"},
        {4.0f,   0.0f,  82.0f, "Highway — Safe shutdown triggered"},
    }},
    {"fault", {
        {5.0f, 100.0f,  30.0f, "Fault Injection — Normal driving"},
        {5.0f, 100.0f, -999.f, "Fault Injection — Sensor dropout active"},
        {5.0f, 100.0f,  33.0f, "Fault Injection — System recovering"},
        {4.0f,  80.0f,  27.0f, "Fault Injection — Recovery confirmed"},
    }},
    {"full", {
        {5.0f,   0.0f,  -8.0f, "Full Run [1/3] — Cold Start: -8 C"},
        {6.0f,  50.0f,   2.0f, "Full Run [1/3] — Heater active"},
        {5.0f,  90.0f,  18.0f, "Full Run [1/3] — Normal reached"},
        {5.0f, 130.0f,  30.0f, "Full Run [2/3] — Accelerating to highway"},
        {5.0f, 180.0f,  46.0f, "Full Run [2/3] — Cooling activated"},
        {4.0f, 180.0f,  62.0f, "Full Run [2/3] — Overtemp warning"},
        {4.0f,  50.0f,  38.0f, "Full Run [2/3] — Decelerating"},
        {4.0f, 100.0f,  28.0f, "Full Run [3/3] — Pre-fault baseline"},
        {5.0f, 100.0f, -999.f, "Full Run [3/3] — Sensor dropout"},
        {5.0f,  80.0f,  27.0f, "Full Run [3/3] — Full recovery"},
    }},
};

// ============================================================
//  GLOBAL SIMULATION STATE (atomic for thread safety)
// ============================================================
std::atomic<float>       g_targetSpeed(0.0f);
std::atomic<float>       g_targetBattTemp(25.0f);
std::atomic<bool>        g_injectFault(false);
std::atomic<bool>        g_scenarioRunning(false);
std::atomic<bool>        g_stopScenario(false);

// Shared scenario label
std::string              g_scenarioLabel = "C++ engine active — select a scenario";
std::mutex               g_labelMutex;

void setLabel(const std::string& label) {
    std::lock_guard<std::mutex> lk(g_labelMutex);
    g_scenarioLabel = label;
}
std::string getLabel() {
    std::lock_guard<std::mutex> lk(g_labelMutex);
    return g_scenarioLabel;
}

// ============================================================
//  SCENARIO RUNNER — runs in its own thread
// ============================================================
void runScenario(const std::string& name, DiagnosticsManager& diag) {
    if (SCENARIOS.find(name) == SCENARIOS.end()) return;

    // Stop any running scenario
    g_stopScenario = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    g_stopScenario = false;
    g_scenarioRunning = true;

    // Reset fault injection
    diag.clearFaults();
    g_injectFault = false;

    const auto& steps = SCENARIOS[name];
    for (const auto& step : steps) {
        if (g_stopScenario) break;

        setLabel(step.label);
        g_targetSpeed.store(step.speed);

        if (step.battTemp < -900.0f) {
            // Fault injection step
            diag.injectSensorFault(20);
            g_injectFault = true;
        } else {
            g_targetBattTemp.store(step.battTemp);
            g_injectFault = false;
        }

        // Wait for step duration, checking stop signal
        int ms = (int)(step.duration * 1000);
        int elapsed = 0;
        while (elapsed < ms && !g_stopScenario) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            elapsed += 100;
        }
    }

    if (!g_stopScenario) {
        g_targetSpeed    = 0.0f;
        g_injectFault    = false;
        setLabel("Simulation complete — select next scenario");
    }
    g_scenarioRunning = false;
}

// ============================================================
//  COMMAND SERVER — listens for scenario commands from Python
//  Uses a simple text protocol: "cold\n", "highway\n", etc.
// ============================================================
void commandServer(DiagnosticsManager& diag) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2), &wsa);
#endif
    int serverFd = (int)socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(9001);  // command port
    bind(serverFd, (sockaddr*)&addr, sizeof(addr));
    listen(serverFd, 5);
    std::cout << "[CMD] Command server on port 9001\n";

    while (true) {
        sockaddr_in client{};
        socklen_t clen = sizeof(client);
        int fd = (int)accept(serverFd, (sockaddr*)&client, &clen);
        if (fd < 0) continue;

        char buf[64] = {};
        int n = recv(fd, buf, sizeof(buf)-1, 0);
        if (n > 0) {
            std::string cmd(buf, n);
            // Trim whitespace/newlines
            while (!cmd.empty() && (cmd.back()=='\n'||cmd.back()=='\r'||cmd.back()==' '))
                cmd.pop_back();

            std::cout << "[CMD] Received scenario: " << cmd << "\n";
            if (SCENARIOS.count(cmd)) {
                std::thread([&diag, cmd](){
                    runScenario(cmd, diag);
                }).detach();
            }
        }
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
    }
}

// ============================================================
//  MAIN
// ============================================================
int main() {
    std::cout << "=== EV Thermal Control System ===\n";
    std::cout << "Starting simulation + socket server...\n\n";

    SensorSimulator    sensor(25.0f, 20.0f, 1.0f);
    ThermalController  controller;
    CANBus             bus("build/can_log.csv");
    DiagnosticsManager diag("build/dtc_log.csv");
    SocketServer       dataServer(9000);

    dataServer.start();

    // Start command server in background
    std::thread cmdThread([&diag](){
        commandServer(diag);
    });
    cmdThread.detach();

    // Auto-start full scenario after 2 seconds
    std::thread autoStart([&diag](){
        std::this_thread::sleep_for(std::chrono::seconds(2));
        runScenario("full", diag);
    });
    autoStart.detach();

    const float deltaTime = 0.2f;
    float actualSpeed     = 0.0f;
    float actualBattTemp  = 25.0f;
    int   tick            = 0;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << std::left
              << std::setw(6)  << "Tick"
              << std::setw(10) << "Speed"
              << std::setw(12) << "BattTemp"
              << std::setw(8)  << "Fan%"
              << std::setw(8)  << "Pump"
              << std::setw(16) << "State"
              << std::setw(8)  << "Faults"
              << "\n"
              << std::string(68,'-') << "\n";

    while (true) {
        // Smooth speed
        float tgtSpd  = g_targetSpeed.load();
        float spdDiff = tgtSpd - actualSpeed;
        float spdRate = spdDiff > 0 ? 0.08f : 0.15f;
        actualSpeed  += spdDiff * spdRate;
        if (std::abs(actualSpeed) < 0.5f && tgtSpd == 0.0f) actualSpeed = 0.0f;

        // Smooth battery temp
        float tgtBT  = g_targetBattTemp.load();
        float btDiff = tgtBT - actualBattTemp;
        actualBattTemp += btDiff * 0.05f;

        // 1. Sensor reading
        SensorReading reading = sensor.update(actualSpeed, deltaTime);
        reading.battery.temperature = actualBattTemp +
            (reading.battery.temperature - 25.0f) * 0.1f;

        // 2. CAN bus
        bus.publish(CANBus::encodeBatteryStatus(reading));
        bus.publish(CANBus::encodeThermalStatus(reading));
        bus.processPendingMessages();

        // 3. Controller
        ActuatorCommand cmd = controller.update(reading);
        reading.thermal.fanSpeedPercent = cmd.fanSpeedPercent;
        reading.thermal.pumpActive      = cmd.pumpActive;
        reading.thermal.heaterActive    = cmd.heaterActive;

        // 4. Actuator command on CAN
        bus.publish(CANBus::encodeActuatorCommand(cmd, reading.timestampMs));
        bus.processPendingMessages();

        // 5. Diagnostics
        diag.update(reading, controller.getCurrentState());

        // 6. Inject scenario label into JSON
        std::string label = getLabel();

        // 7. Build JSON and broadcast
        std::ostringstream fullJson;
        std::string base = buildJson(reading, cmd, diag, actualSpeed, tick);
        // Insert scenario label
        std::string insert = ",\"scenario\":\"" + jsonStr(label) + "\"";
        // Insert before closing brace
        fullJson << base.substr(0, base.size()-1) << insert << "}";
        dataServer.broadcast(fullJson.str());

        // 8. Console every 5 ticks
        if (tick % 5 == 0) {
            std::cout << std::setw(6)  << tick
                      << std::setw(10) << actualSpeed
                      << std::setw(12) << reading.battery.temperature
                      << std::setw(8)  << (cmd.fanSpeedPercent * 100.0f)
                      << std::setw(8)  << (cmd.pumpActive ? "ON" : "OFF")
                      << std::setw(16) << stateToString(cmd.state)
                      << std::setw(8)  << diag.getFaultHistory().size()
                      << "\n";
        }

        tick++;
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(deltaTime * 1000)));
    }
    return 0;
}