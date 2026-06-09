#define NOMINMAX

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
//  GLOBAL POINTERS
// ============================================================
static DiagnosticsManager* g_diag       = nullptr;
static ThermalController*  g_controller = nullptr;

// ============================================================
//  RESET FLAG — tells simulation loop to snap temp instantly
// ============================================================
std::atomic<bool>  g_resetRequested(false);
std::atomic<float> g_resetTemp(25.0f);

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

std::string jsonStr(const std::string& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); i++) {
        char ch = s[i];
        if (ch == '"')       out += "\\\"";
        else if (ch == '\\') out += "\\\\";
        else                 out += ch;
    }
    return out;
}

std::string buildJson(
    const SensorReading&      reading,
    const ActuatorCommand&    cmd,
    const DiagnosticsManager& diag,
    float vehicleSpeed,
    float realSoC,
    int   tick)
{
    std::ostringstream j;
    j << std::fixed << std::setprecision(2);

    std::string dtcArray = "[";
    const auto& active = diag.getActiveFaults();
    for (size_t i = 0; i < active.size(); i++) {
        std::ostringstream code;
        code << "DTC 0x" << std::hex << std::uppercase
             << active[i].code << std::dec
             << " - " << active[i].description;
        dtcArray += "\"" + jsonStr(code.str()) + "\"";
        if (i + 1 < active.size()) dtcArray += ",";
    }
    dtcArray += "]";

    std::string dtc = "";
    if (!active.empty()) {
        std::ostringstream code;
        code << "DTC 0x" << std::hex << std::uppercase
             << active.back().code << std::dec
             << " - " << active.back().description;
        dtc = jsonStr(code.str());
    }

    j << "{"
      << "\"tick\":"       << tick                                     << ","
      << "\"spd\":"        << vehicleSpeed                             << ","
      << "\"bT\":"         << reading.battery.temperature              << ","
      << "\"soc\":"        << (reading.battery.stateOfCharge * 100.0f) << ","
      << "\"realSoC\":"    << realSoC                                  << ","
      << "\"voltage\":"    << reading.battery.voltage                  << ","
      << "\"current\":"    << reading.battery.current                  << ","
      << "\"coolant\":"    << reading.thermal.coolantTemperature       << ","
      << "\"ambient\":"    << reading.thermal.ambientTemperature       << ","
      << "\"fan\":"        << (cmd.fanSpeedPercent * 100.0f)           << ","
      << "\"pump\":"       << (cmd.pumpActive   ? "true" : "false")    << ","
      << "\"heater\":"     << (cmd.heaterActive ? "true" : "false")    << ","
      << "\"sys\":\""      << stateToString(cmd.state) << "\""         << ","
      << "\"fault\":"      << (diag.isFaultInjected() ? "true":"false")<< ","
      << "\"faultCount\":" << (int)diag.getFaultHistory().size()       << ","
      << "\"dtc\":\""      << dtc << "\""                              << ","
      << "\"dtcs\":"       << dtcArray
      << "}";

    return j.str();
}

// ============================================================
//  SCENARIO DEFINITIONS
// ============================================================
struct ScenarioStep {
    float       duration;
    float       speed;
    float       battTemp;
    std::string label;
};
typedef std::vector<ScenarioStep> Scenario;
typedef std::map<std::string, Scenario> ScenarioMap;
ScenarioMap SCENARIOS;

void initScenarios() {
    Scenario cold;
    cold.push_back({5.0f,  0.0f,  -10.0f, "Cold Start - Battery at -10 C"});
    cold.push_back({6.0f,  40.0f,  -4.0f, "Cold Start - Heater active, warming"});
    cold.push_back({6.0f,  70.0f,   8.0f, "Cold Start - Approaching normal range"});
    cold.push_back({5.0f,  90.0f,  19.0f, "Cold Start - Normal temperature reached"});
    SCENARIOS["cold"] = cold;

    Scenario highway;
    highway.push_back({5.0f,  60.0f,  24.0f, "Highway - Cruise at 60 kph"});
    highway.push_back({6.0f, 120.0f,  36.0f, "Highway - Accelerating to 120 kph"});
    highway.push_back({6.0f, 180.0f,  47.0f, "Highway - Cooling system activated"});
    highway.push_back({4.0f, 180.0f,  64.0f, "Highway - Overtemp warning"});
    highway.push_back({4.0f, 180.0f,  74.0f, "Highway - Fault state, full cooling"});
    highway.push_back({4.0f,   0.0f,  82.0f, "Highway - Safe shutdown triggered"});
    SCENARIOS["highway"] = highway;

    Scenario fault;
    fault.push_back({5.0f, 100.0f,  30.0f, "Fault Injection - Normal driving"});
    fault.push_back({5.0f, 100.0f, -999.f, "Fault Injection - Sensor dropout active"});
    fault.push_back({5.0f, 100.0f,  33.0f, "Fault Injection - System recovering"});
    fault.push_back({4.0f,  80.0f,  27.0f, "Fault Injection - Recovery confirmed"});
    SCENARIOS["fault"] = fault;

    Scenario full;
    full.push_back({5.0f,   0.0f,  -8.0f, "Full Run [1/3] - Cold Start: -8 C"});
    full.push_back({6.0f,  50.0f,   2.0f, "Full Run [1/3] - Heater active"});
    full.push_back({5.0f,  90.0f,  18.0f, "Full Run [1/3] - Normal reached"});
    full.push_back({5.0f, 130.0f,  30.0f, "Full Run [2/3] - Accelerating to highway"});
    full.push_back({5.0f, 180.0f,  46.0f, "Full Run [2/3] - Cooling activated"});
    full.push_back({4.0f, 180.0f,  62.0f, "Full Run [2/3] - Overtemp warning"});
    full.push_back({4.0f,  50.0f,  38.0f, "Full Run [2/3] - Decelerating"});
    full.push_back({4.0f, 100.0f,  28.0f, "Full Run [3/3] - Pre-fault baseline"});
    full.push_back({5.0f, 100.0f, -999.f, "Full Run [3/3] - Sensor dropout"});
    full.push_back({5.0f,  80.0f,  27.0f, "Full Run [3/3] - Full recovery"});
    SCENARIOS["full"] = full;
}

// ============================================================
//  GLOBAL STATE
// ============================================================
std::atomic<float> g_targetSpeed(0.0f);
std::atomic<float> g_targetBattTemp(25.0f);
std::atomic<bool>  g_injectFault(false);
std::atomic<bool>  g_scenarioRunning(false);
std::atomic<bool>  g_stopScenario(false);

std::string g_scenarioLabel = "C++ engine active - select a scenario";
std::mutex  g_labelMutex;

void setLabel(const std::string& label) {
    std::lock_guard<std::mutex> lk(g_labelMutex);
    g_scenarioLabel = label;
}
std::string getLabel() {
    std::lock_guard<std::mutex> lk(g_labelMutex);
    return g_scenarioLabel;
}

// ============================================================
//  SCENARIO RUNNER
// ============================================================
void runScenario(const std::string& name) {
    if (!g_diag || !g_controller) return;
    if (SCENARIOS.find(name) == SCENARIOS.end()) return;

    // Stop any running scenario
    g_stopScenario = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    g_stopScenario = false;
    g_scenarioRunning = true;

    // ---- INSTANT FULL RESET ----
    // Set targets to normal
    g_targetSpeed.store(0.0f);
    g_targetBattTemp.store(25.0f);
    g_injectFault = false;
    g_diag->clearFaults();

    // Signal simulation loop to snap actualBattTemp to 25 immediately
    // This prevents the controller from staying in SAFE SHUTDOWN
    g_resetTemp.store(25.0f);
    g_resetRequested.store(true);

    // Reset controller state machine to NORMAL
    g_controller->reset();

    // Wait for simulation loop to process the snap reset (2-3 ticks)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Now run the scenario steps
    const Scenario& steps = SCENARIOS[name];
    for (size_t i = 0; i < steps.size(); i++) {
        if (g_stopScenario) break;
        const ScenarioStep& step = steps[i];

        setLabel(step.label);
        g_targetSpeed.store(step.speed);

        if (step.battTemp < -900.0f) {
            g_diag->injectSensorFault(20);
            g_injectFault = true;
        } else {
            g_targetBattTemp.store(step.battTemp);
            g_injectFault = false;
        }

        int ms = (int)(step.duration * 1000);
        int elapsed = 0;
        while (elapsed < ms && !g_stopScenario) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            elapsed += 100;
        }
    }

    if (!g_stopScenario) {
        g_targetSpeed.store(0.0f);
        g_targetBattTemp.store(25.0f);
        g_injectFault = false;
        setLabel("Simulation complete - select next scenario");
    }
    g_scenarioRunning = false;
}

void runFullScenario()    { runScenario("full");    }
void runColdScenario()    { runScenario("cold");    }
void runHighwayScenario() { runScenario("highway"); }
void runFaultScenario()   { runScenario("fault");   }

// ============================================================
//  COMMAND SERVER — port 9001
// ============================================================
void commandServerLoop() {
    int serverFd = (int)socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&opt, sizeof(opt));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(9001);
    bind(serverFd, (sockaddr*)&addr, sizeof(addr));
    listen(serverFd, 5);
    std::cout << "[CMD] Command server on port 9001\n";

    while (true) {
        sockaddr_in client;
        memset(&client, 0, sizeof(client));
        socklen_t clen = sizeof(client);
        int fd = (int)accept(serverFd, (sockaddr*)&client, &clen);
        if (fd < 0) continue;

        char buf[64];
        memset(buf, 0, sizeof(buf));
        int n = recv(fd, buf, sizeof(buf) - 1, 0);

        if (n > 0) {
            std::string cmd(buf, n);
            while (!cmd.empty() &&
                   (cmd.back() == '\n' || cmd.back() == '\r' ||
                    cmd.back() == ' '))
                cmd.pop_back();

            std::cout << "[CMD] Received scenario: " << cmd << "\n";

            if (cmd == "cold") {
                std::thread t(runColdScenario); t.detach();
            } else if (cmd == "highway") {
                std::thread t(runHighwayScenario); t.detach();
            } else if (cmd == "fault") {
                std::thread t(runFaultScenario); t.detach();
            } else if (cmd == "full") {
                std::thread t(runFullScenario); t.detach();
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
//  AUTO START
// ============================================================
void autoStartFull() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    runScenario("full");
}

// ============================================================
//  MAIN
// ============================================================
int main() {
    initScenarios();

    std::cout << "=== EV Thermal Control System ===\n";
    std::cout << "Starting simulation + socket server...\n\n";

    SensorSimulator    sensor(25.0f, 20.0f, 1.0f);
    ThermalController  controller;
    CANBus             bus("build/can_log.csv");
    DiagnosticsManager diag("build/dtc_log.csv");
    SocketServer       dataServer(9000);

    g_diag       = &diag;
    g_controller = &controller;

    dataServer.start();

    std::thread cmdThread(commandServerLoop);
    cmdThread.detach();

    std::thread autoThread(autoStartFull);
    autoThread.detach();

    const float deltaTime = 0.2f;
    float actualSpeed     = 0.0f;
    float actualBattTemp  = 25.0f;
    float actualSoC       = 100.0f;
    bool  batteryDead     = false;
    int   tick            = 0;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << std::left
              << std::setw(6)  << "Tick"
              << std::setw(10) << "Speed"
              << std::setw(12) << "BattTemp"
              << std::setw(8)  << "Fan%"
              << std::setw(8)  << "Pump"
              << std::setw(10) << "SoC%"
              << std::setw(16) << "State"
              << std::setw(8)  << "Faults"
              << "\n" << std::string(78, '-') << "\n";

    while (true) {

        // ---- INSTANT RESET — snap actualBattTemp when requested ----
        // This is what makes the badge update immediately on new scenario
        if (g_resetRequested.load()) {
            actualBattTemp = g_resetTemp.load();
            actualSpeed    = 0.0f;
            g_resetRequested.store(false);
            std::cout << "[RESET] System reset to normal temp: "
                      << actualBattTemp << "C\n";
        }

        // Dead battery check
        if (actualSoC <= 0.0f && !batteryDead) {
            batteryDead    = true;
            g_targetSpeed  = 0.0f;
            g_stopScenario = true;
            setLabel("Battery depleted - charge required to continue");
            std::cout << "\n*** BATTERY DEPLETED - Vehicle stopped. ***\n\n";
        }

        if (batteryDead) {
            std::ostringstream ds;
            ds << std::fixed << std::setprecision(1);
            ds << "{\"tick\":"    << tick
               << ",\"spd\":0"
               << ",\"bT\":"     << actualBattTemp
               << ",\"soc\":0,\"realSoC\":0,\"fan\":0"
               << ",\"pump\":false,\"heater\":false"
               << ",\"sys\":\"NORMAL\",\"fault\":false"
               << ",\"faultCount\":0,\"dtc\":\"\",\"dtcs\":[]"
               << ",\"scenario\":\"Battery depleted - charge required\""
               << ",\"coolant\":22,\"ambient\":20"
               << ",\"voltage\":300,\"current\":0}";
            dataServer.broadcast(ds.str());
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(deltaTime * 1000)));
            tick++;
            continue;
        }

        // Smooth speed
        float tgtSpd  = g_targetSpeed.load();
        float spdDiff = tgtSpd - actualSpeed;
        float spdRate = spdDiff > 0.0f ? 0.08f : 0.15f;
        actualSpeed  += spdDiff * spdRate;
        if (actualSpeed < 0.5f && actualSpeed > -0.5f && tgtSpd == 0.0f)
            actualSpeed = 0.0f;

        // Smooth battery temp — lerp toward target
        float tgtBT    = g_targetBattTemp.load();
        float btDiff   = tgtBT - actualBattTemp;
        actualBattTemp += btDiff * 0.05f;

        // Realistic SoC drain
        float baseDrain   = 0.0008f;
        float driveDrain  = actualSpeed * 0.000055f;
        float heatDrain   = (actualBattTemp > 40.0f)
                            ? (actualBattTemp - 40.0f) * 0.00015f : 0.0f;
        float coldDrain   = (actualBattTemp < 10.0f)
                            ? (10.0f - actualBattTemp) * 0.0001f  : 0.0f;
        bool  heaterOn    = (actualBattTemp < 10.0f);
        float heaterDrain = heaterOn ? 0.003f : 0.0f;
        float totalDrain  = (baseDrain + driveDrain + heatDrain
                             + coldDrain + heaterDrain) * deltaTime * 60.0f;
        actualSoC = actualSoC - totalDrain;
        if (actualSoC < 0.0f) actualSoC = 0.0f;

        // Sensor reading
        SensorReading reading = sensor.update(actualSpeed, deltaTime);
        reading.battery.temperature   = actualBattTemp +
            (reading.battery.temperature - 25.0f) * 0.1f;
        reading.battery.stateOfCharge = actualSoC / 100.0f;

        // CAN bus
        bus.publish(CANBus::encodeBatteryStatus(reading));
        bus.publish(CANBus::encodeThermalStatus(reading));
        bus.processPendingMessages();

        // Controller
        ActuatorCommand cmd = controller.update(reading);
        reading.thermal.fanSpeedPercent = cmd.fanSpeedPercent;
        reading.thermal.pumpActive      = cmd.pumpActive;
        reading.thermal.heaterActive    = cmd.heaterActive;

        // Actuator CAN
        bus.publish(CANBus::encodeActuatorCommand(cmd, reading.timestampMs));
        bus.processPendingMessages();

        // Diagnostics
        diag.update(reading, controller.getCurrentState());

        // Broadcast JSON with exact speed value
        std::string label    = getLabel();
        std::string base     = buildJson(reading, cmd, diag,
                                         actualSpeed, actualSoC, tick);
        std::string insert   = ",\"scenario\":\"" + jsonStr(label) + "\"";
        std::string fullJson = base.substr(0, base.size() - 1) + insert + "}";
        dataServer.broadcast(fullJson);

        // Console every 5 ticks
        if (tick % 5 == 0) {
            std::cout << std::setw(6)  << tick
                      << std::setw(10) << actualSpeed
                      << std::setw(12) << reading.battery.temperature
                      << std::setw(8)  << (cmd.fanSpeedPercent * 100.0f)
                      << std::setw(8)  << (cmd.pumpActive ? "ON" : "OFF")
                      << std::setw(10) << actualSoC
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