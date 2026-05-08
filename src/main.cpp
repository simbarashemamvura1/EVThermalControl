#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "SensorSimulator.h"
#include "ThermalController.h"
#include "CANBus.h"
#include "DiagnosticsManager.h"

const char* stateToString(ThermalControlState s) {
    switch(s) {
        case ThermalControlState::NORMAL:        return "NORMAL";
        case ThermalControlState::COOLING:       return "COOLING";
        case ThermalControlState::HEATING:       return "HEATING";
        case ThermalControlState::FAULT:         return "FAULT";
        case ThermalControlState::SAFE_SHUTDOWN: return "SAFE_SHUTDOWN";
        default:                                 return "UNKNOWN";
    }
}

int main() {
    SensorSimulator    sensor(25.0f, 20.0f, 0.85f);
    ThermalController  controller;
    CANBus             bus("build/can_log.csv");
    DiagnosticsManager diag("build/dtc_log.csv");

    const float deltaTime    = 0.5f;
    const float vehicleSpeed = 180.0f;

    std::cout << "=== EV Thermal Control System — Full Pipeline ===\n\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << std::left
              << std::setw(8)  << "Time(s)"
              << std::setw(12) << "BattTemp"
              << std::setw(8)  << "Fan%"
              << std::setw(8)  << "Pump"
              << std::setw(16) << "State"
              << std::setw(8)  << "Faults"
              << "\n";
    std::cout << std::string(60, '-') << "\n";

    for (int i = 0; i < 35; i++) {
        uint32_t timestampMs = i * static_cast<uint32_t>(deltaTime * 1000);

        // --- FAULT INJECTION at tick 10 ---
        // Simulates a sensor dropout mid-drive for 4 ticks
        if (i == 10) {
            diag.injectSensorFault(4);
        }

        // 1. Get sensor reading
        SensorReading reading = sensor.update(vehicleSpeed, deltaTime);
        reading.timestampMs   = timestampMs;

        // 2. Publish to CAN bus
        bus.publish(CANBus::encodeBatteryStatus(reading));
        bus.publish(CANBus::encodeThermalStatus(reading));
        bus.processPendingMessages();

        // 3. Run controller
        ActuatorCommand cmd = controller.update(reading);
        bus.publish(CANBus::encodeActuatorCommand(cmd, timestampMs));
        bus.processPendingMessages();

        // 4. Run diagnostics
        diag.update(reading, controller.getCurrentState());

        // 5. Print tick summary
        std::cout << std::setw(8)  << (i * deltaTime)
                  << std::setw(12) << reading.battery.temperature
                  << std::setw(8)  << (cmd.fanSpeedPercent * 100.0f)
                  << std::setw(8)  << (cmd.pumpActive ? "ON" : "OFF")
                  << std::setw(16) << stateToString(cmd.state)
                  << std::setw(8)  << diag.getActiveFaults().size()
                  << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::cout << std::string(60, '-') << "\n";

    // Print full diagnostic report at end
    diag.printFaultReport();

    std::cout << "\nDTC log saved to: build/dtc_log.csv\n";
    return 0;
}