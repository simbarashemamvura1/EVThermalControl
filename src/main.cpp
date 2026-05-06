#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "SensorSimulator.h"
#include "ThermalController.h"

// Helper to print state as string
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

    const float deltaTime    = 0.5f;
    const float vehicleSpeed = 180.0f; // High speed to trigger cooling

    std::cout << "=== EV Thermal Control System ===\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << std::left
              << std::setw(8)  << "Time(s)"
              << std::setw(12) << "BattTemp"
              << std::setw(8)  << "SoC%"
              << std::setw(10) << "Fan%"
              << std::setw(8)  << "Pump"
              << std::setw(8)  << "Heater"
              << std::setw(16) << "State"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    for (int i = 0; i < 40; i++) {
        SensorReading  reading = sensor.update(vehicleSpeed, deltaTime);
        ActuatorCommand cmd    = controller.update(reading);

        // Update thermal state from controller output
        reading.thermal.fanSpeedPercent = cmd.fanSpeedPercent;
        reading.thermal.pumpActive      = cmd.pumpActive;
        reading.thermal.heaterActive    = cmd.heaterActive;

        std::cout << std::setw(8)  << (i * deltaTime)
                  << std::setw(12) << reading.battery.temperature
                  << std::setw(8)  << (reading.battery.stateOfCharge * 100.0f)
                  << std::setw(10) << (cmd.fanSpeedPercent * 100.0f)
                  << std::setw(8)  << (cmd.pumpActive  ? "ON" : "OFF")
                  << std::setw(8)  << (cmd.heaterActive ? "ON" : "OFF")
                  << std::setw(16) << stateToString(cmd.state)
                  << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::cout << std::string(70, '-') << "\n";
    std::cout << "Simulation complete.\n";
    return 0;
}