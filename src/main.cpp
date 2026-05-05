#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "SensorSimulator.h"

int main() {
    // Initial conditions: 25C battery, 20C ambient, 85% SoC
    SensorSimulator sensor(25.0f, 20.0f, 0.85f);

    const float deltaTime    = 0.5f;   // 0.5 second per tick
    const float vehicleSpeed = 180.0f; // kph — try changing this

    std::cout << "=== EV Thermal Control System — Sensor Simulator ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(8)  << "Time(s)"
              << std::setw(12) << "BattTemp"
              << std::setw(10) << "SoC%"
              << std::setw(12) << "Voltage"
              << std::setw(12) << "Current"
              << std::setw(12) << "Coolant"
              << std::endl;
    std::cout << std::string(64, '-') << std::endl;

    for (int i = 0; i < 30; i++) {
        SensorReading r = sensor.update(vehicleSpeed, deltaTime);

        std::cout << std::setw(8)  << (i * deltaTime)
                  << std::setw(12) << r.battery.temperature
                  << std::setw(10) << (r.battery.stateOfCharge * 100.0f)
                  << std::setw(12) << r.battery.voltage
                  << std::setw(12) << r.battery.current
                  << std::setw(12) << r.thermal.coolantTemperature
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::string(64, '-') << std::endl;
    std::cout << "Simulation complete." << std::endl;
    return 0;
}