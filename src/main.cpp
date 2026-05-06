#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "SensorSimulator.h"
#include "ThermalController.h"
#include "CANBus.h"

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
    SensorSimulator   sensor(25.0f, 20.0f, 0.85f);
    ThermalController controller;
    CANBus            bus("build/can_log.csv");

    // Subscribe to battery status frames
    bus.subscribe(CANID::BATTERY_STATUS, [](const CANFrame& frame) {
        float temp = CANBus::decodeFloat(frame, 0);
        // Controller would read this in a real multi-threaded system
        (void)temp; // suppress unused warning
    });

    // Subscribe to actuator commands
    bus.subscribe(CANID::ACTUATOR_COMMAND, [](const CANFrame& frame) {
        // Actuator module would read this and drive hardware
        (void)frame;
    });

    const float deltaTime    = 0.5f;
    const float vehicleSpeed = 180.0f;

    std::cout << "=== EV Thermal Control System + CAN Bus ===\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << std::left
              << std::setw(8)  << "Time(s)"
              << std::setw(12) << "BattTemp"
              << std::setw(8)  << "Fan%"
              << std::setw(8)  << "Pump"
              << std::setw(16) << "State"
              << std::setw(12) << "CAN Frames"
              << "\n";
    std::cout << std::string(64, '-') << "\n";

    int totalFrames = 0;

    for (int i = 0; i < 30; i++) {
        uint32_t timestampMs = i * static_cast<uint32_t>(deltaTime * 1000);

        // 1. Get sensor reading
        SensorReading reading = sensor.update(vehicleSpeed, deltaTime);
        reading.timestampMs   = timestampMs;

        // 2. Publish sensor data as CAN frames
        bus.publish(CANBus::encodeBatteryStatus(reading));
        bus.publish(CANBus::encodeThermalStatus(reading));
        totalFrames += 2;

        // 3. Process bus — deliver frames to subscribers
        bus.processPendingMessages();

        // 4. Controller reads sensor and decides
        ActuatorCommand cmd = controller.update(reading);

        // 5. Publish actuator command as CAN frame
        bus.publish(CANBus::encodeActuatorCommand(cmd, timestampMs));
        totalFrames++;

        // 6. Process again — deliver actuator command
        bus.processPendingMessages();

        std::cout << std::setw(8)  << (i * deltaTime)
                  << std::setw(12) << reading.battery.temperature
                  << std::setw(8)  << (cmd.fanSpeedPercent * 100.0f)
                  << std::setw(8)  << (cmd.pumpActive ? "ON" : "OFF")
                  << std::setw(16) << stateToString(cmd.state)
                  << std::setw(12) << totalFrames
                  << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::cout << std::string(64, '-') << "\n";
    std::cout << "Total CAN frames transmitted: " << totalFrames << "\n";
    std::cout << "CAN log saved to: build/can_log.csv\n";
    return 0;
}