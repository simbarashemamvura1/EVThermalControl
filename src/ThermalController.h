#pragma once

#include "types.h"

// ============================================================
//  THERMAL CONTROLLER
//  State-machine-based controller that reads sensor data
//  and outputs actuator commands every simulation tick.
//  This is the core control logic of the entire system.
// ============================================================

struct ActuatorCommand {
    float    fanSpeedPercent;
    bool     pumpActive;
    bool     heaterActive;
    ThermalControlState state;
};

class ThermalController {
public:
    ThermalController();

    // Call every tick — reads sensor, returns actuator commands
    ActuatorCommand update(const SensorReading& SensorReading);

    // Getters for logging
    ThermalControlState getCurrentState() const;
    int getFaultCode() const;

    // Reset controller to NORMAL — used when starting a new scenario
    // Immediately exits SAFE SHUTDOWN so badge updates within one tick
    void reset() { currentState_ = ThermalControlState::NORMAL; faultCode_ = 0; }

private:
    ThermalControlState currentState_;
    int faultCode_;

    // Hysteresis Thresholds - prevents rapid state switching
    // (e.g. dont turn cooling off the moment temp drops 1 degree)
    // Safe shutdown above this

    // State transition logic
    ThermalControlState computeNextState(const SensorReading& reading);

    // Actuator output logic
    ActuatorCommand computeActuatorCommand(const SensorReading& reading);

    // Validate sensor data is within physical bounds
    bool isSensorValid(const SensorReading& reading);
};