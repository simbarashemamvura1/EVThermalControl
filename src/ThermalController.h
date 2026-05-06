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

    //Call every Tick- reads sensor, returns actuator commands
    ActuatorCommand update(const SensorReading& SensorReading);
    //Getters for logging
    ThermalControlState getCurrentState() const;
    int getFaultCode() const;

private:
    ThermalControlState currentState_;
    int faultCode_;

    //Hysteresis Thresho;ds - prevents rapid state switching
    // (e.g. dont turn cooling off the moment temp drops 1 degree)
  //Safe shutdown above this

    //State transision logic
    ThermalControlState computeNextState(const SensorReading& reading);

    //Actuator output logic
    ActuatorCommand computeActuatorCommand(const SensorReading& reading);

    //Validate sensor data is within physical bounds
    bool isSensorValid(const SensorReading& reading);
};