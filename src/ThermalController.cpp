#include "ThermalController.h"
#include <algorithm>
#include <iostream>

// Bring threshold constants into scope
constexpr float COOLING_ON_TEMP  = 45.0f;
constexpr float COOLING_OFF_TEMP = 40.0f;
constexpr float HEATING_ON_TEMP  = 10.0f;
constexpr float HEATING_OFF_TEMP = 15.0f;
constexpr float FAULT_TEMP       = 70.0f;
constexpr float SHUTDOWN_TEMP    = 80.0f;

// ============================================================
//  CONSTRUCTOR
// ============================================================
ThermalController::ThermalController()
    : currentState_(ThermalControlState::NORMAL)
    , faultCode_(0)
{
}

// ============================================================
//  UPDATE — called every simulation tick
// ============================================================
ActuatorCommand ThermalController::update(const SensorReading& reading) {

    // 1. Validate sensor data first
    if (!isSensorValid(reading)) {
        currentState_ = ThermalControlState::FAULT;
        faultCode_    = 0x0002; // DTC: sensor out of range
        std::cout << "[FAULT] Sensor invalid — DTC 0x0002\n";
    } else {
        // 2. Compute next state based on current readings
        currentState_ = computeNextState(reading);
    }

    // 3. Compute and return actuator commands for this state
    return computeActuatorCommand(reading);
}

// ============================================================
//  STATE MACHINE — core control logic
// ============================================================
ThermalControlState ThermalController::computeNextState(
    const SensorReading& reading)
{
    float temp = reading.battery.temperature;

    // Once in SAFE_SHUTDOWN, stay there — requires manual reset
    if (currentState_ == ThermalControlState::SAFE_SHUTDOWN) {
        return ThermalControlState::SAFE_SHUTDOWN;
    }

    // Critical temperature — safe shutdown
    if (temp >= SHUTDOWN_TEMP) {
        faultCode_ = 0x0001; // DTC: critical overtemp
        std::cout << "[SAFE SHUTDOWN] Battery temp critical: "
                  << temp << "C — DTC 0x0001\n";
        return ThermalControlState::SAFE_SHUTDOWN;
    }

    // Fault temperature
    if (temp >= FAULT_TEMP) {
        faultCode_ = 0x0003; // DTC: overtemp warning
        std::cout << "[FAULT] Overtemp warning: " << temp
                  << "C — DTC 0x0003\n";
        return ThermalControlState::FAULT;
    }

    // Cooling logic with hysteresis
    // Turn ON cooling if temp exceeds upper threshold
    // Turn OFF cooling only when temp drops below lower threshold
    if (currentState_ == ThermalControlState::COOLING) {
        if (temp < COOLING_OFF_TEMP) {
            std::cout << "[NORMAL] Cooling complete. Temp: " << temp << "C\n";
            return ThermalControlState::NORMAL;
        }
        return ThermalControlState::COOLING; // Stay cooling
    }
    if (temp >= COOLING_ON_TEMP) {
        std::cout << "[COOLING] Activated. Temp: " << temp << "C\n";
        return ThermalControlState::COOLING;
    }

    // Heating logic with hysteresis
    if (currentState_ == ThermalControlState::HEATING) {
        if (temp > HEATING_OFF_TEMP) {
            std::cout << "[NORMAL] Heating complete. Temp: " << temp << "C\n";
            return ThermalControlState::NORMAL;
        }
        return ThermalControlState::HEATING; // Stay heating
    }
    if (temp <= HEATING_ON_TEMP) {
        std::cout << "[HEATING] Activated. Temp: " << temp << "C\n";
        return ThermalControlState::HEATING;
    }

    return ThermalControlState::NORMAL;
}

// ============================================================
//  ACTUATOR COMMANDS — what to do in each state
// ============================================================
ActuatorCommand ThermalController::computeActuatorCommand(
    const SensorReading& reading)
{
    ActuatorCommand cmd;
    cmd.state        = currentState_;
    cmd.pumpActive   = false;
    cmd.heaterActive = false;
    cmd.fanSpeedPercent = 0.0f;

    float temp = reading.battery.temperature;

    switch (currentState_) {
        case ThermalControlState::NORMAL:
            // Everything off — passive cooling only
            cmd.fanSpeedPercent = 0.0f;
            cmd.pumpActive      = false;
            cmd.heaterActive    = false;
            break;

        case ThermalControlState::COOLING:
            // Fan speed proportional to how far over threshold we are
            // e.g. at 55C (10 over threshold) → 100% fan
            cmd.fanSpeedPercent = std::min(1.0f, (temp - COOLING_OFF_TEMP) / 15.0f);
            cmd.pumpActive      = true;
            cmd.heaterActive    = false;
            break;

        case ThermalControlState::HEATING:
            cmd.fanSpeedPercent = 0.0f;
            cmd.pumpActive      = false;
            cmd.heaterActive    = true;
            break;

        case ThermalControlState::FAULT:
            // Run cooling at full blast in fault state
            cmd.fanSpeedPercent = 1.0f;
            cmd.pumpActive      = true;
            cmd.heaterActive    = false;
            break;

        case ThermalControlState::SAFE_SHUTDOWN:
            // Everything off — system is shutting down
            cmd.fanSpeedPercent = 0.0f;
            cmd.pumpActive      = false;
            cmd.heaterActive    = false;
            break;
    }

    return cmd;
}

// ============================================================
//  SENSOR VALIDATION
// ============================================================
bool ThermalController::isSensorValid(const SensorReading& reading) {
    // Check battery temp is within physical bounds
    if (reading.battery.temperature < -40.0f ||
        reading.battery.temperature > 100.0f) return false;

    // Check SoC is 0-100%
    if (reading.battery.stateOfCharge < 0.0f ||
        reading.battery.stateOfCharge > 1.0f)  return false;

    // Check voltage is within EV pack range
    if (reading.battery.voltage < 200.0f ||
        reading.battery.voltage > 500.0f)       return false;

    return true;
}

// ============================================================
//  GETTERS
// ============================================================
ThermalControlState ThermalController::getCurrentState() const {
    return currentState_;
}

int ThermalController::getFaultCode() const {
    return faultCode_;
}