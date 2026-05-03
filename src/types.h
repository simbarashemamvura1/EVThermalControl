#pragma once

#include <cstdint>
#include <string>

// ============================================================
//  BATTERY STATE
//  Represents the current condition of the EV battery pack
// ============================================================
struct BatteryState {
    float temperature;      // Battery temp in Celsius
    float stateOfCharge;    // SoC: 0.0 = empty, 1.0 = full
    float voltage;          // Pack voltage in Volts
    float current;          // Current draw in Amps (+ discharge, - charge)
    bool  isCharging;       // True if plugged in and charging
};

// ============================================================
//  THERMAL STATE
//  Represents the thermal management system's current status
// ============================================================
struct ThermalState {
    float ambientTemperature;   // Outside air temp in Celsius
    float coolantTemperature;   // Coolant loop temp in Celsius
    float fanSpeedPercent;      // Fan speed: 0.0 = off, 1.0 = max
    bool  pumpActive;           // Coolant pump on/off
    bool  heaterActive;         // Battery heater on/off
};

// ============================================================
//  SENSOR READING
//  A timestamped snapshot from the sensor layer
// ============================================================
struct SensorReading {
    uint32_t    timestampMs;    // Simulation time in milliseconds
    BatteryState  battery;
    ThermalState  thermal;
    float         vehicleSpeedKph; // Vehicle speed in km/h
};

// ============================================================
//  CAN FRAME
//  Mimics a real CAN bus message structure
// ============================================================
struct CANFrame {
    uint32_t messageId;         // 11-bit CAN ID (e.g. 0x100)
    uint8_t  dataLengthCode;    // DLC: number of bytes in payload (max 8)
    uint8_t  payload[8];        // Raw data bytes
    uint32_t timestampMs;       // When this frame was sent
};

// ============================================================
//  SYSTEM STATE (Control layer output)
//  What mode is the thermal controller currently in?
// ============================================================
enum class ThermalControlState {
    NORMAL,          // Everything within range, no action needed
    COOLING,         // Battery too hot, cooling activated
    HEATING,         // Battery too cold, heater activated
    FAULT,           // Something went wrong, logged a DTC
    SAFE_SHUTDOWN    // Critical fault, system shutting down safely
};

// ============================================================
//  DIAGNOSTIC TROUBLE CODE
//  Logged whenever the diagnostics system detects a fault
// ============================================================
struct DiagnosticCode {
    uint16_t    code;           // DTC number e.g. 0x0001
    std::string description;    // Human-readable fault description
    uint32_t    timestampMs;    // When the fault was detected
    bool        isActive;       // Still happening or historical?
};