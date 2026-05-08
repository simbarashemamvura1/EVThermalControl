#pragma once

#include "types.h"
#include <vector>
#include <string>
#include <fstream>

// ============================================================
//  FAULT SEVERITY LEVELS
// ============================================================
enum class FaultSeverity {
    INFO,       // Informational — no action needed
    WARNING,    // Something unusual — monitor closely
    CRITICAL    // Immediate action required
};

// ============================================================
//  DIAGNOSTICS MANAGER
//  Detects faults, assigns DTCs, logs fault history.
//  Supports fault injection for testing.
// ============================================================
class DiagnosticsManager {
public:
    DiagnosticsManager(const std::string& logFilePath);
    ~DiagnosticsManager();

    // Call every tick — checks reading for fault conditions
    void update(const SensorReading& reading,
                ThermalControlState controllerState);

    // Fault injection — simulate a broken sensor for N ticks
    void injectSensorFault(int durationTicks);

    // Get all active faults
    const std::vector<DiagnosticCode>& getActiveFaults() const;

    // Get full fault history
    const std::vector<DiagnosticCode>& getFaultHistory() const;

    // Check if a specific DTC is active
    bool isFaultActive(uint16_t dtcCode) const;

    // Clear all active faults (like a mechanic clearing codes)
    void clearFaults();

    // Print fault summary to console
    void printFaultReport() const;

    // Is sensor currently being injected with fault?
    bool isFaultInjected() const;

private:
    std::vector<DiagnosticCode> activeFaults_;
    std::vector<DiagnosticCode> faultHistory_;
    std::ofstream               logFile_;
    int                         faultInjectionTicksRemaining_;
    uint32_t                    lastTimestampMs_;

    // Fault detection methods
    void checkTemperatureFaults(const SensorReading& reading,
                                ThermalControlState state);
    void checkSensorValidity(const SensorReading& reading);
    void checkSoCFaults(const SensorReading& reading);

    // Log and register a fault
    void raiseFault(uint16_t code,
                    const std::string& description,
                    FaultSeverity severity,
                    uint32_t timestampMs);

    // Resolve a fault that is no longer active
    void resolveFault(uint16_t code);

    void logToFile(const DiagnosticCode& dtc,
                   FaultSeverity severity,
                   const std::string& event);

    const char* severityToString(FaultSeverity s) const;
};