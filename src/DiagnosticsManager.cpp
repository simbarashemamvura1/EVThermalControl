#include "DiagnosticsManager.h"
#include "ThermalController.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

// ============================================================
//  DTC CODE DEFINITIONS
//  These would normally live in a .dbc or ODX file
// ============================================================
namespace DTC {
    constexpr uint16_t OVERTEMP_WARNING    = 0x0001;
    constexpr uint16_t OVERTEMP_CRITICAL   = 0x0002;
    constexpr uint16_t UNDERTEMP_WARNING   = 0x0003;
    constexpr uint16_t SENSOR_OUT_OF_RANGE = 0x0004;
    constexpr uint16_t SENSOR_INJECTED     = 0x0005;
    constexpr uint16_t LOW_SOC_WARNING     = 0x0006;
    constexpr uint16_t CRITICAL_SOC        = 0x0007;
    constexpr uint16_t SAFE_SHUTDOWN       = 0x0008;
}

// ============================================================
//  CONSTRUCTOR / DESTRUCTOR
// ============================================================
DiagnosticsManager::DiagnosticsManager(const std::string& logFilePath)
    : faultInjectionTicksRemaining_(0)
    , lastTimestampMs_(0)
{
    logFile_.open(logFilePath, std::ios::out | std::ios::trunc);
    if (logFile_.is_open()) {
        logFile_ << "timestamp_ms,event,dtc_code,"
                 << "description,severity\n";
        std::cout << "[DIAG] DiagnosticsManager initialized. "
                  << "Logging to: " << logFilePath << "\n";
    }
}

DiagnosticsManager::~DiagnosticsManager() {
    if (logFile_.is_open()) logFile_.close();
}

// ============================================================
//  UPDATE — call every simulation tick
// ============================================================
void DiagnosticsManager::update(const SensorReading& reading,
                                 ThermalControlState controllerState) {
    lastTimestampMs_ = reading.timestampMs;

    // Decrement fault injection counter
    if (faultInjectionTicksRemaining_ > 0) {
        faultInjectionTicksRemaining_--;
        raiseFault(DTC::SENSOR_INJECTED,
                   "Fault injection active — sensor data unreliable",
                   FaultSeverity::CRITICAL,
                   reading.timestampMs);
        if (faultInjectionTicksRemaining_ == 0) {
            resolveFault(DTC::SENSOR_INJECTED);
            std::cout << "[DIAG] Fault injection ended. System recovering.\n";
        }
    }

    // Run all fault checks
    checkTemperatureFaults(reading, controllerState);
    checkSensorValidity(reading);
    checkSoCFaults(reading);
}

// ============================================================
//  FAULT INJECTION — simulate sensor failure for N ticks
// ============================================================
void DiagnosticsManager::injectSensorFault(int durationTicks) {
    faultInjectionTicksRemaining_ = durationTicks;
    std::cout << "[DIAG] *** FAULT INJECTED — sensor failure simulated for "
              << durationTicks << " ticks ***\n";
}

// ============================================================
//  TEMPERATURE FAULT CHECKS
// ============================================================
void DiagnosticsManager::checkTemperatureFaults(
    const SensorReading& reading,
    ThermalControlState state)
{
    float temp = reading.battery.temperature;

    // Safe shutdown state
    if (state == ThermalControlState::SAFE_SHUTDOWN) {
        raiseFault(DTC::SAFE_SHUTDOWN,
                   "System in SAFE_SHUTDOWN — battery critically hot",
                   FaultSeverity::CRITICAL,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::SAFE_SHUTDOWN);
    }

    // Critical overtemp
    if (temp > 75.0f) {
        raiseFault(DTC::OVERTEMP_CRITICAL,
                   "Battery temperature critically high",
                   FaultSeverity::CRITICAL,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::OVERTEMP_CRITICAL);
    }

    // Overtemp warning
    if (temp > 60.0f && temp <= 75.0f) {
        raiseFault(DTC::OVERTEMP_WARNING,
                   "Battery temperature above warning threshold",
                   FaultSeverity::WARNING,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::OVERTEMP_WARNING);
    }

    // Undertemp warning
    if (temp < 5.0f) {
        raiseFault(DTC::UNDERTEMP_WARNING,
                   "Battery temperature dangerously low",
                   FaultSeverity::WARNING,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::UNDERTEMP_WARNING);
    }
}

// ============================================================
//  SENSOR VALIDITY CHECKS
// ============================================================
void DiagnosticsManager::checkSensorValidity(const SensorReading& reading) {
    bool invalid = false;

    if (reading.battery.temperature < -40.0f ||
        reading.battery.temperature > 100.0f) invalid = true;
    if (reading.battery.voltage < 200.0f ||
        reading.battery.voltage > 500.0f)       invalid = true;
    if (reading.battery.stateOfCharge < 0.0f ||
        reading.battery.stateOfCharge > 1.0f)   invalid = true;

    if (invalid || isFaultInjected()) {
        raiseFault(DTC::SENSOR_OUT_OF_RANGE,
                   "Sensor reading out of physical bounds",
                   FaultSeverity::CRITICAL,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::SENSOR_OUT_OF_RANGE);
    }
}

// ============================================================
//  SOC FAULT CHECKS
// ============================================================
void DiagnosticsManager::checkSoCFaults(const SensorReading& reading) {
    float soc = reading.battery.stateOfCharge * 100.0f;

    if (soc < 5.0f) {
        raiseFault(DTC::CRITICAL_SOC,
                   "Battery SoC critically low — charge immediately",
                   FaultSeverity::CRITICAL,
                   reading.timestampMs);
    } else if (soc < 15.0f) {
        raiseFault(DTC::LOW_SOC_WARNING,
                   "Battery SoC low warning",
                   FaultSeverity::WARNING,
                   reading.timestampMs);
    } else {
        resolveFault(DTC::LOW_SOC_WARNING);
        resolveFault(DTC::CRITICAL_SOC);
    }
}

// ============================================================
//  RAISE FAULT — add to active list if not already there
// ============================================================
void DiagnosticsManager::raiseFault(uint16_t code,
                                     const std::string& description,
                                     FaultSeverity severity,
                                     uint32_t timestampMs) {
    // Check if already active
    for (auto& f : activeFaults_) {
        if (f.code == code) return; // Already active, don't duplicate
    }

    DiagnosticCode dtc;
    dtc.code        = code;
    dtc.description = description;
    dtc.timestampMs = timestampMs;
    dtc.isActive    = true;

    activeFaults_.push_back(dtc);
    faultHistory_.push_back(dtc);

    std::cout << "[DTC 0x" << std::hex << std::uppercase << code
              << std::dec << "] " << severityToString(severity)
              << ": " << description << "\n";

    logToFile(dtc, severity, "RAISED");
}

// ============================================================
//  RESOLVE FAULT — mark as inactive
// ============================================================
void DiagnosticsManager::resolveFault(uint16_t code) {
    auto it = std::find_if(activeFaults_.begin(), activeFaults_.end(),
        [code](const DiagnosticCode& d){ return d.code == code; });

    if (it != activeFaults_.end()) {
        it->isActive = false;
        logToFile(*it, FaultSeverity::INFO, "RESOLVED");
        activeFaults_.erase(it);
    }
}

// ============================================================
//  PRINT FAULT REPORT
// ============================================================
void DiagnosticsManager::printFaultReport() const {
    std::cout << "\n=== DIAGNOSTIC FAULT REPORT ===\n";
    std::cout << "Active faults:  " << activeFaults_.size() << "\n";
    std::cout << "Total in history: " << faultHistory_.size() << "\n\n";

    if (faultHistory_.empty()) {
        std::cout << "No faults recorded.\n";
        return;
    }

    std::cout << std::left
              << std::setw(10) << "DTC"
              << std::setw(12) << "Time(ms)"
              << std::setw(8)  << "Active"
              << "Description\n";
    std::cout << std::string(60, '-') << "\n";

    for (const auto& dtc : faultHistory_) {
        std::cout << "0x" << std::hex << std::uppercase
                  << std::setw(8) << dtc.code
                  << std::dec
                  << std::setw(12) << dtc.timestampMs
                  << std::setw(8)  << (dtc.isActive ? "YES" : "NO")
                  << dtc.description << "\n";
    }
    std::cout << std::string(60, '-') << "\n";
}

// ============================================================
//  GETTERS
// ============================================================
const std::vector<DiagnosticCode>& DiagnosticsManager::getActiveFaults() const {
    return activeFaults_;
}

const std::vector<DiagnosticCode>& DiagnosticsManager::getFaultHistory() const {
    return faultHistory_;
}

bool DiagnosticsManager::isFaultActive(uint16_t dtcCode) const {
    for (const auto& f : activeFaults_) {
        if (f.code == dtcCode) return true;
    }
    return false;
}

void DiagnosticsManager::clearFaults() {
    activeFaults_.clear();
    std::cout << "[DIAG] All active faults cleared.\n";
}

bool DiagnosticsManager::isFaultInjected() const {
    return faultInjectionTicksRemaining_ > 0;
}

// ============================================================
//  LOG TO FILE
// ============================================================
void DiagnosticsManager::logToFile(const DiagnosticCode& dtc,
                                    FaultSeverity severity,
                                    const std::string& event) {
    if (!logFile_.is_open()) return;
    logFile_ << dtc.timestampMs << ","
             << event << ","
             << "0x" << std::hex << std::uppercase << dtc.code
             << std::dec << ","
             << dtc.description << ","
             << severityToString(severity) << "\n";
}

const char* DiagnosticsManager::severityToString(FaultSeverity s) const {
    switch(s) {
        case FaultSeverity::INFO:     return "INFO";
        case FaultSeverity::WARNING:  return "WARNING";
        case FaultSeverity::CRITICAL: return "CRITICAL";
        default:                      return "UNKNOWN";
    }
}