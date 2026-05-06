#pragma once

#include "types.h"
#include "ThermalController.h"
#include <vector>
#include <queue>
#include <string>
#include <fstream>
#include <functional>

// ============================================================
//  CAN MESSAGE IDs — real automotive style
//  In a real vehicle these are defined in a .dbc file
// ============================================================
namespace CANID {
    constexpr uint32_t BATTERY_STATUS    = 0x100; // Sensor → Controller
    constexpr uint32_t THERMAL_STATUS    = 0x101; // Sensor → Controller
    constexpr uint32_t ACTUATOR_COMMAND  = 0x200; // Controller → Actuator
    constexpr uint32_t FAULT_STATUS      = 0x300; // Diagnostics → All
}

// ============================================================
//  CAN BUS
//  Simulates an in-process CAN bus. Modules publish frames,
//  subscribers receive them. All traffic is logged to file.
// ============================================================
class CANBus {
public:
    // Callback type — called when a matching frame arrives
    using MessageCallback = std::function<void(const CANFrame&)>;

    CANBus(const std::string& logFilePath);
    ~CANBus();

    // Publish a frame onto the bus
    void publish(const CANFrame& frame);

    // Subscribe to a specific message ID
    void subscribe(uint32_t messageId, MessageCallback callback);

    // Process all pending messages — call once per tick
    void processPendingMessages();

    // Helper: encode a SensorReading into CAN frames
    static CANFrame encodeBatteryStatus(const SensorReading& reading);
    static CANFrame encodeThermalStatus(const SensorReading& reading);

    // Helper: encode ActuatorCommand into a CAN frame
    static CANFrame encodeActuatorCommand(const ActuatorCommand& cmd,
                                           uint32_t timestampMs);

    // Helper: decode frames back to readable values
    static float decodeFloat(const CANFrame& frame, int byteOffset);

private:
    struct Subscription {
        uint32_t        messageId;
        MessageCallback callback;
    };

    std::queue<CANFrame>       txQueue_;      // Outgoing message queue
    std::vector<Subscription>  subscribers_;  // Registered listeners
    std::ofstream              logFile_;      // CAN traffic log

    void logFrame(const CANFrame& frame, const std::string& direction);
    void encodeFloatIntoBytes(float value, uint8_t* bytes, int offset);
};