#include "CANBus.h"
#include "ThermalController.h"
#include <iostream>
#include <iomanip>
#include <cstring>

// ============================================================
//  CONSTRUCTOR / DESTRUCTOR
// ============================================================
CANBus::CANBus(const std::string& logFilePath) {
    logFile_.open(logFilePath, std::ios::out | std::ios::trunc);
    if (logFile_.is_open()) {
        logFile_ << "timestamp_ms,direction,msg_id,dlc,"
                 << "b0,b1,b2,b3,b4,b5,b6,b7\n";
        std::cout << "[CAN] Bus initialized. Logging to: "
                  << logFilePath << "\n";
    } else {
        std::cerr << "[CAN] Warning: could not open log file.\n";
    }
}

CANBus::~CANBus() {
    if (logFile_.is_open()) logFile_.close();
}

// ============================================================
//  PUBLISH — put a frame on the bus
// ============================================================
void CANBus::publish(const CANFrame& frame) {
    txQueue_.push(frame);
    logFrame(frame, "TX");
}

// ============================================================
//  SUBSCRIBE — register a callback for a message ID
// ============================================================
void CANBus::subscribe(uint32_t messageId, MessageCallback callback) {
    subscribers_.push_back({messageId, callback});
}

// ============================================================
//  PROCESS — dispatch queued messages to subscribers
//  Call once per simulation tick
// ============================================================
void CANBus::processPendingMessages() {
    while (!txQueue_.empty()) {
        CANFrame frame = txQueue_.front();
        txQueue_.pop();

        // Deliver to all matching subscribers
        for (auto& sub : subscribers_) {
            if (sub.messageId == frame.messageId) {
                sub.callback(frame);
            }
        }
    }
}

// ============================================================
//  ENCODE: SensorReading → CAN frames
//  Packs float values into 8-byte payloads
//  2 bytes per value (scaled integer encoding)
// ============================================================
CANFrame CANBus::encodeBatteryStatus(const SensorReading& reading) {
    CANFrame frame;
    frame.messageId      = CANID::BATTERY_STATUS;
    frame.dataLengthCode = 8;
    frame.timestampMs    = reading.timestampMs;

    // Pack: [temp(2B)][SoC%(2B)][voltage(2B)][current(2B)]
    // Scale: temp * 10, SoC * 100, voltage * 10, current * 10
    int16_t temp    = static_cast<int16_t>(reading.battery.temperature * 10);
    int16_t soc     = static_cast<int16_t>(reading.battery.stateOfCharge * 1000);
    int16_t voltage = static_cast<int16_t>(reading.battery.voltage * 10);
    int16_t current = static_cast<int16_t>(reading.battery.current * 10);

    memcpy(&frame.payload[0], &temp,    2);
    memcpy(&frame.payload[2], &soc,     2);
    memcpy(&frame.payload[4], &voltage, 2);
    memcpy(&frame.payload[6], &current, 2);

    return frame;
}

CANFrame CANBus::encodeThermalStatus(const SensorReading& reading) {
    CANFrame frame;
    frame.messageId      = CANID::THERMAL_STATUS;
    frame.dataLengthCode = 6;
    frame.timestampMs    = reading.timestampMs;

    // Pack: [ambientTemp(2B)][coolantTemp(2B)][fanSpeed%(2B)]
    int16_t ambient  = static_cast<int16_t>(reading.thermal.ambientTemperature * 10);
    int16_t coolant  = static_cast<int16_t>(reading.thermal.coolantTemperature * 10);
    int16_t fanSpeed = static_cast<int16_t>(reading.thermal.fanSpeedPercent * 1000);

    memcpy(&frame.payload[0], &ambient,  2);
    memcpy(&frame.payload[2], &coolant,  2);
    memcpy(&frame.payload[4], &fanSpeed, 2);

    return frame;
}

CANFrame CANBus::encodeActuatorCommand(const ActuatorCommand& cmd,
                                        uint32_t timestampMs) {
    CANFrame frame;
    frame.messageId      = CANID::ACTUATOR_COMMAND;
    frame.dataLengthCode = 4;
    frame.timestampMs    = timestampMs;

    // Pack: [fanSpeed%(2B)][flags(1B)][state(1B)]
    int16_t fanSpeed = static_cast<int16_t>(cmd.fanSpeedPercent * 1000);
    uint8_t flags    = (cmd.pumpActive ? 0x01 : 0x00) |
                       (cmd.heaterActive ? 0x02 : 0x00);
    uint8_t state    = static_cast<uint8_t>(cmd.state);

    memcpy(&frame.payload[0], &fanSpeed, 2);
    frame.payload[2] = flags;
    frame.payload[3] = state;

    return frame;
}

// ============================================================
//  DECODE — extract a float from a frame payload
// ============================================================
float CANBus::decodeFloat(const CANFrame& frame, int byteOffset) {
    int16_t raw = 0;
    memcpy(&raw, &frame.payload[byteOffset], 2);
    return static_cast<float>(raw) / 10.0f;
}

// ============================================================
//  LOG — write frame to CSV log file
// ============================================================
void CANBus::logFrame(const CANFrame& frame, const std::string& direction) {
    if (!logFile_.is_open()) return;

    logFile_ << frame.timestampMs << ","
             << direction << ","
             << "0x" << std::hex << std::uppercase << frame.messageId
             << std::dec << ","
             << static_cast<int>(frame.dataLengthCode);

    for (int i = 0; i < 8; i++) {
        logFile_ << "," << std::hex << std::uppercase
                 << std::setw(2) << std::setfill('0')
                 << static_cast<int>(frame.payload[i]);
    }
    logFile_ << std::dec << "\n";
}

void CANBus::encodeFloatIntoBytes(float value, uint8_t* bytes, int offset) {
    int16_t scaled = static_cast<int16_t>(value * 10);
    memcpy(bytes + offset, &scaled, 2);
}