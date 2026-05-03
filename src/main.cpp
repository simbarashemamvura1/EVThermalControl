#include <iostream>
#include "types.h"

int main() {
    // Create a sample sensor reading to verify our structs work
    SensorReading reading;
    reading.timestampMs = 1000;
    reading.battery.temperature = 25.0f;
    reading.battery.stateOfCharge = 0.85f;
    reading.battery.voltage = 400.0f;
    reading.battery.current = 50.0f;
    reading.battery.isCharging = false;
    reading.thermal.ambientTemperature = 20.0f;
    reading.thermal.fanSpeedPercent = 0.0f;
    reading.thermal.pumpActive = false;
    reading.thermal.heaterActive = false;
    reading.vehicleSpeedKph = 80.0f;

    // Print it out to confirm everything works
    std::cout << "=== EV Thermal Control System ===" << std::endl;
    std::cout << "Time:        " << reading.timestampMs << " ms" << std::endl;
    std::cout << "Battery Temp: " << reading.battery.temperature << " C" << std::endl;
    std::cout << "SoC:          " << reading.battery.stateOfCharge * 100 << "%" << std::endl;
    std::cout << "Speed:        " << reading.vehicleSpeedKph << " kph" << std::endl;

    return 0;
}