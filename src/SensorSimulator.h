#pragma once

#include "types.h"
#include <random>

// ============================================================
//  SENSOR SIMULATOR
//  Generates realistic EV sensor data over time.
//  In a real vehicle this would read from hardware.
//  Here we simulate physics-based behaviour with noise.
// ============================================================
class SensorSimulator {
public:
    // Constructor — sets initial conditions
    SensorSimulator(float initialBatteryTemp,
                    float initialAmbientTemp,
                    float initialSoC);

    // Call this every simulation tick to get a new reading
    SensorReading update(float vehicleSpeedKph, float deltaTimeSeconds);

    // Reset simulation back to initial conditions
    void reset();

private:
    // Current simulated state
    float batteryTemp_;       // Current battery temperature (C)
    float ambientTemp_;       // Ambient temperature (C) — fixed for now
    float stateOfCharge_;     // SoC: 0.0 to 1.0
    float coolantTemp_;       // Coolant loop temperature (C)
    uint32_t tickCount_;      // How many ticks have elapsed

    // Initial conditions (for reset)
    float initBatteryTemp_;
    float initAmbientTemp_;
    float initSoC_;

    // Random noise generator
    std::mt19937 rng_;
    std::normal_distribution<float> noiseDist_;

    // Physics helpers
    float computeHeatGeneration(float speedKph, float socFraction);
    float computeCooling(float battTemp, float ambientTemp, float fanSpeed);
    float addNoise(float value, float magnitude);
};