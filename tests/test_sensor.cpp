#include <gtest/gtest.h>
#include "SensorSimulator.h"

// ============================================================
//  SENSOR SIMULATOR TESTS
// ============================================================

// Battery temperature should rise at high speed
TEST(SensorSimulator, TemperatureRisesWithSpeed) {
    SensorSimulator sensor(25.0f, 20.0f, 0.85f);

    float prevTemp = 25.0f;
    for (int i = 0; i < 10; i++) {
        SensorReading r = sensor.update(180.0f, 0.5f);
        EXPECT_GT(r.battery.temperature, prevTemp - 1.0f)
            << "Temperature should trend upward at high speed";
        prevTemp = r.battery.temperature;
    }
}

// SoC should drain over time
TEST(SensorSimulator, SoCDrainsOverTime) {
    SensorSimulator sensor(25.0f, 20.0f, 0.85f);

    SensorReading first = sensor.update(100.0f, 0.5f);
    for (int i = 0; i < 20; i++) sensor.update(100.0f, 0.5f);
    SensorReading last = sensor.update(100.0f, 0.5f);

    EXPECT_LT(last.battery.stateOfCharge, first.battery.stateOfCharge)
        << "SoC should decrease over time";
}

// SoC should never go below 0
TEST(SensorSimulator, SoCNeverGoesNegative) {
    SensorSimulator sensor(25.0f, 20.0f, 0.01f); // Start almost empty

    for (int i = 0; i < 100; i++) {
        SensorReading r = sensor.update(200.0f, 1.0f);
        EXPECT_GE(r.battery.stateOfCharge, 0.0f)
            << "SoC should never go below 0";
    }
}

// Temperature should never exceed physical limits
TEST(SensorSimulator, TemperatureStaysWithinPhysicalLimits) {
    SensorSimulator sensor(25.0f, 20.0f, 0.85f);

    for (int i = 0; i < 100; i++) {
        SensorReading r = sensor.update(200.0f, 1.0f);
        EXPECT_LE(r.battery.temperature, 90.1f)
            << "Temperature should not exceed physical max";
        EXPECT_GE(r.battery.temperature, -30.0f)
            << "Temperature should not go below physical min";
    }
}

// Higher speed should produce more heat
TEST(SensorSimulator, HigherSpeedProducesMoreHeat) {
    SensorSimulator slowSensor(25.0f, 20.0f, 0.85f);
    SensorSimulator fastSensor(25.0f, 20.0f, 0.85f);

    float slowTemp = 25.0f, fastTemp = 25.0f;
    for (int i = 0; i < 15; i++) {
        slowTemp = slowSensor.update(60.0f,  0.5f).battery.temperature;
        fastTemp = fastSensor.update(180.0f, 0.5f).battery.temperature;
    }

    EXPECT_GT(fastTemp, slowTemp)
        << "Higher speed should produce higher battery temperature";
}

// Voltage should correlate with SoC
TEST(SensorSimulator, VoltageCorrelatesWithSoC) {
    SensorSimulator sensor(25.0f, 20.0f, 0.85f);

    SensorReading r = sensor.update(0.0f, 0.5f);
    float expectedVoltageMin = 350.0f + (r.battery.stateOfCharge * 50.0f) - 5.0f;
    float expectedVoltageMax = 350.0f + (r.battery.stateOfCharge * 50.0f) + 5.0f;

    EXPECT_GE(r.battery.voltage, expectedVoltageMin);
    EXPECT_LE(r.battery.voltage, expectedVoltageMax);
}
