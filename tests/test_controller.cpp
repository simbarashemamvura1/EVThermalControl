#include <gtest/gtest.h>
#include "SensorSimulator.h"
#include "ThermalController.h"

// ============================================================
//  Helper: build a SensorReading with specific battery temp
// ============================================================
SensorReading makeReading(float battTemp, float soc = 0.85f,
                           float speed = 80.0f) {
    SensorReading r;
    r.timestampMs               = 1000;
    r.vehicleSpeedKph           = speed;
    r.battery.temperature       = battTemp;
    r.battery.stateOfCharge     = soc;
    r.battery.voltage           = 380.0f;
    r.battery.current           = 100.0f;
    r.battery.isCharging        = false;
    r.thermal.ambientTemperature = 20.0f;
    r.thermal.coolantTemperature = 22.0f;
    r.thermal.fanSpeedPercent    = 0.0f;
    r.thermal.pumpActive         = false;
    r.thermal.heaterActive       = false;
    return r;
}

// ============================================================
//  THERMAL CONTROLLER TESTS
// ============================================================

// Normal temperature — system should be idle
TEST(ThermalController, NormalTempStaysNormal) {
    ThermalController controller;
    SensorReading r = makeReading(30.0f);
    ActuatorCommand cmd = controller.update(r);

    EXPECT_EQ(cmd.state, ThermalControlState::NORMAL);
    EXPECT_FLOAT_EQ(cmd.fanSpeedPercent, 0.0f);
    EXPECT_FALSE(cmd.pumpActive);
    EXPECT_FALSE(cmd.heaterActive);
}

// Above 45C — cooling must activate
TEST(ThermalController, CoolingActivatesAboveThreshold) {
    ThermalController controller;
    SensorReading r = makeReading(47.0f);
    ActuatorCommand cmd = controller.update(r);

    EXPECT_EQ(cmd.state, ThermalControlState::COOLING);
    EXPECT_GT(cmd.fanSpeedPercent, 0.0f) << "Fan should be running";
    EXPECT_TRUE(cmd.pumpActive)           << "Pump should be active";
    EXPECT_FALSE(cmd.heaterActive);
}

// Below 10C — heating must activate (cold start scenario)
TEST(ThermalController, HeatingActivatesBelowThreshold) {
    ThermalController controller;
    SensorReading r = makeReading(-5.0f);
    ActuatorCommand cmd = controller.update(r);

    EXPECT_EQ(cmd.state, ThermalControlState::HEATING);
    EXPECT_TRUE(cmd.heaterActive)          << "Heater should be ON";
    EXPECT_FALSE(cmd.pumpActive);
    EXPECT_FLOAT_EQ(cmd.fanSpeedPercent, 0.0f);
}

// Above 70C — fault state
TEST(ThermalController, FaultStateAbove70C) {
    ThermalController controller;
    SensorReading r = makeReading(72.0f);
    ActuatorCommand cmd = controller.update(r);

    EXPECT_EQ(cmd.state, ThermalControlState::FAULT);
    EXPECT_FLOAT_EQ(cmd.fanSpeedPercent, 1.0f) << "Fan full blast in fault";
    EXPECT_TRUE(cmd.pumpActive);
}

// Above 80C — safe shutdown
TEST(ThermalController, SafeShutdownAbove80C) {
    ThermalController controller;
    SensorReading r = makeReading(82.0f);
    ActuatorCommand cmd = controller.update(r);

    EXPECT_EQ(cmd.state, ThermalControlState::SAFE_SHUTDOWN);
    EXPECT_FLOAT_EQ(cmd.fanSpeedPercent, 0.0f) << "Everything off in shutdown";
    EXPECT_FALSE(cmd.pumpActive);
}

// Safe shutdown is sticky — stays shut down even if temp drops
TEST(ThermalController, SafeShutdownIsSticky) {
    ThermalController controller;

    // First trigger shutdown
    ActuatorCommand cmd = controller.update(makeReading(82.0f));
    EXPECT_EQ(cmd.state, ThermalControlState::SAFE_SHUTDOWN);

    // Now give it a normal temperature — should stay shut down
    cmd = controller.update(makeReading(30.0f));
    EXPECT_EQ(cmd.state, ThermalControlState::SAFE_SHUTDOWN)
        << "Safe shutdown should require manual reset";
}

// Hysteresis — cooling shouldn't turn off until temp drops to 40C
TEST(ThermalController, CoolingHysteresis) {
    ThermalController controller;

    // Trigger cooling
    controller.update(makeReading(47.0f));
    EXPECT_EQ(controller.getCurrentState(), ThermalControlState::COOLING);

    // Drop to 43C — still above COOLING_OFF (40C), should stay cooling
    ActuatorCommand cmd = controller.update(makeReading(43.0f));
    EXPECT_EQ(cmd.state, ThermalControlState::COOLING)
        << "Should stay in COOLING until temp drops below 40C";

    // Drop to 38C — below COOLING_OFF threshold, should return to normal
    cmd = controller.update(makeReading(38.0f));
    EXPECT_EQ(cmd.state, ThermalControlState::NORMAL)
        << "Should return to NORMAL below 40C";
}

// Fan speed should be proportional to temperature
TEST(ThermalController, FanSpeedProportionalToTemp) {
    ThermalController controller;

    ActuatorCommand cmd1 = controller.update(makeReading(46.0f));
    ThermalController controller2;
    ActuatorCommand cmd2 = controller2.update(makeReading(55.0f));

    EXPECT_GT(cmd2.fanSpeedPercent, cmd1.fanSpeedPercent)
        << "Higher temp should mean higher fan speed";
}
