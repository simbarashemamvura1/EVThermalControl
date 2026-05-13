#include <gtest/gtest.h>
#include "DiagnosticsManager.h"
#include "ThermalController.h"

// ============================================================
//  Helper: build a basic sensor reading
// ============================================================
SensorReading makeDiagReading(float battTemp, float soc = 0.85f,
                               uint32_t ts = 1000) {
    SensorReading r;
    r.timestampMs               = ts;
    r.vehicleSpeedKph           = 80.0f;
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
//  DIAGNOSTICS MANAGER TESTS
// ============================================================

// No faults at normal temperature
TEST(DiagnosticsManager, NoFaultsAtNormalTemp) {
    DiagnosticsManager diag("build/test_dtc.csv");
    SensorReading r = makeDiagReading(30.0f);
    diag.update(r, ThermalControlState::NORMAL);

    EXPECT_EQ(diag.getActiveFaults().size(), 0)
        << "No faults expected at normal temperature";
}

// Overtemp warning raised above 60C
TEST(DiagnosticsManager, OvertempWarningRaisedAbove60C) {
    DiagnosticsManager diag("build/test_dtc.csv");
    SensorReading r = makeDiagReading(65.0f);
    diag.update(r, ThermalControlState::FAULT);

    EXPECT_TRUE(diag.isFaultActive(0x0001))
        << "DTC 0x0001 overtemp warning should be active above 60C";
}

// Critical overtemp raised above 75C
TEST(DiagnosticsManager, CriticalOvertempAbove75C) {
    DiagnosticsManager diag("build/test_dtc.csv");
    SensorReading r = makeDiagReading(78.0f);
    diag.update(r, ThermalControlState::FAULT);

    EXPECT_TRUE(diag.isFaultActive(0x0002))
        << "DTC 0x0002 critical overtemp should be active above 75C";
}

// Fault injection test — DTC raised during injection
TEST(DiagnosticsManager, FaultInjectionRaisesDTC) {
    DiagnosticsManager diag("build/test_dtc.csv");
    diag.injectSensorFault(3);

    SensorReading r = makeDiagReading(30.0f);
    diag.update(r, ThermalControlState::NORMAL);

    EXPECT_TRUE(diag.isFaultInjected())
        << "System should report fault injection active";
    EXPECT_GT(diag.getActiveFaults().size(), 0)
        << "At least one fault should be active during injection";
}

// Fault injection resolves after N ticks
TEST(DiagnosticsManager, FaultInjectionResolvesAfterDuration) {
    DiagnosticsManager diag("build/test_dtc.csv");
    diag.injectSensorFault(3);

    SensorReading r = makeDiagReading(30.0f);

    // Tick 1 — still injected
    diag.update(r, ThermalControlState::NORMAL);
    EXPECT_TRUE(diag.isFaultInjected());

    // Tick 2 — still injected
    diag.update(r, ThermalControlState::NORMAL);
    EXPECT_TRUE(diag.isFaultInjected());

    // Tick 3 — still injected (last tick)
    diag.update(r, ThermalControlState::NORMAL);
    EXPECT_FALSE(diag.isFaultInjected())
        << "Fault injection should resolve after duration expires";
}

// Fault resolves when condition clears
TEST(DiagnosticsManager, FaultResolvesWhenConditionClears) {
    DiagnosticsManager diag("build/test_dtc.csv");

    // Raise overtemp warning
    diag.update(makeDiagReading(65.0f), ThermalControlState::FAULT);
    EXPECT_TRUE(diag.isFaultActive(0x0001));

    // Temperature back to normal — fault should resolve
    diag.update(makeDiagReading(30.0f), ThermalControlState::NORMAL);
    EXPECT_FALSE(diag.isFaultActive(0x0001))
        << "Overtemp warning should resolve when temp returns to normal";
}

// Low SoC warning
TEST(DiagnosticsManager, LowSoCWarningRaised) {
    DiagnosticsManager diag("build/test_dtc.csv");
    SensorReading r = makeDiagReading(30.0f, 0.10f); // 10% SoC
    diag.update(r, ThermalControlState::NORMAL);

    EXPECT_TRUE(diag.isFaultActive(0x0006))
        << "DTC 0x0006 low SoC warning should be active below 15%";
}

// Fault history grows over time
TEST(DiagnosticsManager, FaultHistoryGrowsOverTime) {
    DiagnosticsManager diag("build/test_dtc.csv");

    diag.update(makeDiagReading(65.0f, 0.85f, 1000),
                ThermalControlState::FAULT);
    diag.update(makeDiagReading(78.0f, 0.85f, 2000),
                ThermalControlState::FAULT);

    EXPECT_GE(diag.getFaultHistory().size(), 2)
        << "Fault history should accumulate across ticks";
}
