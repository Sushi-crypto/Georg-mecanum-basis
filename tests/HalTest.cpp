#include "CppUTest/TestHarness.h"

// Header Dateien
#include "HalMecanum.hpp"

//----------------------- Testbereich -----------------------

// Testgruppe für den HAL
TEST_GROUP(HalTestGroup)
{
    // Erstellt nach jedem Test Run eine komplett neue Testumgebung
    void setup() {}
    void teardown() {}
};

// Test 1: Sehen ob die HAL Räder auch exakt in der Nullstellung sind
TEST(HalTestGroup, InitialWheelSpeedsAreZero)
{
    // Reden müssen im Stillstand 0.0m/s zurückgeben!
    HalMecanum hal;
    for(int i = 0; i < 4; ++i){
        DOUBLES_EQUAL(0.0, hal.getWheelSpeed(i), 0.0001);
    }
}

// Test 2: Wenn wir 1.0 m/s vorwärts befehlen, müssen alle 4 Räder 
// auf 1.0 m/s stehen!
TEST(HalTestGroup, DriveForwardSetsAllWheelSpeeds)
{
    HalMecanum hal;

    hal.setForwardVelocity(1.0);

    for(int i = 0; i < 4; ++i) {
        DOUBLES_EQUAL(1.0, hal.getWheelSpeed(i), 0.0001);
    }
}

// Test 3: Seitfährtsfahren 
TEST(HalTestGroup, StrafeRightSetsDiagonalWheelSpeeds)
{
    HalMecanum hal;

    hal.setStrafeVelocity(1.0);

    // Vorne Links und Hinten Rechts drehen vorwärts (+1.0)
    DOUBLES_EQUAL(1.0, hal.getWheelSpeed(0), 0.0001);   // VL
    DOUBLES_EQUAL(-1.0, hal.getWheelSpeed(1), 0.0001);  // VR
    DOUBLES_EQUAL(-1.0, hal.getWheelSpeed(2), 0.0001);  // HL
    DOUBLES_EQUAL(1.0, hal.getWheelSpeed(3), 0.0001);   // HR
}

// Test 4: Prüfe ob rohe Enkoder-Ticks korrekt in m/s umgerechnet wird
TEST(HalTestGroup, EnkoderTicksAreConvertedToWheelSpeed)
{
    HalMecanum hal;

    // Angenommen: 2000 Ticks pro Sekunde entsprechen exakt 1.0 m/s
    // Dieser Befehl existiert noch nicht -> Test schlägt ROT fehl!
    hal.updateEncoderTicks(0, 2000);

    // Wir erwarten, dass getWheelSpeed(0) jetzt genau 1.0 m/s misst!
    DOUBLES_EQUAL(1.0,hal.getWheelSpeed(0), 0.0001);
}