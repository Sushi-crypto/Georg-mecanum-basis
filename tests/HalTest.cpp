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

// Test 5: Erkennt die HAL, wenn ein Rad klemmt?
TEST(HalTestGroup, DetectsStalledWheel)
{
    HalMecanum hal;

    // Wir befehlen Vorwärtsfahrt (1.0 m/s)
    hal.setForwardVelocity(1.0);

    // Aber der Enkoder sendet von Rad 0 stur 0 Ticks (Rad 0 klemmt!)
    hal.updateEncoderTicks(0, 0);

    // Die HAL muss jetzt erkennen, dass Rad 0 blockiert ist!
    CHECK_TRUE(hal.isWheelStalled(0));

    // Rad 1 dreht sich brav mit 2000 Ticks (1.0 m/s) -> Alles super!
    hal.updateEncoderTicks(1, 2000);
    CHECK_FALSE(hal.isWheelStalled(1));
}

// Test 6: Rotiert die Basis im Uhrzeigersinn?
TEST(HalTestGroup, RotateClockwiseSetsOppositeSideSpeed)
{
    HalMecanum hal;

    hal.setRotationVelocity(1.0);

    // Linke Räder (FL, HL) drehen vorwärts (+1.0), rechte Räder 
    // (FR, HR) rückwärts (-1.0)
    DOUBLES_EQUAL(1.0, hal.getWheelSpeed(0), 0.0001);   // Vorne Links
    DOUBLES_EQUAL(-1.0, hal.getWheelSpeed(1), 0.0001);  // Vorne Rechts
    DOUBLES_EQUAL(-1.0, hal.getWheelSpeed(2), 0.0001);  // Hinten Links
    DOUBLES_EQUAL(1.0, hal.getWheelSpeed(3), 0.0001);   // Hinten Rechts
}

// Test 7: Kombinierte Bewegung (Vorwärts + Seitwärts + Rotation gelichzeitig)
TEST(HalTestGroup, CombinedMovementCalculatesCorrect)
{
    HalMecanum hal;

    // Beispiel-Befehl: vx = 1.0 m/s, vy = 0.5 m/s, omega = 0.2 rad/s
    // Dieser Befehl existiert noch gar nicht!
    hal.drive(1.0, 0.5, 0.2);

    // Erwartete Mathematik nach der Mecanum-Formel:
    // FL (0): 1.0 + 0.5 + 0.2 = 1.7
    // FR (1): 1.0 - 0.5 - 0.2 = 0.3
    // HL (2): 1.0 - 0.5 + 0.2 = 0.7
    // HR (3): 1.0 + 0.5 - 0.2 = 1.3

    DOUBLES_EQUAL(1.7, hal.getWheelSpeed(0), 0.0001); // Vorne Links
    DOUBLES_EQUAL(0.3, hal.getWheelSpeed(1), 0.0001); // Vorne Rechts
    DOUBLES_EQUAL(0.7, hal.getWheelSpeed(2), 0.0001); // Hinten Links
    DOUBLES_EQUAL(1.3, hal.getWheelSpeed(3), 0.0001); // Hinten Rechts
}
