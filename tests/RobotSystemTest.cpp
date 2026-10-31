#include "CppUTest/TestHarness.h"
#include "LidarMessung.h"
#include "HalMecanum.hpp"
#include "RobotSystem.hpp"
#include <vector>

// Testgruppe für Robotsystems
TEST_GROUP(RobotSystemTestGroup)
{
    void setup() {}
    void teardown() {}
};

// Test 1: Lässt sich das RobotSystem sauber initialisieren?
TEST(RobotSystemTestGroup, SystemRichtigInitiliasiert)
{
    // 1. Wir erstellen unsere Hardware (HAL)
    HalMecanum hal;

    // 2. Wir erstellen das Gesamtsystem (5.0m x 5.0m Karte, 0.05m Auflösung)
    RobotSystem robot(hal, 5.0, 5.0, 0.05);

    // 3. Nach dem Start muss der Roboter exakt auf (0,0) stehen!
    DOUBLES_EQUAL(0.0, robot.getX(), 0.0001);
    DOUBLES_EQUAL(0.0, robot.getY(), 0.0001);
}

// Test 2: Verarbeitet das RobotSystem einen Scan und steuert die HAL an?
TEST(RobotSystemTestGroup, ProcessingScanTriggersDriveCommand)
{
    HalMecanum hal;
    RobotSystem robot(hal, 5.0, 5.0, 0.05);

    // Wir erstellen einen einfachen Test-Scan (z.B. eine Wand in 2 Metern 
    // Entfernung)
    std::vector<LidarMessung> scan;
    scan.push_back({0.0, 2.0}); // 0 Grad, 2.0 Meter

    robot.verarbeiteScanUndFahre(scan, 0.5, 0.0, 0.0);

    // Wir erwarten, dass die HAL den Befehl für 0.5 m/s Vorwärtsfahrt empfangen hat!
    // Bei HalMecanum haben dann Vorne Links (0) und Vorne Rechts (1) 0.5 m/s
    DOUBLES_EQUAL(0.5, hal.getWheelSpeed(0), 0.0001);
} 