#include "CppUTest/TestHarness.h"
#include "LidarMessung.h"
#include "HalMecanum.hpp"
#include "RobotSystem.hpp"
#include "PT2Regler.hpp"
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

    // Wir fordern 0.5 m/s Vorwärtsfahrt an
    robot.verarbeiteScanUndFahre(scan, 0.5, 0.0, 0.0);

    // Durch den PT2-Filter rollt der Roboter im ersten Zeitschritt mit 
    // eimer_1 = 0 + (0,5 - 0)*(0,05/0,2) = 0,125
    // eimer_2 = 0 + (0,125 - 0)*(0,05/0,2) = 0,03125
    DOUBLES_EQUAL(0.03125, hal.getWheelSpeed(0), 0.0001);
} 

// Test 3: Prüft, ob Fahrbefehle im RobotSystem durch den PT2Regler sanft geglättet werden!
TEST(RobotSystemTestGroup, SmoothsVelocityUsingPT2Filter)
{
    HalMecanum hal;
    RobotSystem robot(hal, 5.0, 5.0, 0.05);

    std::vector<LidarMessung> scan = {{0.0, 5.0}};
    double dt = 0.05; // 50 Millisekunden Zeitfenster

    // Wir fordern schlagartig Vorwärtsfahrt mit 0.5 m/s 
    robot.verarbeiteScanUndFahre(scan, 0.5, 0.0, 0.0, dt);

    // Dank des PT2-Filters darf die Rad-Geschwindigkeit bei t = 0.05s NOCH NICHT sofort 
    // auf 0.5 m/s stehen!
    // Sie muss sanft darunter liegen (z.B. kleiner als 0.4 m/s, aber größer als 0.0 m/s):
    double radGeschwindigkeit = hal.getWheelSpeed(0);
    CHECK(radGeschwindigkeit < 0.4);
    CHECK(radGeschwindigkeit > 0.0);
}