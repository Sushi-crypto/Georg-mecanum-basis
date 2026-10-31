#include "CppUTest/TestHarness.h"
#include "Bug3Controller.hpp"
#include "LidarMessung.h"
#include <vector>

// Testgruppe für den Bug3-Algorithmus
TEST_GROUP(Bug3TestGroup)
{
    void setup() {}
    void teardown() {}
};

// Test 1: Roboter fährt ohne Hindernis direkt auf das Ziel zu
TEST(Bug3TestGroup, DrivesStraightTowardsGoalWhenPathClear)
{
    Bug3Controller bug;

    // Roboter steht auf (0,0) und das Ziel ist bei (2.0, 0.0) 
    // -> Reine Vorwärtsfahrt
    double roboterX = 0.0;
    double roboterY = 0.0;
    double roboterTheta = 0.0;

    double zielX = 2.0;
    double zielY = 0.0;

    // Kein Hindernis im Weg (LIDAR misst überall 5.0 Meter Freiraum)
    std::vector<LidarMessung> freierScan = {{0.0, 5.0}};

    // Berechne Geschwindigkeits-Befehl (vx, vy, omega)
    Bug3Output cmd = bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, freierScan);

    // Wir erwarten reine Vorwärtsfahrt (vx > 0, vy = 0, omega = 0)
    DOUBLES_EQUAL(0.5, cmd.vx, 0.0001);
    DOUBLES_EQUAL(0.0, cmd.vy, 0.0001);
    DOUBLES_EQUAL(0.0, cmd.omega, 0.0001);

    // Status muss GOAL_SEEKING sein!
    CHECK_EQUAL(static_cast<int>(BugState::GOAL_SEEKING), static_cast<int>(cmd.state));
}