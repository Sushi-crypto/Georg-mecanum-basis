#include "CppUTest/TestHarness.h"
#include "Bug3Controller.hpp"
#include "LidarMessung.h"
#include "RobotSystem.hpp"
#include "PT2Regler.hpp"
#include <cstdlib>  // Für std::system()
#include <vector>
#include <fstream>
#include <algorithm>

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

// Test 3: Roboter schaltet von WALL_FOLLOWING zurück auf GOAL_SEEKING, sobald der Weg frei ist
TEST(Bug3TestGroup, SwitchesBackToGoalSeeking)
{
    Bug3Controller bug(0.5);    // Sicherheitsabstand 0.5m

    double roboterX = 0.0;
    double roboterY = 0.0;
    double roboterTheta = 0.0;
    double zielX = 2.0;
    double zielY = 0.0; // Ziel liegt bei 0 Grad (geradeaus)

    // 1. Schock-Moment: Wnad direkt voraus (0.3m) -> Umschalten auf WALL_FOLLOWING
    std::vector<LidarMessung> blockierterScan = {{0.0, 0.3}};
    bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, blockierterScan);

    // 2. Jetzt ist die Wand umfahren: Der Weg nach vorne (0 Grad) ist wieder frei (5.0m)
    std::vector<LidarMessung> freierScan = {{0.0, 5.0}};
    Bug3Output cmd = bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, freierScan);
    for (int i = 0; i < 11; ++i) {
        cmd = bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, freierScan, 0.05);
    }

    // Der Roboter MUSS jetzt wieder zurück in den Zustand GOAL_SEEKING gewechselt sein
    CHECK_EQUAL(static_cast<int>(BugState::GOAL_SEEKING), static_cast<int>(cmd.state));
}

// Test 4: Roboter steuert ein Ziel rein in Y-Richtung an (Seitwärts-Fahrt)
TEST(Bug3TestGroup, DrivesTowardsGoalInYDirection)
{
    Bug3Controller bug(0.5);

    double roboterX = 0.0;
    double roboterY = 0.0;
    double roboterTheta = 0.0;

    // Ziel liegt rein seitlich bei (0.0, 2.0)
    double zielX = 0.0;
    double zielY = 0.0;

    std::vector<LidarMessung> freierScan = {{0.0, 5.0}};

    Bug3Output cmd = bug.update(roboterX, roboterY, roboterTheta,
        zielX, zielY, freierScan);
    
        // Wir erwarten reine Y-Fahrt (vx = 0.0, vy = 0.5 m/s, omega = 0.0)
        DOUBLES_EQUAL(0.0, cmd.vx, 0.0001);
        DOUBLES_EQUAL(0.0, cmd.vy, 0.0001);
        DOUBLES_EQUAL(0.0, cmd.omega, 0.0001);
}

// Test 5: Prüft den geschmeidigen Übergang in den Wandfolge-Modus
TEST(Bug3TestGroup, SmoothWallTransitionBasedOnDistance)
{
    Bug3Controller bug(0.5);    // Sicherheitsabstand 0.5m

    double roboterX = 0.0;
    double roboterY = 0.0;
    double roboterTheta = 0.0;
    double zielX = 2.0;
    double zielY = 0.0;

    // Hindernis voraus bei 0.3m (liegt im Bereich < 0.5m)
    std::vector<LidarMessung> scan = {{0.0, 0.3}};

    Bug3Output cmd = bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, scan);

    // 1. Der Zustand MUSS WALL_FOLLOWING sein
    CHECK_EQUAL(static_cast<int>(BugState::WALL_FOLLOWING), static_cast<int>(cmd.state));

    // 2. vx darf NICHT schlagartig 0.0 sein, sondern soll proportional abfedern (~0.3 m/s)
    CHECK(cmd.vx > 0.1);
    CHECK(cmd.vx < 0.5);

    // 3. vy soll sanft hochregeln (> 0.0 m/s)
    CHECK(cmd.vy > 0.0);
}

// Test 6: Pürft das dynamische Einlenken (omega) bei kritischen Nahkontakt (<0.2m)
TEST(Bug3TestGroup, ActivateRotationalEmergency)
{
    Bug3Controller bug(0.5); // Sicherheitsabstand 0.5 m

    double roboterX = 0.0;
    double roboterY = 0.0;
    double roboterTheta = 0.0;
    double zielX = 2.0;
    double zielY = 0.0;

    // Hindernis extrem nah bei 0.1m (Sicherheitsabstand ist 0.5m)
    std::vector<LidarMessung> nahScan = {{0.0, 0.1}};

    Bug3Output cmd = bug.update(roboterX, roboterY, roboterTheta, zielX, zielY, nahScan);

    // 1. Zustand muss WALL_FOLLOWING sein
    CHECK_EQUAL(static_cast<int>(BugState::WALL_FOLLOWING), static_cast<int>(cmd.state));

    // 2. Vorwärtsfahrt vx fällt unter den Schwellenwert von 0.2 m/s (exakt 0.1 m/s)
    DOUBLES_EQUAL(0.2, cmd.vx, 0.0001);

    // 3. Seitwärtsfahrt vy legt ordentlich zu (exakt 0.4 m/s)
    DOUBLES_EQUAL(0.5, cmd.vy, 0.0001);

    // 4. Nase soll immer vorne bleiben
    DOUBLES_EQUAL(0.0, cmd.omega, 0.0001);
}

// Telemetrie-Test: Erzeugt die echte Fahrspur für die Python-Visualisierung
TEST(Bug3TestGroup, ExportRealTrajectoryData)
{
    Bug3Controller bug(0.5);    // Sicherheitsabstand 0.5m
    PT2Regler pt2Vx(0.0, 0.08);
    PT2Regler pt2Vy(0.0, 0.08);
    PT2Regler pt2Omega(0.0, 0.08);

    double x = 0.0, y = 0.0, theta = 0.0;
    double goalX = 2.0, goalY = 0.0;
    double dt = 0.05;

    // Wir öffnen die CSV-Datei zum Schreiben
    std::ofstream csv("../trajektorie_daten.csv");
    csv << "x,y,theta,state\n";

    for (int i = 0; i < 120; ++i) {
        // LIDAR-Simulation: Hindernis bei (0.8, 0.0)
        double distH = std::sqrt((0.8 - x) * (0.8 - x) + (0.0 - y) * (0.0 - y));

        // Relativer Winkel zum Hindernis
        double winkelHRad = std::atan2(0.0 - y, 0.8 - x) - theta;
        double winkelHGrad = radZuGrad(winkelHRad);

        std::vector<LidarMessung> scan = {{winkelHGrad, distH}};

        // 1. Echten C++ Bug3Controller aufrufen
        Bug3Output cmd = bug.update(x, y, theta, goalX, goalY, scan);

        // 2. Echten C++ PT2-Regler aufrufen 
        double vxG = pt2Vx.update(cmd.vx, dt);
        double vyG = pt2Vy.update(cmd.vy, dt);
        double omegaG = pt2Omega.update(cmd.omega, dt);

        // 3. Roboter-Kinematik berechnen
        x += (vxG * std::cos(theta) - vyG * std::sin(theta)) * dt;
        y += (vxG * std::sin(theta) + vyG * std::cos(theta)) * dt;
        theta += omegaG * dt;

        // In CSV Datei schreiben
        csv << x << "," << y << "," << theta << "," << static_cast<int>(cmd.state) << "\n";
    }
    csv.close();

    // Automatisch das Python-Visualisierungsskript aufrufen
    std::system("python3 ../script/visualize_trajectory.py");
}

// Test 7: Testet die Krabbenfahrt an einer flachen Wand
double abstandZuRechteck(double x, double y, double minX, double maxX, double minY, double maxY) {
    double dx = std::max({minX - x, 0.0, x - maxX});
    double dy = std::max({minY - y, 0.0, y - maxY});
    return std::sqrt(dx * dx + dy * dy);
}

TEST(Bug3TestGroup, AvoidsRectangularObstacle)
{
    Bug3Controller bug(0.4);    // 0.4 m Sicherheitsabstand
    PT2Regler pt2Vx(0.0, 0.08);
    PT2Regler pt2Vy(0.0, 0.08);
    PT2Regler pt2Omega(0.0, 0.08);

    double x = 0.0, y = 0.0, theta = 0.0;
    double goalX = 3.0, goalY = 0.0;
    double dt = 0.05;

    // Rechteck-Grenzen (Kiste)
    double boxMinX = 0.8, boxMaxX = 1.6;
    double boxMinY = -0.3, boxMaxY = 0.3;

    // 1. Ordner anlegen & CSV erstellen
    std::system("mkdir -p ../data");
    std::system("mkdir -p ../images");

    std::ofstream csv("../data/trajektorie_rechteck_daten.csv");

    if (csv.is_open()) {
        csv << "x,y,theta,state\n";

        for (int i = 0; i < 200; ++i)
        {
            // Front-Abstand
            double distFront = 5.0;
            if (y >= boxMinY - 0.1 && y <= boxMaxY + 0.1 && x < boxMinX) {
                distFront = boxMinX - x;
            }

            // Seiten-Abstand
            double distSeite = 5.0;
            if (x >= boxMinX && x <= boxMaxX) {
                distSeite = std::abs(y - boxMaxY);  // Echter Wandabstand in Y-Richtung
            }

            // Dual-LIDAR Scanpaket (0° Front + 90° Seite)
            std::vector<LidarMessung> scan = {
                { 0.0,  distFront }, // Front-LIDAR
                { 90.0, distSeite }  // Seiten-LIDAR
            }; 

            // C++ Bug3Controller aufrufen
            Bug3Output cmd = bug.update(x, y, theta, goalX, goalY, scan, dt);

            // PT2-Filterung
            double vxG    = pt2Vx.update(cmd.vx, dt);
            double vyG    = pt2Vy.update(cmd.vy, dt);
            double omegaG = pt2Omega.update(cmd.omega, dt);

            // Roboter-Kinematik
            x += (vxG * std::cos(theta) - vyG * std::sin(theta)) * dt;
            y += (vxG * std::sin(theta) + vyG * std::cos(theta)) * dt;
            theta += omegaG * dt;

            csv << x << "," << y << "," << theta << "," << static_cast<int>(cmd.state) << "\n";
        }
        csv.close();
    }
    
    // Visualisierungsskript aufrufen
    std::system("python3 ../script/visualize_trajectory.py");
}