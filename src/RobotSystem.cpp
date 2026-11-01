#include "RobotSystem.hpp"

void RobotSystem::verarbeiteScanUndFahre(const 
    std::vector<LidarMessung>& scan, double vx, double vy, double omega, double dt)
{
    // 1. SLAM-Controller verarbeitet den LIDAR-Scan und zeichnet die Karte
    slam.verarbeiteScan(scan);

    // 2. PT2-Filter glättet die Wunsch-Geschwindigkeit vx sanft in einer S-Kurve
    double vxGeglaetet = pt2Vx.update(vx, dt);

    // 3. Befehl direkt an die HAL durchreichen
    hal.drive(vxGeglaetet, vy, omega);
}