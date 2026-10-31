#include "RobotSystem.hpp"

void RobotSystem::verarbeiteScanUndFahre(const 
    std::vector<LidarMessung>& scan, double vx, double vy, double omega)
{
    // SLAM-Controller verarbeitet den LIDAR-Scan und zeichnet die Karte
    slam.verarbeiteScan(scan);

    // Befehl direkt an die HAL durchreichen
    hal.drive(vx, vy, omega);
}