#include "RobotSystem.hpp"
#include "LidarUtils.hpp"

void RobotSystem::verarbeiteScanUndFahre(const std::vector<LidarMessung>& scan, 
    double vx, double vy, double omega, double dt)
{
    // 1. SLAM-Controller verarbeitet den LIDAR-Scan und zeichnet die Karte
    slam.verarbeiteScan(scan);

    // 2. PT2-Filter glättet die Wunsch-Geschwindigkeit vx sanft in einer S-Kurve
    double vxGeglaettet = pt2Vx.update(vx, dt);
    double vyGeglaettet = pt2Vy.update(vy, dt);
    double omegaGeglaettet = pt2Omega.update(omega, dt);

    // 3. Befehl direkt an die HAL durchreichen
    hal.drive(vxGeglaettet, vyGeglaettet, omegaGeglaettet);
}

void RobotSystem::fahreZumZiel(double goalX, double goalY, 
    const std::vector<LidarMessung>& scan, double dt)
{
    // 1. SLAM verarbeitet den LIDAR-Scan und korrigiert die Roboterposition
    slam.verarbeiteScan(scan);

    // 2. Bug3-Controller berechnet vollautomatisch die Ausweich- und Navigationsentscheidung
    // (Inklusive dynamischer Elchtest-Abbremsung & adaptivem Einlenken bei Hindernissen)
    Bug3Output cmd = bug.update(getX(), getY(), getTheta(), goalX, goalY, scan);

    // 3. PT2-Filter glättet die geforderte Geschwindigkeiten
    double vxGeglaettet = pt2Vx.update(cmd.vx, dt);
    double vyGeglaettet = pt2Vy.update(cmd.vy, dt);
    double omegaGeglaettet = pt2Omega.update(cmd.omega, dt);

    // 4. Der fertige autonome Fahrbefehl geht direkt an die HAL
    hal.drive(vxGeglaettet, vyGeglaettet, omegaGeglaettet);
}

void RobotSystem::fahreZumZielDualLidar(
    double goalX, double goalY,
    const std::vector<LidarMessung>& scanLinks,
    const std::vector<LidarMessung>& scanRechts,
    double dt)
{
    // 1. Dual-LIDAR zu 270°-Panorama verschmelzen
    std::vector<LidarMessung> gesamtScan = LidarUtils::fuegeDualLidarZusammen(scanLinks, scanRechts);

    // 2. Ortung und Kartierung (SLAM) aktualisieren
    slam.verarbeiteScan(gesamtScan);

    // 3. Pfadplanung (Bug3) mit dem Panoramabild
    Bug3Output cmd = bug.update(getX(), getY(), getTheta(), goalX, goalY, gesamtScan, dt);

    // 4. Beschleunigung glätten (PT2)
    double vxGeglaettet = pt2Vx.update(cmd.vx, dt);
    double vyGeglaettet = pt2Vy.update(cmd.vy, dt);
    double omegaGeglaettet = pt2Omega.update(cmd.omega, dt);

    // 5. Motor-Befehl an Hardware senden
    hal.drive(vxGeglaettet, vyGeglaettet, omegaGeglaettet);
}

