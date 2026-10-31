#pragma once

#include "IHalDrive.hpp"
#include "SlamController2D.h"

/**
 * @brief Die Haupt-Steuerung des Roboters
 * Verbindet die Hardware-Abstraktion (IHalDrive) mit dem 2D-SLAM
 */
class RobotSystem {
private:
    IHalDrive& hal;  // Fahrwerk (z.B. HalMecanum)
    SlamController2D slam;  // SLAM-Kartierung

public:
    /**
     * @brief Konstruktor für das Robotersystem
     * @param driveHardware Universal-Schnittstelle zum Fahrwerk
     * @param kartenBreite  Breite der Belegungskarte in Metern
     * @param kartenHoehe   Höhe der Belegungskarte in Metern
     * @param aufloesung    Grid-Auflösung in Metern pro Zelle
     */
    RobotSystem(IHalDrive& drivehardware, double kartenBreite, 
        double kartenHoehe, double aufloesung) : hal(drivehardware), 
        slam(kartenBreite, kartenHoehe, aufloesung)
    {
    }

    // Nimmt den LIDAR-Scan entgegen, aktualisiert SLAM und steuert das Fahrwerk
    void verarbeiteScanUndFahre(const std::vector<LidarMessung>& scan, double vx, 
        double vy, double omega);

    // Positions-Abfragen vom SLAM-Controller durchreichen
    double getX() const { return slam.getX(); }
    double getY() const { return slam.getY(); }
    double getTheta() const { return slam.getTheta(); }
};