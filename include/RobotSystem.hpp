#pragma once
#include "Bug3Controller.hpp"
#include "IHalDrive.hpp"
#include "PT2Regler.hpp"
#include "SlamController2D.h"


/**
 * @brief Die Haupt-Steuerung des Roboters
 * Verbindet die Hardware-Abstraktion (IHalDrive) mit dem 2D-SLAM
 */
class RobotSystem {
private:
    IHalDrive& hal;  // Fahrwerk (z.B. HalMecanum)
    SlamController2D slam;  // SLAM-Kartierung
    PT2Regler pt2Vx;    // S-Kurvenglättung für Vorwärtsgeschwindigkeit
    PT2Regler pt2Vy;    // S-Kurvenglättung für Seitwärtsgeschwindigkeit
    PT2Regler pt2Omega; // S-Kurvenglättung für Drehbewegung
    Bug3Controller bug; // Autonomer Bug3-Navigator mit adaptiver Wandfolge

public:
    /**
     * @brief Konstruktor für das Robotersystem
     * @param driveHardware Universal-Schnittstelle zum Fahrwerk
     * @param kartenBreite  Breite der Belegungskarte in Metern
     * @param kartenHoehe   Höhe der Belegungskarte in Metern
     * @param aufloesung    Grid-Auflösung in Metern pro Zelle
     */
    RobotSystem(IHalDrive& drivehardware, double kartenBreite, 
        double kartenHoehe, double aufloesung) 
        : hal(drivehardware), 
          slam(kartenBreite, kartenHoehe, aufloesung), 
          pt2Vx(0.0), 
          pt2Vy(0.0), 
          pt2Omega(0.0),
          bug(0.5)    // Sicherheitsabstand 0.5m
    {}

    // Positions-Abfragen vom SLAM-Controller durchreichen
    double getX() const { return slam.getX(); }
    double getY() const { return slam.getY(); }
    double getTheta() const { return slam.getTheta(); }

    // Nimmt den LIDAR-Scan entgegen, aktualisiert SLAM und steuert das Fahrwerk 
    // inklusive PT2-Filter
    void verarbeiteScanUndFahre(const std::vector<LidarMessung>& scan, 
        double vx, double vy, double omega, double dt = 0.05);

    // Vollautonomische Zielanfahrt mit geschmeidigem Bug3-Ausweichmanöver
    void fahreZumZiel(double goalX, double goalY,
        const std::vector<LidarMessung>& scan,
        double dt = 0.05);
};