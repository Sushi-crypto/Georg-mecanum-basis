#pragma once
#include "LidarMessung.h"
#include <cmath>
#include <vector>

// Klasse für SLAM-Bauer im 3D-Raum
class SlamMapBuilder3D
{
private:
    OccupancyGrid3D karte3D;
    LidarSensor2D sensor;
    RayCaster3D caster3D;

public: 
    SlamMapBuilder3D(double breiteMeter, double hoheMeter, double tiefeMeter, 
        double aufloesung = 1.0, double minR = 0.1, double maxR = 12.0) : karte3D(breiteMeter,
            hoheMeter, tiefeMeter, aufloesung), sensor(minR, maxR) {}

    // Nimmt einen rohen 2D-Scan entgegen, filtert ihn und zeichnet ihn auf Roboterhöhe 
    // (z) in die 3D-Karte
    void verarbeiteScan(double robX, double robY, double robZ, 
        const std::vector<LidarMessung>& roherScan)
    {
        // 1. Ungültige Messungen herausfiltern
        std::vector<LidarMessung> bereinigterScan = sensor.filtereScan(roherScan);

        // 2. Mit dem 3D-Raycaster in die räumliche 3D-Karte zeichnen
        caster3D.zeichneRundumScan(karte3D, robX, robY, robZ, bereinigterScan);
    }

    const OccupancyGrid3D& getKarte3D() const { return karte3D; }
    OccupancyGrid3D& getKarte3D() { return karte3D; }
};