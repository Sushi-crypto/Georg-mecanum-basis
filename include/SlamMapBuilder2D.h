#pragma once
#include "LidarMessung.h"
#include <cmath>
#include <vector>

// Klasse für den 2D-Slam Bauer
class SlamMapBuilder2D
{
private:
    OccupancyGrid karte;
    LidarSensor2D sensor;
    RayCaster2D caster;
    double aufloesungMeterProZelle;

public:
    SlamMapBuilder2D(int breite, int hoehe, double aufloesung = 0.05, 
        double minR = 0.1, double maxR = 12.0) : karte(breite, hoehe), sensor(minR, maxR),
        aufloesungMeterProZelle(aufloesung) {}

    // Nimmt einen rohen Scan entgegen, filtert ihn und zeichnet ihn in die 2D-Karte
    void verarbeiteScan(double robX, double robY, const std::vector<LidarMessung>& roherScan)
    {
        // 1. Ungültige Messungen (zu nah / zu weit) herausfiltern
        std::vector<LidarMessung> bereinigterScan = sensor.filtereScan(roherScan);

        // 2. Mit dem 2D-RayCaster direkt in das OccupancyGrid zeichnen
        caster.zeichneRundumScan(karte, robX, robY, bereinigterScan, 
            aufloesungMeterProZelle);
    }

    const OccupancyGrid& getKarte() const { return karte; }
    OccupancyGrid& getKarte() { return karte; }
};