#pragma once
#include <cmath>
#include <vector>
#include "LidarMessung.h"
#include "MathUtils3D.h"

// Klasse für 2D-LIDAR
class LidarSensor2D
{
private: 
    double minReichweiteMeter;
    double maxReichweiteMeter;

public:
    LidarSensor2D(double minR = 0.1, double maxR = 12.0) 
        : minReichweiteMeter(minR), maxReichweiteMeter(maxR) {}

    // Prüft ob ein Messwert innerhalb der physikalischen Reichweite liegt
    bool istMessungGueltig(double distanzMeter) const
    {
        return (distanzMeter >= minReichweiteMeter && 
                distanzMeter <= maxReichweiteMeter);
    }

    // Rechnet Winkel (Grad) und Abstand (Meter) in x- und y-Koordinaten um
    // Grad muss in Rad umgewandelt werden für cos() und sin() Funktionen
    void polarZuKartesisch(double winkelGrad, double distanzMeter, 
        double& x, double& y) const
    {
        double rad = gradZuRad(winkelGrad);
        
        x = distanzMeter * std::cos(rad);
        y = distanzMeter * std::sin(rad);
    }

    // Filtert alle fehlerhaften Messungen aus einem kompletten Scan heraus
    std::vector<LidarMessung> filtereScan(const 
        std::vector<LidarMessung>& roherScan) const
    {
        std::vector<LidarMessung> ergebnis;
        for (const auto& messung : roherScan)
        {
            if (istMessungGueltig(messung.distanzMeter))
            {
                ergebnis.push_back(messung);
            }
        }
        return ergebnis;
    }
    
    // Rechnet einen kompletten 2D-Scan in eine 3D-Punktwolke um!
    // Für einen flachen 2D-LIDAR ist Z lokal immer 0.0m!
    std::vector<Point3D> scanZuPunktwolke3D(const std::vector<LidarMessung>& scan) const
    {
        std::vector<Point3D> punktwolke;
        for (const auto& messung : scan)
        {
            if (istMessungGueltig(messung.distanzMeter))
            {
                double rad = gradZuRad(messung.winkelGrad);

                Point3D punkt;
                punkt.x = messung.distanzMeter * std::cos(rad); 
                punkt.y = messung.distanzMeter * std::sin(rad);
                punkt.z = 0.0;  // Flacher 2D-Scan -> Höhe ist 0!

                punktwolke.push_back(punkt);
            }
        }
        return punktwolke;
    }
};