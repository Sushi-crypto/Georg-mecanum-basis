#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include "LidarMessung.h"
#include "LidarSensor2D.h"
#include "Verschiebung3D.h"

// Klasse für den Scan Matcher im 3D-Raum
class ScanMatcher3D
{
private:
    LidarSensor2D sensorHelper;

public:
    // Berechnet die Abweichung im 3D-Raum bei gegebener Roboterhöhe Z
    double berechneAbweichung3D(const std::vector<LidarMessung>& scanA,
        const std::vector<LidarMessung>& scanB,
        double dx, double dy, double dz, 
        double robZ_A, double robZ_B) const
    {
        if (scanA.size() != scanB.size()) return 999.0;

        double gesamtFehler = 0.0;

        for (size_t i=0; i < scanA.size(); ++i)
        {
            double ax = 0.0, ay = 0.0;
            double bx = 0.0, by = 0.0;

            sensorHelper.polarZuKartesisch(scanA[i].winkelGrad, 
                scanA[i].distanzMeter, ax, ay);
                
            sensorHelper.polarZuKartesisch(scanB[i].winkelGrad, 
                scanB[i].distanzMeter, bx, by);

            // 3D-Koordinaten inklusive Z-Höhe
            double az = robZ_A;
            double bz = robZ_B + dz;

            double diffX = ax - (bx + dx);
            double diffY = ay - (by + dy);
            double diffZ = az - bz;

            gesamtFehler += (diffX * diffX + diffY * diffY + diffZ * diffZ);
        }

        return gesamtFehler;
    }
    
    // Raster-Suche über X, Y und Z
    void findeBesteVerschiebung3D(const std::vector<LidarMessung>& scanA,
        const std::vector<LidarMessung>& scanB, double robZ_A, double robZ_B, 
        double minDx, double maxDx, double minDy, double maxDy, double minDz, 
        double maxDz, double schritt, double& bestesDx, double& bestesDy, double& bestesDz) const
    {
        double besterFehler = std::numeric_limits<double>::max();
        bestesDx = minDx;
        bestesDy = minDy;
        bestesDz = minDz;

        for (double testDx = minDx; testDx <= maxDx + 0.001; testDx += schritt)
        {
            for (double testDy = minDy; testDy <= maxDy + 0.001; testDy += schritt)
            {
                for (double testDz = minDz; testDz <= maxDz + 0.001; testDz += schritt)
                {
                    double fehler = berechneAbweichung3D(scanA, scanB, testDx, testDy, testDz, 
                            robZ_A, robZ_B);

                    if (fehler < besterFehler)
                    {
                        besterFehler = fehler;
                        bestesDx = testDx;
                        bestesDy = testDy;
                        bestesDz = testDz;
                    }
                }
            }
        }
    }
};