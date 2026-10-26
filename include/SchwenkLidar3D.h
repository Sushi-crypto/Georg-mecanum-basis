#pragma once
#include <cmath>
#include <vector>
#include "MathUtils3D.h"
#include "LidarMessung.h"
#include "LidarSensor2D.h"

class SchwenkLidar3D
{
private:
    LidarSensor2D basisLidar;

public: 
    // Konstruktor: Nutzt intern die Reichweitenprüfung unseres 2D-LIDARs
    SchwenkLidar3D(double minR = 0.1, double maxR = 12.0) : basisLidar(minR, maxR) {}

    // Wandert einen 2D-Scan zusammen mit dem aktuellen 
    // Servo-Neigungswinkel (Pitch in Grad)in eine echte 3D-Punktwolke um!
    std::vector<Point3D> scanZu3DPunktwolke(const std::vector<LidarMessung>& scan2D,
        double servoPitchGrad) const
    {
        std::vector<Point3D> punktwolke3D;

        // Neigungswinkel des Servomotors in Bogenmaß umrechnen
        double radServo = gradZuRad(servoPitchGrad);
        double cosServo = std::cos(radServo);
        double sinServo = std::sin(radServo);

        for (const auto& messung : scan2D)
        {
            if (basisLidar.istMessungGueltig(messung.distanzMeter))
            {
                // Laserwinkel des rotierenden Kopfes in Bogenmaß umrechnen
                double radLaser = gradZuRad(messung.winkelGrad);
                Point3D p;

                //Kugelkoordinaten-Transformation in kartesische 3D-Weltpunkte (X, Y, Z)
                p.x = messung.distanzMeter * cosServo * std::cos(radLaser);
                p.y = messung.distanzMeter * cosServo * std::sin(radLaser);
                p.z = messung.distanzMeter * sinServo; // Höhe Z entsteht durch den Servo!

                punktwolke3D.push_back(p);
            }
        }

        return punktwolke3D;
        
    }
};