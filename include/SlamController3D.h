#pragma once
#include <cmath>
#include <vector>
#include "OccupancyGrid3D.h"
#include "MathUtils3D.h"

class SlamController3D
{
private:
    OccupancyGrid3D karte;

    // Die 6D-Pose des Roboters im dreidimensionalen Raum
    double roboterX;
    double roboterY;
    double roboterZ;
    double roboterRoll;   // Neigung um X-Achse
    double roboterPitch;  // Neigung um Y-Achse
    double roboterYaw;    // Neigung um Z-Achse

    bool hatErstenScan;

public:
    // Konstruktor: Erstellt Raumgröße (Breite, Länge, Höhe in Metern)
    // & Auflösung
    SlamController3D(double breiteMeter, double hoeheMeter, double tiefeMeter, double aufloesungMeter) 
            : karte(breiteMeter, hoeheMeter, tiefeMeter, aufloesungMeter),
                roboterX(0.0), roboterY(0.0), roboterZ(0.0),
                    roboterRoll(0.0), roboterPitch(0.0), roboterYaw(0.0), hatErstenScan(false)
    {
    }

    // Nimmt eine 3D-Punktwolke entgegen und trägt die Hindernisse in das Voxel-Grid ein!
    void verarbeitePunktwolke(const std::vector<Point3D>& punktwolke)
    {
        for (const auto& punkt : punktwolke)
        {
            // 1. Lokalen Sensorpunkt mit der aktuellen 6D-Roboterpose in Weltkoordinaten umrechnen
            Point3D weltPunkt = transformierePunkt3D(punkt, 
                roboterX, roboterY, roboterZ, 
                roboterRoll, roboterPitch, roboterYaw);

            // 2. Hindernis-punkt in die 3D-Voxel-Karte eintragen (100.0 = Belegt)
            karte.setzeZelle(weltPunkt.x, weltPunkt.y, weltPunkt.z, 100.0);
        }

        hatErstenScan = true;
    }

    // Verarbeitet die Punktwolke in der echten Roboter Fahrt
    void setzePose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        roboterX = x;
        roboterY = y;
        roboterZ = z;
        roboterRoll = roll;
        roboterPitch = pitch;
        roboterYaw = yaw;
    }

    // Getter für die 6D-Pose
    double getX() const { return roboterX; }
    double getY() const { return roboterY; }
    double getZ() const { return roboterZ; }
    double getRoll() const { return roboterRoll; }
    double getPitch() const { return roboterPitch; }
    double getYaw() const { return roboterYaw; }

    // Zugriff auf die 3D-Karte (z.B. für Tests oder Visualisierung)
    const OccupancyGrid3D& getKarte() const { return karte; }
};