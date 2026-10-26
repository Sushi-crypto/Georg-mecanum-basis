#pragma once
#include <cmath>
#include <vector>
#include "LidarMessung.h"
#include "MathUtils3D.h"

// Klasse für 3D-LIDAR
class RayCaster3D
{
public:
    void zeichneStrahl(OccupancyGrid3D& karte3D,
                        double x1, double y1, double z1,
                        double x2, double y2, double z2)
    {
        // 1. Richtungsvektor & Gesamtdistanz berechnen
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double distanz = std::sqrt(dx * dx + dy * dy + dz * dz);

        if(distanz == 0.0) return;

        // 2. in kleinen Schritten (z.B. 0.5 Meter) am Strahl entlangwandern
        double schrittweite = 0.5;
        int anzahlSchritte = static_cast<int>(distanz / schrittweite);

        // 3. Weg VOR dem Hindernis als FREI (0.0) markieren
        for(int i = 0; i < anzahlSchritte; i++)
        {
            double t = (i * schrittweite) / distanz;
            double currX = x1 + t * dx;
            double currY = y1 + t * dy;
            double currZ = z1 + t * dz;

            karte3D.setzeZelle(currX, currY, currZ, 0.0);
        }

        // 4. Den eigentlichen Trefferpunkt am Ende als BELEGT (1.0) markieren
        karte3D.setzeZelle(x2, y2, z2, 1.0);
    
    }
    
    // Die Methode für den Rundumscan 
    void zeichneRundumScan(OccupancyGrid3D& karte3D,
                                double robX, double robY, double robZ, 
                                const std::vector<LidarMessung>&scan)
    {

        for (const auto& messung : scan)
        {
             // Grad in Bogenmaß umrechnen
            double rad = gradZuRad(messung.winkelGrad);

            // Zielpunkt des Laserstrahls mit Winkelfunktionen berechnen
            double zielX = robX + messung.distanzMeter * std::cos(rad);
            double zielY = robY + messung.distanzMeter * std::sin(rad);
            double zielZ = robZ; // Laser rotiert waagerecht auf Roboterhöhe

            // Ab durch die Karte mit den bewährten Laserstrahl-Funktion
            zeichneStrahl(karte3D, robX, robY, robZ, zielX, zielY, zielZ);
        }
    }
};