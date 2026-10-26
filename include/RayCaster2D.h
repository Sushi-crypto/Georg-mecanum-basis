#pragma once
#include <cmath>
#include <vector>
#include "LidarMessung.h"
#include "MathUtils3D.h"

// Klasse für 2D-Laserstrahlenschreiber
class RayCaster2D
{
public:
    void zeichneStrahl(OccupancyGrid& karte,
                        double x1, double y1, 
                        double x2, double y2,
                        double aufloesungProZelle = 0.05)
    {
        // 1. Richtungsvektor & Gesamtdistanz berechnen
        double dx = x2 - x1;
        double dy = y2 - y1;
        double distanz = std::sqrt(dx * dx + dy * dy);

        if(distanz == 0.0) return;

        // 2. in kleinen Schritten (z.B. 0.5 Meter) am Strahl entlangwandern
        double schrittweite = aufloesungProZelle*0.5;
        int anzahlSchritte = static_cast<int>(distanz / schrittweite);

        // 3. Weg VOR dem Hindernis als FREI (0.0) markieren
        for(int i = 0; i < anzahlSchritte; i++)
        {
            double t = (i * schrittweite) / distanz;
            double currX = x1 + t * dx;
            double currY = y1 + t * dy;

            // Umrechnung von Meter in Raster-Indizes
            int zelleX = static_cast<int>(std::round(currX / aufloesungProZelle));
            int zelleY = static_cast<int>(std::round(currY / aufloesungProZelle));
            
            // Strahl in die 2D-Karte einzeichnen
            karte.setzeZelle(zelleX, zelleY, 0); // 0 = FREI
        }

        // 4. Den eigentlichen Trefferpunkt am Ende als BELEGT (100) markieren
        int zielZelleX = static_cast<int>(std::round(x2 / aufloesungProZelle));
        int zielZelleY = static_cast<int>(std::round(y2 / aufloesungProZelle));
        karte.setzeZelle(zielZelleX, zielZelleY, 100); // 100 = belegt
    
    }
    
    // Die Methode für den Rundumscan 
    void zeichneRundumScan(OccupancyGrid& karte,
                                double robX, double robY,  
                                const std::vector<LidarMessung>&scan, 
                                double aufloesungMeterProZelle = 0.05)
    {

        for (const auto& messung : scan)
        {
             // Grad in Bogenmaß umrechnen
            double rad = gradZuRad(messung.winkelGrad);

            // Zielpunkt des Laserstrahls mit Winkelfunktionen berechnen
            double zielX = robX + messung.distanzMeter * std::cos(rad);
            double zielY = robY + messung.distanzMeter * std::sin(rad);
            
            // Ab durch die Karte mit den bewährten Laserstrahl-Funktion
            zeichneStrahl(karte, robX, robY, zielX, zielY, aufloesungMeterProZelle);
        }
    }
};