#pragma once
#include <cmath>
#include <vector>

// Klasse für den OccupancyGrid für eine 3D-Karte
class OccupancyGrid3D
{
private:
    double breite;
    double hoehe;
    double tiefe;
    double aufloesung; // z.B. 0.1 m pro Zelle

    int anzX, anzY, anzZ; // Anzahl der internen Kästchen
    std::vector<std::vector<std::vector<double>>> raster;

    // Hilfsfunktion: Wandelt Meter-Koordinate (double) in Kästchen-Index (int) um
    int meterZuIndex(double meter) const
    {
        if (meter < 0.0) return -1;
        return static_cast<int>(meter / aufloesung); // Findet den richtigen Kästchen (0 bis 99)
    }

public:
    
    OccupancyGrid3D(double b, double h, double t, double res) : breite(b), hoehe(h), tiefe(t), aufloesung(res)
    {
       anzX = static_cast<int>(b / res);
       anzY = static_cast<int>(h / res);
       anzZ = static_cast<int>(t / res);

       // 3D-Vektor mit double-Werten aufbauen und initial mit -1.0 Unbekannt füllen
       raster = std::vector<std::vector<std::vector<double>>>(
                    anzX, std::vector<std::vector<double>>(
                        anzY, std::vector<double>(
                            anzZ, -1.0)));
    }

    // Liest den Wert einer Zelle an Postition (x, y, z) aus 
    double getZelle(double x, double y, double z) const
    {
        int ix = meterZuIndex(x);
        int iy = meterZuIndex(y);
        int iz = meterZuIndex(z);

        if (ix >= 0 && ix < anzX && iy >= 0 && iy < anzY && iz >= 0 && iz < anzZ)
        {
            return raster[ix][iy][iz];
        }

        return -1.0; // Sicherheitsnetz für Abfragen außerhalb der Karte
    }

    // Setzt den Wert einer Zelle an Position (x, y, z) 
    void setzeZelle(double x, double y, double z, double wert)
    {
        int ix = meterZuIndex(x);
        int iy = meterZuIndex(y);
        int iz = meterZuIndex(z);

        if (ix >= 0 && ix < anzX && iy >= 0 && iy < anzY && iz >= 0 && iz < anzZ)
        {
            raster[ix][iy][iz] = wert;
        }
    }

    // Getter für die Dimensionen
    int getAnzX() const { return anzX; }
    int getAnzY() const { return anzY; }
    int getAnzZ() const { return anzZ; }
    double getAufloesung() const { return aufloesung; }
};