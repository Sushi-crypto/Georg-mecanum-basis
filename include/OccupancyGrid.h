#pragma once
#include <cmath>
#include <vector>

// Klasse für den OccupacyGrid für eine 2D-Karte
class OccupancyGrid
{
private:
    int breite;
    int hoehe;
    double aufloesung;
    // Unser 2D-Raster: Ein Vektor aus  Vektoren!
    std::vector<std::vector<int>> raster;

public:
    // Der Konstruktor: Baut das Gitter und füllt alle Zellen mit -1 (Unbekannt)
    OccupancyGrid(int b, int h, double a = 1.0) : breite(b), hoehe(h), aufloesung(a)
    {
        raster = std::vector<std::vector<int>>(breite, std::vector<int>(hoehe, -1));
    }

    // Wandelt Weltmeter (double) in Rasterkästchen (int) um!
    int weltZuGridX(double xMeter) const
    {
        // std::round rundet 2.99999999 sauber auf 3 auf!
        return static_cast<int>(std::round(xMeter / aufloesung));
    }

    int weltZuGridY(double yMeter) const
    {
        return static_cast<int>(std::round(yMeter / aufloesung));
    }

    // Liest den Wert einer Zelle an Postition (x, y) aus 
    int getZelle(int x, int y) const
    {
        if (x >= 0 && x < breite && y >= 0 && y < hoehe)
        {
            return raster[x][y];
        }

        // Sicherheitsnetz für Abfragen außerhalb der Karte
        return -1;
    }

    // Setzt den Wert einer Zelle an Position (x, y) 
    void setzeZelle(int x, int y, int wert)
    {
        if (x >= 0 && x < breite && y >= 0 && y < hoehe)
        {
            raster[x][y] = wert;
        }
    }

    // Überladung für Kommazahlen (double in Metern) mit Rundung
    int getZelle(double xMeter, double yMeter) const
    {
        int gx = weltZuGridX(xMeter);
        int gy = weltZuGridY(yMeter);
        return getZelle(gx, gy); // Ruft die Int-Methode mit den gerundeten Indizen auf
    }

    void setzeZelle(double xMeter, double yMeter, int wert)
    {
        int gx = weltZuGridX(xMeter);
        int gy = weltZuGridY(yMeter);
        setzeZelle(gx, gy, wert);
    }
};