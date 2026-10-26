#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include "LidarMessung.h"
#include "LidarSensor2D.h"
#include "MathUtils3D.h"


// Klasse für den 2D-Scanmatcher
// Vergleicht zwei LIDAR-Scans miteinander, um die genaue Positionsverschiebung 
// (Schlupf/Drift) des Roboters zu ermitteln

class ScanMatcher2D
{
private:
    // Hilfsklasse zur Umrechnung von Polarkoordinaten in (x,y)
    LidarSensor2D sensorHelper;
    
public: 
    // Berechnet die quadratische Abweichung zwischen zwei Scans bei einer vermuteten Verschiebung (dx, dy)

    // Abschnitt 1: Fehlerberechnung für eine vermutete Verschiebung
    // Berechnet den Gesamtabweichungsfehler (quadratischer Fehler) zwischen
    // Scan A (Referenz) und Scan B, wenn man Scan B um (dx, dy) verschiebt.
    // Ein Fehler von 0.0 bedeutet: Beide Scans liegen exakt übereinander!

    double berechneAbweichung(const std::vector<LidarMessung>& scanA,
        const std::vector<LidarMessung>& scanB, double dx, double dy, double dThetaGrad = 0.0) const
    {
        // Sicherheitsabfrage: Beide Scans müssen dieselbe Anzahl Punkte haben
        if (scanA.size() != scanB.size()) return 999.0;

        double gesamtFehler = 0.0;

        // Grad in Bogenmaß (Radians) umrechnen für cos/sin
        double rad = gradZuRad(dThetaGrad);
        double cosT = std::cos(rad);
        double sinT = std::sin(rad);

        // Wir wandern Punkt für Punkt durch beide Scans durch 
        for (size_t i = 0; i < scanA.size(); ++i)
        {
            double ax = 0.0, ay = 0.0;
            double bx = 0.0, by = 0.0;

            // 1. Messpunkte von Polarkoordinaten (Winkel, Distanz) in flache (x, y) Koordinaten umrechnen
            sensorHelper.polarZuKartesisch(scanA[i].winkelGrad, scanA[i].distanzMeter, ax, ay);
            sensorHelper.polarZuKartesisch(scanB[i].winkelGrad, scanB[i].distanzMeter, bx, by);

            // 2. Erst den Punkt B um den Winkel dTheta drehen
            double gedrehtBx = bx * cosT - by * sinT;
            double gedrehtBy = bx * sinT + by * cosT;

            // 3. Dann die Test-Verschiebung (dx, dy) auf Punkt B anwenden
            double verschobenBx = gedrehtBx + dx;
            double verschobenBy = gedrehtBy + dy;

            // 4. Den Abstand zwischen Punkt A und verschobenem Punkt B berechnen
            double diffX = ax - verschobenBx;
            double diffY = ay - verschobenBy;

            // 4. Abstand quadrieren und auf den Gesamtfehler aufsummieren. Durch das Quadrat zählen 
            // größere Abweichungen stärker
            gesamtFehler += (diffX * diffX + diffY * diffY);
        }

        return gesamtFehler;
    }

    // Abschnitt 2: Raster-Suche nach der besten Verschiebung in X und Y
    // Testet systematisch alle Verschiebungen von minDx/minDy bis maxDx/maxDy in kleinen Schritten (z.B alle
    // 0.1 Meter) durch. Gibt das dx/dy zurück, das den KLEINSTEN Abweichungsfehler erzeugt!
    // Sucht im Suchfenster [min, max] mit der Schrittweite schritt den besten Versatz

    void findeBesteVerschiebungUndDrehung2D(const std::vector<LidarMessung>& scanA,
        const std::vector<LidarMessung>& scanB, 
        double minDx, double maxDx, 
        double minDy, double maxDy,
        double minDTheta, double maxDTheta,
        double schrittPos, double schrittWinkel, 
        double& bestesDx, double& bestesDy, double& bestesDTheta) const
    {
        // Notbremse: Verhindert jede Endlosschleife, falls Schrittweiten <= 0 übergeben wurden!
        if (schrittPos <= 0.0 || schrittWinkel <= 0.0)
        {
            bestesDx = minDx;
            bestesDy = minDy;
            bestesDTheta = minDTheta;
            return;
        }

        // Starten mit einem extrem hohen Anfangsfehler als Startwert
        double besterFehler = std::numeric_limits<double>::max();
        bestesDx = minDx;
        bestesDy = minDy;
        bestesDTheta = minDTheta;

        // Schleife 1: Wandert durch die X-Verschiebungen (Vor/Zurück)
        for (double testDx = minDx; testDx <= maxDx + 0.001; testDx += schrittPos)
        {
            // Schleife 2: Wandert durch die Y-Verschiebungen (Links/Rechts)
            for (double testDy = minDy; testDy <= maxDy + 0.001; testDy += schrittPos)
            {
                // Schleife 3: Theta (Drehung im/gegen den Uhrzeigersinn)
                for (double testDTheta = minDTheta; testDTheta <= maxDTheta + 0.001; testDTheta += schrittWinkel)
                {
                    // Wie gut passen die Scans zusammen, wenn wir um (testDx, testDy) verschieben/verdrehen (testDTheta)
                    double fehler = berechneAbweichung(scanA, scanB, testDx, testDy, testDTheta);

                    // Haben wir eine bessere Übereinstimmung gefunden?
                    if (fehler < besterFehler)
                    {
                        besterFehler = fehler;
                        bestesDx = testDx;
                        bestesDy = testDy;
                        bestesDTheta = testDTheta;
                    }
                }
            }
        }
    }

    // Überladungs-Brücke für bestehende Tests (ohne Drehung)
    void findeBesteVerschiebung2D(const std::vector<LidarMessung>& scanA,
        const std::vector<LidarMessung>& scanB, double minDx, double maxDx,
        double minDy, double maxDy, double schritt, 
        double& bestesDx, double& bestesDy) const
    {
        double dummyTheta = 0.0;
        this->findeBesteVerschiebungUndDrehung2D(scanA, scanB, 
            minDx, maxDx, minDy, maxDy, 
            0.0, 0.0, schritt, 1.0, bestesDx, bestesDy, dummyTheta);
    }
};  