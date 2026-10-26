#pragma once
#include <vector>
#include <cmath>
#include "LidarMessung.h"
#include "OccupancyGrid.h"
#include "RayCaster2D.h"
#include "ScanMatcher2D.h"
#include "MathUtils3D.h"

// Klasse für SlamController2D 
// Verbindet den ScanMatcher 2D (Drift-Korrektur) mit der 2D-Karte, 
// um die Roboter-Pose schrittweise zu aktualisieren und die Karte zu zeichnen

class SlamController2D
{
private:
    ScanMatcher2D matcher;
    RayCaster2D rayCaster;
    OccupancyGrid karte;
    double aufloesung;

    // Die geschätzte Pose des Roboters in der Welt (X, Y, Theta)
    double roboterX;
    double roboterY;
    double roboterTheta;

    // Der vorherige Scan als Referenz für den nächsten Vergleich
    std::vector<LidarMessung> alterScan;
    bool hatErstenScan;

public:
    // Konstruktor: Erstellt eine Karte mit gegebener Breite, Höhe und 
    // Auflösung
    SlamController2D(double breiteMeter, double hoeheMeter, double aufloesungMeter) 
    : karte(static_cast<int>(std::round(breiteMeter / aufloesungMeter)), 
        static_cast<int>(std::round(hoeheMeter / aufloesungMeter)), 
            aufloesungMeter), aufloesung(aufloesungMeter),
             roboterX(0.0), roboterY(0.0), roboterTheta(0.0), hatErstenScan(false)
    {

    }

    // Verarbeitet den nächsten LIDAR-Scan vom Sensor
    void verarbeiteScan(const std::vector<LidarMessung>& neuerScan)
    {
        if (neuerScan.empty()) return;

        // Fall 1: Das ist der allererste Scan beim Starten des Roboters
        if (!hatErstenScan)
        {
            // Karte direkt an Startposition (0,0) einzeichnen
            zeichneScanInKarte(neuerScan);
            alterScan = neuerScan;
            hatErstenScan = true;
            return;
        }

        // Fall 2: Folge-Scan -> Abweichung zum vorherigen Scan ermitteln!
        double dx = 0.0;
        double dy = 0.0;
        double dTheta = 0.0;

        // Suche im Suchfenster (-0.5m bis +0.5m, -10 bis +10 Grad)
        matcher.findeBesteVerschiebungUndDrehung2D(
            alterScan, neuerScan,
            -0.5, 0.5,  // minDx, maxDx
            -0.5, 0.5,  // minDy, maxDy
            -10.0, 10.0, //minDTheta, maxDTheta
            0.05, 1.0,   // Schrittweiten: 5 cm, 1 Grad
            dx, dy, dTheta
        );

        // 1. Roboter-Pose um den gemessenen Schlupf korrigieren
        roboterX += dx;
        roboterY += dy;
        roboterTheta += dTheta;

        // 2. Den neuen Scan an der korrigierten Pose in die karte eintragen
        zeichneScanInKarte(neuerScan);

        // 3. Den aktuellen Scan als neue Referenz für das nächste Mal merken
        alterScan = neuerScan;
    }

    // Hilfsmethode: Trägt alle Strahlen eines Scans zur Roboterpose ein
    void zeichneScanInKarte(const std::vector<LidarMessung>& scan)
    {
        for (const auto& messung : scan)
        {
            // Der Strahl startet an der aktuellen Roboterposition
            double startX = roboterX;
            double startY = roboterY;

            // Messwinkel im Roboterkoordinatensystem + globale Roboter-Ausrichtung
            double gesamtwinkelGrad = messung.winkelGrad + roboterTheta;
            double rad = gradZuRad(gesamtwinkelGrad);

            // Zielounkt des Laserstrahls berechnen
            double zielX = startX + messung.distanzMeter * std::cos(rad);
            double zielY = startY + messung.distanzMeter * std::sin(rad);

            // Strahl in das Raster einzeichnen (Freier Weg + Hindernis-Punkt)
            rayCaster.zeichneStrahl(karte, startX, startY, zielX, zielY, aufloesung);
        }
    }

    // Getter für die aktuelle Roboter-Pose
    double getX() const { return roboterX; }
    double getY() const { return roboterY; }
    double getTheta() const { return roboterTheta; }

    // Zugriff auf die Karte (z.B. für Tests)
    const OccupancyGrid& getKarte() const { return karte; }   
};