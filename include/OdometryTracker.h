#pragma once
#include "MathUtils3D.h"

// Klasse für Odometrie
class OdometryTracker
{
private:
    double x;
    double y;
    double z;
    double ausrichtungGrad; // 0° = Osten (+X), 90° = Norden (+Y)

    double vx;
    double vy;
    double vz;

public:
    // Der Konstruktor: Setzt die Startposition des Roboters (Standard: 0.0, 0.0, 0.0, 0.0)
    OdometryTracker(double x0 = 0.0, double y0 = 0.0, double z0 = 0.0, double theta0Grad = 0.0) : 
                    x(x0), y(y0), z(z0), ausrichtungGrad(theta0Grad), vx(0.0), vy(0.0), vz(0.0)
    {

    }

    void setzeAusrichtungGrad(double grad)
    {
        ausrichtungGrad = grad;
    }

    double getAusrichtungGrad() const
    {
        return ausrichtungGrad;
    }

    // Berechne die neue Postition und merke die Geschwindigkeiten (vx, vy, vz) und der Zeitspanne (dt) 
    // unter Berücksichtigung des Ausrichtungsgrades

    void aktualisiereGeschwindigkeitOdometrie(double neueVx, double neueVy, double neueVz, double dt)
    {
        // Die eingehende Geschwindigkeiten in Objekten gespeichert
        vx = neueVx;
        vy = neueVy;
        vz = neueVz;

        double rad = gradZuRad(ausrichtungGrad);

        // Vorwärtsbewegung (vx) wird auf X und Y aufgeteilt anhand der Blickrichtung
        double deltaX = (vx * std::cos(rad) - vy * std::sin(rad)) * dt;
        double deltaY = (vx * std::sin(rad) + vy * std::cos(rad)) * dt;
        double deltaZ = vz * dt; // Z-Achse (Höhe) bleibt unabhängig von der Drehung am Boden

        x += deltaX;
        y += deltaY;
        z += deltaZ;
    }

    // Getter-Methoden zum Abfragen der aktuellen Position
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    // Getter-Methoden zum Abfragen der gefahrenen Geschwindigkeiten
    double getGeschwindigkeitX() const { return vx; }
    double getGeschwindigkeitY() const { return vy; }
    double getGeschwindigkeitZ() const { return vz; }
};