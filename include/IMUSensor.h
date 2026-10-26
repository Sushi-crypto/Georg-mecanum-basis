#pragma once

// Klasse für IMU
class IMUSensor
{
private:
    // Globale Gesamtposition (Gesamtkilometerstand)
    double x, y, z;
    
    // Teilstrecke des jeweils LETZTEN Schritts (Tageskilometerzähler)
    double letztesDeltaX, letztesDeltaY, letztesDeltaZ;

    // Aktuelle Geschwindigkeiten
    double vx, vy, vz;

public:
    // Der Konstruktor: Am Anfang steht der Roboter still (Geschwindigkeit 0.0)
    IMUSensor(double x0 = 0.0, double y0 = 0.0, double z0 = 0.0) : x(x0), y(y0), z(z0),
                                                                letztesDeltaX(0.0), letztesDeltaY(0.0), letztesDeltaZ(0.0),
                                                                    vx(0.0), vy(0.0), vz(0.0) {}


    // Setzt den Sensor für einen neuen Testlauf auf Null zurück
    void reset(double startX = 0.0, double startY = 0.0, double startZ = 0.0)
    {
        x = startX; 
        y = startY;
        z = startZ;

        letztesDeltaX = 0.0;
        letztesDeltaY = 0.0;
        letztesDeltaZ = 0.0;

        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
    }

    // Nimmt die Beschleunigungssignale (ax, ay, az) auf und berechnet die neue Geschwindigkeit
    void verarbeiteBeschleunigung(double ax, double ay, double az, double dt)
    {
        // 1. Die Teilstrecke (Delta) für DIESEN konkreten Zeitschritt berechnen:
        // s = v*dt + 0.5*a*dt^2
        letztesDeltaX = vx * dt + 0.5 * ax * dt * dt;
        letztesDeltaY += vy * dt + 0.5 * ay * dt * dt;
        letztesDeltaZ += vz * dt + 0.5 * az * dt * dt;

        // 2. Auf die globale Gesamtposition aufaddieren:
        x += letztesDeltaX;
        y += letztesDeltaY;
        z += letztesDeltaZ;

        // Geschwindigkeiten für den NÄCHSTEN Schritt aufbauen:
        // v = v_alt + a* dt
        vx += ax * dt; 
        vy += ay * dt;
        vz += az * dt;
    }

    // Getter-Methoden für die Einzeletappe (Tageskilometer)
    double getLetztesDeltaX() const { return letztesDeltaX; }
    double getLetztesDeltaY() const { return letztesDeltaY; }
    double getLetztesDeltaZ() const { return letztesDeltaZ; }

    // Getter-Methoden für Position
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    // Getter-Methoden für Geschwindigkeit
    double getGeschwindigkeitX() const { return vx; }
    double getGeschwindigkeitY() const { return vy; }
    double getGeschwindigkeitZ() const { return vz; }
};