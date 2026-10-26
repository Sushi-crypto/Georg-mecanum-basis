#pragma once
#include <cmath>

// Struktur für einen einzelnen Punkt im 3D-Raum
struct Point3D
{
    double x; 
    double y; 
    double z;
};

// Grad-Werte müssen für trigonometrische Funktion in rad umgewandelt werden
inline double gradZuRad(double grad) {
    return grad * (3.14159265358979323846 / 180.0);
}

// Funktion um rad in Grad-Werte umzuwandeln
inline double radZuGrad(double rad) {
    return rad * (180.0 / 3.14159265358979323846);
}

// Wandelt einen lokalen Laserpunkt (x, y, z) mit Hilfe 
// der 6D-Roboterpose in Weltkoordinaten um
// Roll, Pitch und Yaw sollen in Grad angegeben werden!
inline Point3D transformierePunkt3D(const Point3D& lokal, 
    double roboterX, double roboterY, double roboterZ,
    double rollGrad, double pitchGrad, double yawGrad)
{
    // Grad-Werte in Rad umwandeln
    double roll = gradZuRad(rollGrad);
    double pitch = gradZuRad(pitchGrad);
    double yaw = gradZuRad(yawGrad);

    // Trigonometrische Vorberechnungen für Sinus und Cosinus
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // Drehung des Punktes im Raum über dei 3D-Rotationsmatrix 
    // (Yaw-Pitch-Roll)
    double xRot = lokal.x * (cy * cp) + 
                  lokal.y * (cy * sp * sr - sy * cr) +
                  lokal.z * (cy * sp * cr + sy * sr);

    double yRot = lokal.x * (sy * cp) +
                  lokal.y * (sy * sp * sr + cy * cr) +
                  lokal.z * (sy * sp * cr - cy * sr);

    double zRot = lokal.x * (-sp) +
                  lokal.y * (cp * sr) +
                  lokal.z * (cp * cr);

    // 2. Translation: Roboterposition in der Welt hinzuaddieren 
    Point3D welt;
    welt.x = xRot + roboterX;
    welt.y = yRot + roboterY;
    welt.z = zRot + roboterZ;

    return welt;
}
