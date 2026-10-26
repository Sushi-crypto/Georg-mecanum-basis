#pragma once

// Klasse für die Mecanum-Kinematik
class MecanumKinematik
{
public:
    int berechneRad1(int vy, int vx, int drehung)
    {
        // Mathematische Lösung für Rad1
        return vy + vx + drehung;
    }

    int berechneRad2(int vy, int vx, int drehung)
    {
        // Mathematische Lösung für Rad2
        return vy - vx + drehung;
    }   

    int berechneRad3(int vy, int vx, int drehung)
    {
        // Mathematische Lösung für Rad3 identisch zu Rad2!
        return vy - vx + drehung;
    }  

    int berechneRad4(int vy, int vx, int drehung)
    {
        // Mathematische Lösung für Rad4 identisch zu Rad 1
        return vy + vx + drehung;
    }

};