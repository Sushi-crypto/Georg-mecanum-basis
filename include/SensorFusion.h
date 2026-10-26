#pragma once
#include <cmath>

// Klasse für Sensor-Fusion
class SensorFusion
{
private:
    double alphaStandard;

public:
    // Der Konstruktor hat 0.8 als Standardwert 
    SensorFusion(double vertrauensFaktorAlpha = 0.8)
    {
        // Plausibilitäts-Check
        if (vertrauensFaktorAlpha < 0.0)
        {
            alphaStandard = 0.0;

        } else if (vertrauensFaktorAlpha > 1.0){

            alphaStandard = 1.0;

        } 
    }

    // ----------- Manuelles Eintippen des Alpha-Wertes --------------------------

    double fusedGeschwindigkeitX(double odomVx, double imuVx) const
    {
        return alphaStandard * odomVx + (1.0 - alphaStandard) * imuVx;
    }

    double fusedGeschwindigkeitY(double odomVy, double imuVy) const
    {
        return alphaStandard * odomVy + (1.0 - alphaStandard) * imuVy;
    }

    double fusedGeschwindigkeitZ(double odomVz, double imuVz) const
    {
        return alphaStandard * odomVz + (1.0 - alphaStandard) * imuVz;
    }

    // ---------------- 3-Stufen Vollautomatik -----------------------------------

    // Vollautomatische Fusion für X- & Y-Richtung (Drift und Aquaplaning)
    // X-Richtung
    double fusedGeschwindigkeitXVollautomatisch(double odomVx, double imuVx) const
    {
        // 1. Abweichung zwischen den beiden Sensoren messen
        double differenz = odomVx - imuVx;
        double alpha;

        // 2. Automatische Zustandserkennung
        if (differenz > 1.0) // Stufe 1
        {
            // Notfall: Aquaplaning / Radschlupf erkannt! -> v_odom >> v_imu
            // Odometrie ist unzuverlässig -> Alpha auf 0.1 senken!
            alpha = 0.1;

        } else if (differenz < -1.0) { // Stufe 2

            // Notfall: Negativer Schlupf! -> ABS-Fall -> v_imu >> v_odom
            // Räder blockieren beim Bremsen ODER IMU-Glitch/Fremdschub!
            // speziellen Wert einschalten mehr auf IMU vertrauen
            alpha = 0.2;

        } else if (std::abs(differenz) >= 0.2) { // Stufe 3

            // Moderate Abweichung: Teppich / Zu starker Grip! -> IMU wackelt leicht!
            // Räder greifen stark -> Hohes Vertrauen in Odometrie (Alpha = 0.9)
            alpha = 0.9;

        } else { // Stufe 4

            // Normales Fahren -> Differenz zwischen Odom und IMU liegen unter 0,2!
            // 80% Vertrauen in Odom, 20% Vertrauen in IMU
            alpha = 0.8;

        }

        return alpha * odomVx + (1.0 - alpha) * imuVx;
    }

    // Y-Richtung
    double fusedGeschwindigkeitYVollautomatisch(double odomVy, double imuVy) const
    {
        // 1. Abweichung zwischen den beiden Sensoren messen
        double differenz = odomVy - imuVy;
        double alpha;

        // 2. Automatische Zustandserkennung
        if (differenz > 1.0) // Stufe 1
        {
            // Notfall: Aquaplaning / Radschlupf erkannt! -> v_odom >> v_imu
            // Odometrie ist unzuverlässig -> Alpha auf 0.1 senken!
            alpha = 0.1;

        } else if (differenz < -1.0) { // Stufe 2

            // Notfall: Negativer Schlupf! -> ABS-Fall -> v_imu >> v_odom
            // Räder blockieren beim Bremsen ODER IMU-Glitch/Fremdschub!
            // speziellen Wert einschalten mehr auf IMU vertrauen
            alpha = 0.2;

        } else if (std::abs(differenz) >= 0.2) { // Stufe 3

            // Moderate Abweichung: Teppich / Zu starker Grip! -> IMU wackelt leicht!
            // Räder greifen stark -> Hohes Vertrauen in Odometrie (Alpha = 0.9)
            alpha = 0.9;

        } else { // Stufe 4

            // Normales Fahren -> Differenz zwischen Odom und IMU liegen unter 0,2!
            // 80% Vertrauen in Odom, 20% Vertrauen in IMU
            alpha = 0.8;

        }

        return alpha * odomVy + (1.0 - alpha) * imuVy;
    }

    // Vollautomatische Fusion Z (Kantenerschüttung & Fallen)
    double fusedGeschwindigkeitZVollautomatisch(double odomVz, double imuVz) const
    {
        double differenz = odomVz - imuVz;
        double alpha;

        if (differenz > 0.5) // Stufe 1
        {
            // Erschütterung durch Teppichkante oder Absatz!
            // Wir vertrauen zu 90% der flachen Odometrie (alpha = 0.9),
            // um den IMU-Ruckler komplett wegzubügeln!
            alpha = 0.9;

        } else if (differenz < -0.5) { // Stufe 2

            // Steigen / Auffahren (v_imu stößt nach oben: z.B. Türschwelle)
            // Brauchen dem Odom nicht so viel zu vertrauen wie Stufe 1!
            alpha = 0.85;
            
        } else {

            // Ruhige Fahrt
            alpha = 0.8;

        }

        return alpha * odomVz + (1.0 - alpha) * imuVz;
    }
};