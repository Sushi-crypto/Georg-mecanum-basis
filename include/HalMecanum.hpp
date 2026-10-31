#pragma once
#include "IHalDrive.hpp"

/**
 * @brief Hardware Abstraction Layer (HAL) für ein Mecanum-Fahrwerk 
 * mit 4 Rädern
 * 
 * Diese Klasse bildet die Brücke zwischen der klugen Navigations-Software
 * und den echten Motoren/Sensoren an den 4 Rädern:
 * Index 0: Vorne Links (FL)
 * Index 1: Vorne Rechts (FR)
 * Index 2: Hinten Links (HL)
 * Index 3: Hinten Rechts (HR)
 */

// Klasse für die HAL Mecanum Räder
class HalMecanum : public IHalDrive{
public: 
    // Konstruktor: Setzt beim Einschalten alle Geschwindigkeiten 
    // sicher auf 0.0 m/s
    HalMecanum();


    /** Methoden aus dem IHALDrive-Vertrag
     * Das 'override' garantiert, dass wir das Universal-Interface
     * exakt erfüllen. Universal-Befehl für beliebige kombinierte Bewegungen.
     * Das 'override' garantiert, dass wir das Universal-Interface exakt erfüllt.
     */ 
    
    // Universal-Befehl für beliebige kombinierte Bewegungen
    void drive(double vx, double vy, double omega) override;

    /**
     * @brief Liest die aktuell gemessene Geschwindigkeit eines einzelnen 
     * Rades aus
     */
    double getWheelSpeed(int wheelIndex) const override;

    /**
     * @brief Nimmt rohe Sensor-Impulse (Ticks) vom Rad-Enkoder und wandelt 
     * sie in m/s um 
     */
    void updateEncoderTicks(int wheelIndex, int ticksPerSecond) override;

    // Prüft, ob ein Rad blockiert/geklemmt ist
    bool isWheelStalled(int wheelIndex) const override;

    /**
     * Einzelbefehle (Hilfsmethoden für gezielte Tests oder einfache
     * Manöver)
     */
    void setForwardVelocity(double speed); // Vorwärts- bzw. Rückwärtsfahrt
    void setStrafeVelocity(double speed); // Reiner Krabbengang nach links/rechts
    void setRotationVelocity(double speed); // Reine Pirouette

private: 
    double wheelSpeeds[4];
    
    double commandedSpeeds[4]; // Speichert, was wir befohlen haben!
};