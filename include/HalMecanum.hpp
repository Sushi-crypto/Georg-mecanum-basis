#pragma once

// Klasse für die HAL Mecanum Räder
class HalMecanum {
public: 
    HalMecanum();

    // Füttert rohe Enkoder-Ticks für ein Rad (wheelIndex 0...3)
    void updateEncoderTicks(int wheelIndex, int ticksPerSecond);

    // Prüft, ob ein Rad blockiert/geklemmt ist
    bool isWheelStalled(int wheelIndex) const;

    // Gibt die Geschwindigkeit eines Rads zurück
    // (0: Vorne Links, 1: Vorner rechts, 2: Hinten links, 
    // 3: Hinten Rechts)
    double getWheelSpeed(int wheelIndex) const;


    // Befehl für die Vorwärtsgeschwindigkeit
    void setForwardVelocity(double speed);

    // Befehl für das Seitlichfahren
    void setStrafeVelocity(double speed);

    // Rotationsgeschwindigkeit (psoitiv = im Uhrzeigersinn / nach rechts)
    void setRotationVelocity(double speed);

private: 
    double wheelSpeeds[4];

    double commandedSpeeds[4]; // Speichert, was wir befohlen haben!
};