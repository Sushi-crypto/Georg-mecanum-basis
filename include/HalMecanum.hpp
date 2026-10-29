#pragma once

// Klasse für die HAL Mecanum Räder
class HalMecanum {
public: 
    HalMecanum();

    // Gibt die Geschwindigkeit eines Rads zurück
    // (0: Vorne Links, 1: Vorner rechts, 2: Hinten links, 
    // 3: Hinten Rechts)
    double getWheelSpeed(int wheelIndex) const;


    // Befehl für die Vorwärtsgeschwindigkeit
    void setForwardVelocity(double speed);

    // Befehl für das Seitlichfahren
    void setStrafeVelocity(double speed);
    
private: 
    double wheelSpeeds[4];
};