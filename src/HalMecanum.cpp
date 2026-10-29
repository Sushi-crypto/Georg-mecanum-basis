#include "HalMecanum.hpp"

HalMecanum::HalMecanum(){
    
    // Beim Einschalten der HAL stehen alle 4 Räder sicher auf 0.0 m/s!
    for (int i = 0; i < 4; ++i)
    {
        wheelSpeeds[i] = 0.0;
    }
}

double HalMecanum::getWheelSpeed(int wheelIndex) const {

    if (wheelIndex >= 0 && wheelIndex < 4) {
        return wheelSpeeds[wheelIndex];
    }

    return 0.0;
}

// Vorwärtsfahrt setzt alle 4 Räder auf die gleiche Geschwindigkeit
void HalMecanum::setForwardVelocity(double speed) {
    for (int i = 0; i < 4; ++i){
        wheelSpeeds[i] = speed;
    }
}

// Seitwärtsfahrt nutzt die 45°-Rollen der Diagonalen 
void HalMecanum::setStrafeVelocity(double speed) {
    wheelSpeeds[0] = speed; // Vorne Links: +
    wheelSpeeds[1] = -speed; // Vorne Rechts: -
    wheelSpeeds[2] = -speed; // Hinten Links: -
    wheelSpeeds[3] = speed; // Vorne Links: +
}