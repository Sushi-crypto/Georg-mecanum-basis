#pragma once

// Das C++ Interface
class IHalDrive {
public:
    virtual ~IHalDrive() = default;

    // Jedes Fahrwerk MUSS diese Befehle beherrschen:
    virtual void drive(double vx, double vy, double omega) = 0;
    virtual double getWheelSpeed(int wheelIndex) const = 0;
    virtual void updateEncoderTicks(int wheelIndex, int ticksPerSecond) = 0;
    virtual bool isWheelStalled(int wheelIndex) const = 0; 
};