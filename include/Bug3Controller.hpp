#pragma once
#include <vector>
#include <cmath>
#include "LidarMessung.h"

// Die beiden Zustände unseres Käfers
enum class BugState {
    GOAL_SEEKING, 
    WALL_FOLLOWING
};

// Ausgabestruktur für Fahrbefehl + aktuellen Zustand
struct Bug3Output {
    double vx;
    double vy;
    double omega;
    BugState state;
};

class Bug3Controller {
private:
    BugState currentState;
    double maxSpeed;

public:
    Bug3Controller() : currentState(BugState::GOAL_SEEKING), maxSpeed(0.5) {}

    Bug3Output update(double x, double y, double theta,
        double goalX, double goalY, const std::vector<LidarMessung>& scan)
    {
        Bug3Output output;
        output.state = currentState;

        // Vektor zum Ziel berechnen
        double dx = goalX - x;
        double dy = goalY - y;
        double distanzZumZiel = std::sqrt(dx * dx + dy * dy);

        if (currentState == BugState::GOAL_SEEKING) {
            // Wenn der Pfad frei ist: Fahre mit maxSpeed in Richtung Ziel
            output.vx = maxSpeed;
            output.vy = 0.0;
            output.omega = 0.0;
        }

        return output;
    }
};