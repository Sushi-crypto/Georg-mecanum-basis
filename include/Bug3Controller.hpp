#pragma once
#include <vector>
#include <cmath>
#include "LidarMessung.h"
#include "MathUtils3D.h"

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
    double sicherheitsAbstand;

public:
    Bug3Controller(double abstand = 0.5) : currentState(BugState::GOAL_SEEKING), 
        maxSpeed(0.5), sicherheitsAbstand(abstand) {}

    Bug3Output update(double x, double y, double theta,
        double goalX, double goalY, const std::vector<LidarMessung>& scan)
    {
        Bug3Output output;

        // Vektor zum Ziel berechnen
        double dx = goalX - x;
        double dy = goalY - y;
        double zielWinkelRad = std::atan2(dy, dx) - theta;
        double zielWinkelGrad = radZuGrad(zielWinkelRad);

        // 1. Prüfen, ob im Blickwinkel zum Ziel ein Hindernis liegt
        bool zielWegBlockiert = false;
        bool hindernisVoraus = false;

        for (const auto& messung : scan) {

            // Hindernis direkt voraus?
            if (std::abs(messung.winkelGrad) < 15.0 
                && messung.distanzMeter < sicherheitsAbstand) {
                    hindernisVoraus = true;
            }

            // Hindernis in Richtung des Ziels?
            double winkelDifferenz = std::abs(messung.winkelGrad - zielWinkelGrad);
            if (winkelDifferenz < 15.0 && messung.distanzMeter < sicherheitsAbstand) {
                zielWegBlockiert = true;
            }
        }

        // 2. Zielumschaltung (State Machine)
        if (currentState == BugState::GOAL_SEEKING && hindernisVoraus) {
            // Hindernis entdeckt -> Wand folgen
            currentState = BugState::WALL_FOLLOWING;
        }
        else if (currentState == BugState::WALL_FOLLOWING && ! zielWegBlockiert) {
            // Sichtlinie zum Ziel ist wieder frei -> Zurück auf Zielkurs!
            currentState = BugState::GOAL_SEEKING;
        }

        output.state = currentState;

        // 3. Bewegungsbefehle generieren
        if (currentState == BugState::GOAL_SEEKING) {
            // Wenn der Pfad frei ist: Fahre mit maxSpeed in Richtung Ziel
            output.vx = maxSpeed;
            output.vy = 0.0;
            output.omega = 0.0;
        }
        else if (currentState == BugState::WALL_FOLLOWING) {
            // Beim Wandfolgen gleitet der Roboter seitlich am Hindernis vorbei
            output.vx = 0.0;
            output.vy = maxSpeed; // Krabben-Gang nach rechts um das Hindernis herum
            output.omega = 0.0;
        }

        return output;
    }
};