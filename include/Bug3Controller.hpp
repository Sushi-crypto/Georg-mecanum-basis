#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
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
    double nachlaufZeit;    // 0.5 Sekunden Nachlauf
    double nachlaufTimer;   // Verbleibende Nachlaufzeit

public:
    Bug3Controller(double abstand = 0.4) 
        : currentState(BugState::GOAL_SEEKING), 
          maxSpeed(0.5), 
          sicherheitsAbstand(abstand),
          nachlaufZeit(0.5),
          nachlaufTimer(0.0) {}

    Bug3Output update(double x, double y, double theta,
        double goalX, double goalY, 
        const std::vector<LidarMessung>& scan,
        double dt = 0.05)
    {
        Bug3Output output;

        // 1. Vektor zum Ziel berechnen
        double dx = goalX - x;
        double dy = goalY - y;
        double distanzZumZiel = std::sqrt(dx * dx + dy * dy);   
        

        // 2. Prüfen, ob im Blickwinkel zum Ziel ein Hindernis liegt
        bool hindernisVoraus = false;
        bool hindernisSeitlich = false;

        for (const auto& messung : scan) {

            // Hindernis direkt voraus? (Front-Sensor = +- 20°)
            if (std::abs(messung.winkelGrad) <= 20.0 
                && messung.distanzMeter < sicherheitsAbstand) {
                    hindernisVoraus = true;
            }

            // Hindernis seitlich? (Seiten-Sensor 30° bis 120°)
            if (messung.winkelGrad >= 30.0 && messung.winkelGrad <= 120.0
                && messung.distanzMeter< sicherheitsAbstand) {
                    hindernisSeitlich = true;
                }
        }

        // 3. Zustand Umschalten (State Machine)
        if (currentState == BugState::GOAL_SEEKING) {
            if (hindernisVoraus) {
                currentState = BugState::WALL_FOLLOWING;
                nachlaufTimer = nachlaufZeit;   // 0.5s Timer startet
            }
        }
        else if (currentState == BugState::WALL_FOLLOWING) {
            if (hindernisVoraus || hindernisSeitlich) {
                // Wand ist aktiv da -> Timer auf 0.5s halten
                nachlaufTimer = nachlaufZeit;
            } else {
                // Wand ist weg -> 0.5s runterzählen
                nachlaufTimer -= dt;
                if (nachlaufTimer <= 0.0) {
                    currentState = BugState::GOAL_SEEKING;  // Erst jetzt umschalten!
                }
            }
        }

        output.state = currentState;

        // 4. Bewegungsbefehle generieren
        if (currentState == BugState::GOAL_SEEKING) {
            
            if (distanzZumZiel > 0.05) {

                // Normierter Richtungsvektor mal maxSpeed -> Geht in X- UND Y-Richtung
                output.vx = maxSpeed * (dx / distanzZumZiel);
                output.vy = maxSpeed * (dy / distanzZumZiel);
                output.omega = 0.0;

            } else {

                output.vx = 0.0;
                output.vy = 0.0;
                output.omega = 0.0;

            }  
        }
        else if (currentState == BugState::WALL_FOLLOWING) {
            if (hindernisVoraus) {
                // Vor dem Hindernis: Vorwärts stoppen, rein seitlich ausweichen
                output.vx = 0.2;
                output.vy = maxSpeed;
                output.omega = 0.0;
            } else {
                // An der Flanke entlang: Vorwärtsgleiten an der Kiste
                output.vx = 0.25;
                output.vy = 0.0;
                output.omega = 0.0;
            }           
        }
        return output;
    }
};