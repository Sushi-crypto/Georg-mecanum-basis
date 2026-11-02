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
    Bug3Controller(double abstand = 0.5) 
        : currentState(BugState::GOAL_SEEKING), 
          maxSpeed(0.5), 
          sicherheitsAbstand(abstand) {}

    Bug3Output update(double x, double y, double theta,
        double goalX, double goalY, 
        const std::vector<LidarMessung>& scan)
    {
        Bug3Output output;

        // 1. Vektor zum Ziel berechnen
        double dx = goalX - x;
        double dy = goalY - y;
        double distanzZumZiel = std::sqrt(dx * dx + dy * dy);   // Messen Luftlinie
        double zielWinkelRad = std::atan2(dy, dx) - theta;
        double zielWinkelGrad = radZuGrad(zielWinkelRad);

        // 2. Prüfen, ob im Blickwinkel zum Ziel ein Hindernis liegt
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

        // 3. Zustand Umschalten (State Machine)
        if (currentState == BugState::GOAL_SEEKING && hindernisVoraus) {
            // Hindernis entdeckt -> Wand folgen
            currentState = BugState::WALL_FOLLOWING;
        }
        else if (currentState == BugState::WALL_FOLLOWING && ! zielWegBlockiert) {
            // Sichtlinie zum Ziel ist wieder frei -> Zurück auf Zielkurs!
            currentState = BugState::GOAL_SEEKING;
        }

        output.state = currentState;

        // 4. Bewegungsbefehle generieren
        if (currentState == BugState::GOAL_SEEKING) {
            if (distanzZumZiel > 0.001) {

                // Normierter Richtungsvektor mal maxSpeed -> Geht in X- UND Y-Richtung
                output.vx = maxSpeed * (dx / distanzZumZiel);
                output.vy = maxSpeed * (dy / distanzZumZiel);

                // Sanfte Ausrichtung in Zielrichtung 
                double kp_omega = 1.0;
                output.omega = kp_omega * zielWinkelRad;
                if (output.omega > 1.0) output.omega = 1.0;
                if (output.omega < -1.0) output.omega = -1.0;

            } else {

                output.vx = 0.0;
                output.vy = 0.0;
                output.omega = 0.0;

            }  
        }
        else if (currentState == BugState::WALL_FOLLOWING) {

            // Minimalen Abstand zum Hindernis voraus ermittelt
            double minHindernisAbstand = sicherheitsAbstand;
            for (const auto& messung : scan) {
                if (std::abs(messung.winkelGrad) < 30.0) {
                    if (messung.distanzMeter < minHindernisAbstand) {
                        minHindernisAbstand = messung.distanzMeter;
                    }
                }
            }

            // Dämpfungsfaktor (0.0 bis 1.0) basierend auf dem Abstand berechnen
            double abstandsFaktor = minHindernisAbstand / sicherheitsAbstand;
            if (abstandsFaktor < 0.0) abstandsFaktor = 0.0;
            if (abstandsFaktor > 1.0) abstandsFaktor = 1.0;

            // Gescheidiges Überblenden: Vorwärts sanft reduzieren, Seitwärts sanft 
            // aufbauen (Elchtest-Prinzip)
            output.vx = maxSpeed * abstandsFaktor;
            output.vy = maxSpeed * (1.0 - abstandsFaktor);

            // Dynamisches Kp für die Drehung (omega)
            // Je näher das Hindernis rückt (abstandsFaktor -> 0.0), desto stärker
            // steigt kP von 0.5 auf 2.5 an!
            double kp_basis = 0.5;
            double kp_dynamisch = kp_basis + (1.0 - abstandsFaktor) * 2.0;

            // Sobald vx unter den kritischen  Schwellenwert von 0.2 m/s fällt, reißt 
            // der Roboter die Nase weg!
            if (output.vx < 0.2) {
                output.omega = -kp_dynamisch * (1.0 - abstandsFaktor);
            } else {
                output.omega = 0.0;
            }
        }

        return output;
    }
};