#pragma once

// Klasse für den PT2-Regler bzgl. der Hinderniserkennung
class PT2Regler
{
private:
    double eimer1; // Erste Dämpfungsstude (PT1)
    double eimer2; // Zweite Dämpfungsstufe (PT2)
    double T;       // Zeitkonstante (je größer T, desto weicher/langsamer)

public:
    // Der Konstruktor: Standardgemäß starten wir bei 100.0, 
    // aber wir können jetzt auch mit 0.0 anfangen

    PT2Regler(double startwert = 0.0, double zeitkonstante = 0.2)
        : eimer1(startwert), 
          eimer2(startwert), 
          T(zeitkonstante)
    {
    }

    // Überprüft im dt-Zeitfenster was der Status des Eimerstandes ist

    double update(double v_soll, double dt)
    {
        // Eimer 1 nähert sich dem Sollwert an
        eimer1 += (v_soll - eimer1) * (dt/T);

        // Eimer 2 nähert sich dem Stand von Eimer 1 an (S-Kurve!)
        eimer2 += (eimer1 - eimer2) * (dt/T);

        return eimer2; // Das ist die geglättete Ist-Geschwindigkeit!
    }
};