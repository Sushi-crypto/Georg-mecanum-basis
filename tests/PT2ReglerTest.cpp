#include "CppUTest/TestHarness.h"
#include "PT2Regler.hpp"

TEST_GROUP(PT2ReglerTestGroup)
{

};

// Test 1: Sanftes Abbremsen statt schlagartigem Stillstand 
TEST(PT2ReglerTestGroup, SmoothsAbruptSpeedDrop)
{
    // Wir starten bei voller Fahrt (Startwert = 100.0 m/s)
    PT2Regler regler(100.0);

    double v_soll = 0.0;  // Schlagartige Vollbremsung gefordert!
    double dt = 0.05;     // Kleines Zeitintervall von 50 Millisekunden

    // 1. Erster Zeitschritt: Eimer 1 fängt den Schock ab
    double v_ist_1 = regler.update(v_soll, dt);

    // Geschwindigkeit darf NICHT sofort auf 0.0 abgestürzt sein!
    CHECK(v_ist_1 < 100.0);
    CHECK(v_ist_1 > 80.0);  // Soll noch sanft nahe 100 liegen

    // 2. Zweiter Zeitschritt: Eimer 2 erzeugt die sanfte S-Kurve
    double v_ist_2 = regler.update(v_soll, dt);

    // Die Geschwindigkeit sinkt weiter kontinuierlich, bleibt aber über 0.0
    CHECK(v_ist_2 < v_ist_1);
    CHECK(v_ist_2 > 0.0);
}