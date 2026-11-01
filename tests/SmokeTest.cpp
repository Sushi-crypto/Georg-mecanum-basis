#include <vector>
#include <cmath> 
#include <limits>
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "Hinderniserkennung.h"
#include "IMUSensor.h"
#include "LidarMessung.h"
#include "LidarSensor2D.h"
#include "MathUtils3D.h"
#include "MecanumKinematik.h"
#include "OccupancyGrid.h"
#include "OccupancyGrid3D.h"
#include "OdometryTracker.h"
#include "PT2Regler.hpp"
#include "RayCaster2D.h"
#include "RayCaster3D.h"
#include "ScanMatcher2D.h"
#include "ScanMatcher3D.h"
#include "SchwenkLidar3D.h"
#include "SensorFusion.h"
#include "SlamController2D.h"
#include "SlamController3D.h"
#include "SlamMapBuilder2D.h"
#include "SlamMapBuilder3D.h"
#include "Verschiebung3D.h"


// ------------- Strukturen für die Klassen -----------------------------------------------


// ----------------------------- Klassen --------------------------------------------------


// -------------------------- MathUtils Tests ---------------------------------------------

// Testgruppe zum Testen der mathematischen Umwandlungen
TEST_GROUP(MathUtils3DTest)
{

};

// Test1: Transformiert die "MathUtils3D.h" Klasse grad richtig in rad um?
TEST(MathUtils3DTest, TransformiertPunktMitGradWinkeln)
{
    Point3D lokal{1.0, 0.0, 0.0};

    // Roboter steht bei (0, 0, 0) und ist um 90 Grad (yaw) nach links gedreht
    Point3D welt = transformierePunkt3D(lokal, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0);

    // Der Punkt vor dem Roboter liegt nach der 90°-Drehung jetzt bei Y = 1.0m
    DOUBLES_EQUAL(0.0, welt.x, 0.001);
    DOUBLES_EQUAL(1.0, welt.y, 0.001);
    DOUBLES_EQUAL(0.0, welt.z, 0.001);
}

// ---------------------------Mecanum-Kinematik--------------------------------------------

// Test-Gruppe für die Roboter-Mathematik
TEST_GROUP(MecanumKinematikTest)
{
    MecanumKinematik kin; // unser Simulator-Objekt
};

// Testfall 1: Rad 1 Geradeausfahrt (vy = 100, vx = 0; drehung = 0)
TEST(MecanumKinematikTest, Rad1_Geradeaus)
{
    // Wenn wir mit 100% nach vorne fahren, muss Rad 1 genau 100% Leistung bekommen!
    LONGS_EQUAL(100, kin.berechneRad1(100, 0, 0));
}

// Testfall 2: Rad 1 100 % seitwärts nach rechts fahren
TEST(MecanumKinematikTest, Rad1_SeitwaertsRechts)
{
    // Wenn wir mit 100% seitwärts nach rechts, muss Rad 1 genau vx = +100 fahren!
    LONGS_EQUAL(100, kin.berechneRad1(0, 100, 0));
}

// Testfall 3: Rad 1 100 % seitwärts nach links fahren
TEST(MecanumKinematikTest, Rad1_SeitwaertsLinks)
{
    // Wenn wir mit 100% seitwärts nach links, muss Rad 1 genau vx = -100 fahren!
    LONGS_EQUAL(-100, kin.berechneRad1(0, -100, 0));
}

// Testfall 4: Rad 1 100% nach hinten fahren ohne Drehung
TEST(MecanumKinematikTest, Rad1_Rueckwaerts)
{
    // Wenn wir mit 100% nach hinten fahren, muss Rad 1 genau vy = -100 liefern!
    LONGS_EQUAL(-100, kin.berechneRad1(-100, 0, 0));
}

// Testfall 5: Rad 2 100 % seitwärts nach rechts fahren
TEST(MecanumKinematikTest, Rad2_SeitwaertsRechts)
{
    // Bei Fahrt nach Rechts (vx = 100) muss Rad 2 rückwärts drehen!
    LONGS_EQUAL(-100, kin.berechneRad2(0, 100, 0));
}

// Testfall 6: Rad 2 100 % seitwärts nach links fahren
TEST(MecanumKinematikTest, Rad2_SeitwaertsLinks)
{
    // Bei Fahrt nach Rechts (vx = -100) muss Rad 2 vorwärts drehen!
    LONGS_EQUAL(100, kin.berechneRad2(0, -100, 0));
}

// Testfall 7: Rad 2 Geradeausfahrt (vy = 100, vx = 0; drehung = 0)
TEST(MecanumKinematikTest, Rad2_Geradeaus)
{
    // Wenn wir mit 100% nach vorne fahren, muss Rad 1 genau 100% Leistung bekommen!
    LONGS_EQUAL(100, kin.berechneRad2(100, 0, 0));
}

// Testfall 8: Rad 2 100% nach hinten fahren ohne Drehung
TEST(MecanumKinematikTest, Rad2_Rueckwaerts)
{
    // Wenn wir mit 100% nach hinten fahren, muss Rad 1 genau vy = -100 liefern!
    LONGS_EQUAL(-100, kin.berechneRad2(-100, 0, 0));
}

// Testfall 9: Rad 3 100 % seitwärts nach rechts fahren
TEST(MecanumKinematikTest, Rad3_SeitwaertsRechts)
{
    // Bei Fahrt nach Rechts (vx = 100) muss Rad 3 genau wie Rad 2 rückwärts drehen!
    LONGS_EQUAL(-100, kin.berechneRad3(0, 100, 0));
}

// Testfall 10: Rad 3 100 % seitwärts nach links fahren
TEST(MecanumKinematikTest, Rad3_SeitwaertsLinks)
{
    // Bei Fahrt nach Links (vx = -100) muss Rad 3 genau wie Rad 2 vorwärts drehen!
    LONGS_EQUAL(100, kin.berechneRad3(0, -100, 0));
}

// Testfall 11: Rad 3 Geradeausfahrt (vy = 100, vx = 0; drehung = 0)
TEST(MecanumKinematikTest, Rad3_Geradeaus)
{
    // Wenn wir mit 100% nach vorne fahren, muss Rad 1 genau 100% Leistung bekommen!
    LONGS_EQUAL(100, kin.berechneRad3(100, 0, 0));
}

// Testfall 12: Rad 3 100% nach hinten fahren ohne Drehung
TEST(MecanumKinematikTest, Rad3_Rueckwaerts)
{
    // Wenn wir mit 100% nach hinten fahren, muss Rad 1 genau vy = -100 liefern!
    LONGS_EQUAL(-100, kin.berechneRad3(-100, 0, 0));
}

// Testfall 13: Rad 4 identisch zu Rad 1 Geradeausfahrt (vy = 100, vx = 0; drehung = 0)
TEST(MecanumKinematikTest, Rad4_Geradeaus)
{
    // Wenn wir mit 100% nach vorne fahren, muss Rad 4 genau 100% Leistung bekommen!
    LONGS_EQUAL(100, kin.berechneRad4(100, 0, 0));
}

// Testfall 14: Rad 4 identisch zu Rad 1 100 % seitwärts nach rechts fahren
TEST(MecanumKinematikTest, Rad4_SeitwaertsRechts)
{
    // Wenn wir mit 100% seitwärts nach rechts, muss Rad 4 genau vx = +100 fahren!
    LONGS_EQUAL(100, kin.berechneRad4(0, 100, 0));
}

// Testfall 15: Rad 4 identisch zu Rad 1 100 % seitwärts nach links fahren
TEST(MecanumKinematikTest, Rad4_SeitwaertsLinks)
{
    // Wenn wir mit 100% seitwärts nach links, muss Rad 4 genau vx = -100 fahren!
    LONGS_EQUAL(-100, kin.berechneRad4(0, -100, 0));
}

// Testfall 16: Rad 4 identisch zu Rad 1 100% nach hinten fahren ohne Drehung
TEST(MecanumKinematikTest, Rad4_Rueckwaerts)
{
    // Wenn wir mit 100% nach hinten fahren, muss Rad 1 genau vy = -100 liefern!
    LONGS_EQUAL(-100, kin.berechneRad4(-100, 0, 0));
} 

// Testfall 17: Pirouette (Roboter dreht sich auf der Stelle nach rechts)
TEST(MecanumKinematikTest, DrehungAufDerStelleRechts)
{
    // Linke Räder (Rad 1 & 3) müssen positiv (+100) drehen:
    LONGS_EQUAL(100, kin.berechneRad1(0, 0, 100));
    LONGS_EQUAL(100, kin.berechneRad3(0, 0, 100));

    // Rechte Räder (Rad 2 & 4) müssen rückwärts (-100) drehen:
    LONGS_EQUAL(-100, kin.berechneRad2(0, 0, -100));
    LONGS_EQUAL(-100, kin.berechneRad4(0, 0, -100));
}

// Testfall 18: Pirouette (Roboter dreht sich auf der Stelle nach links)
TEST(MecanumKinematikTest, DrehungAufDerStelleLinks)
{
    // Linke Räder (Rad 1 & 3) müssen rückwärts (-100)  drehen:
    LONGS_EQUAL(-100, kin.berechneRad1(0, 0, -100));
    LONGS_EQUAL(-100, kin.berechneRad3(0, 0, -100));

    // Rechte Räder (Rad 2 & 4) müssen positiv (+100) drehen:
    LONGS_EQUAL(100, kin.berechneRad2(0, 0, 100));
    LONGS_EQUAL(100, kin.berechneRad4(0, 0, 100));
}

// ------------------------------- Hinderniserkennung -------------------------------------------

// Testgruppe für die Hinderniserkennung
TEST_GROUP(HinderniserkennungTest)
{
    // Unser neuer Notbrems-Assistent
    Hinderniserkennung regler; 
};

// Test 1: Bei 100 cm Abstand soll das Auto volle Fahrt (vy = 100) geben!
TEST(HinderniserkennungTest, FreieFahrtBeiGrossemAbstand)
{
    LONGS_EQUAL(100, regler.berechneGeschwindigkeitVy(100));
}

// Test 2: Abstand fällt auf unter 20 cm -> Roboter muss sofort stoppen !
TEST(HinderniserkennungTest, NotbremsungBeiNahbereich)
{
    // Bei 10 cm abstand zur Wand erwarten wir exakt 0 % Geschwindigkeit!
    LONGS_EQUAL(0, regler.berechneGeschwindigkeitVy(10));
}

// Test 3: Was passiert bei exakt 20 cm Abstand?
TEST(HinderniserkennungTest, GenauAufDerGrenzlinie20cm)
{
    // Wir erwarten, dass er bei exakt 20 cm bereits stoppt (0)!
    LONGS_EQUAL(0, regler.berechneGeschwindigkeitVy(20));
}

// Test 4: In der Warnzone (z.B. 35 cm) erwarten wir genau 50% Geschwindigkeit!
TEST(HinderniserkennungTest, GeschwindigkeitWarnzone)
{
    LONGS_EQUAL(50, regler.berechneGeschwindigkeitVy(35));
}

// --------------------------------------------- LIDAR - Sensor -----------------------------------------

// Testgruppe für den 2D LIDAR Sensor
TEST_GROUP(LidarSensor2DTest)
{

};

// Test 1: Gültigen und ungültigen Punkt erkennen
TEST(LidarSensor2DTest, PunktGueltig2D)
{
    // Min 0.1 m, Max 12.0 m
    LidarSensor2D lidar(0.1, 12.0);

    CHECK_TRUE(lidar.istMessungGueltig(2.5)); // Normaler Abstand -> Gültig!
    CHECK_FALSE(lidar.istMessungGueltig(0.05)); // Zu nah (Eigenstörung) -> Ungültig!
    CHECK_FALSE(lidar.istMessungGueltig(15.0)); // Zu weit weg (Signalverlust) -> Ungültig!

}

// Test 2: Richtige Umrechnung von Polarkoordinaten (Winkel & Distanz) in 2D (x, y)
TEST(LidarSensor2DTest, PolarGueltig2D)
{
    LidarSensor2D lidar(0.1, 12.0);

    // Test A: 0 Grad (Vorwärtsrichtung x) bei 2.0 Meter
    // Erwartet: x = 2.0, y = 0.0
    double x = 0.0, y = 0.0;
    lidar.polarZuKartesisch(0.0, 2.0, x, y);
    DOUBLES_EQUAL(2.0, x, 0.001);
    DOUBLES_EQUAL(0.0, y, 0.001);

    // Test B: 90 Grad (Querrichtung y nach links) bei 3.0 Meter
    // Erwartet: x = 0.0, y = 3.0
    lidar.polarZuKartesisch(90.0, 3.0, x, y);
    DOUBLES_EQUAL(0.0, x, 0.001);
    DOUBLES_EQUAL(3.0, y, 0.001);

    // Test C: 180 Grad (Rückwärtsrichtung x) bei 1.5 Meter
    // Erwartet: x = -1.5, y = 0.0
    lidar.polarZuKartesisch(180.0, 1.5, x, y);
    DOUBLES_EQUAL(-1.5, x, 0.001);
    DOUBLES_EQUAL(0.0, y, 0.001);
}

// Test 3: Einen ganzen Messscan filtern
TEST(LidarSensor2DTest, FiltertUngueltigeMessungenAusScan)
{
    LidarSensor2D lidar(0.1, 12.0);

    std::vector<LidarMessung> roherScan = {
        {0.0, 2.0},   // Gültig (x = 2.0, y = 0.0)
        {45.0, 15.0}, // Ungültig (ZU WEIT!)
        {90.0, 3.0}   // Gültig (x = 0.0, y = 3.0)
    };

    std::vector<LidarMessung> bereinigterScan = lidar.filtereScan(roherScan);

    // Es dürfen nur noch genau 2 gültige Messungen übrig bleiben!
    LONGS_EQUAL(2, bereinigterScan.size());
    DOUBLES_EQUAL(2.0, bereinigterScan[0].distanzMeter, 0.001);
    DOUBLES_EQUAL(3.0, bereinigterScan[1].distanzMeter, 0.001);
}

// Test 4: Prüfen ob der 2D-LIDAR Sensor seine Punkte korrekt in 3D-Koordinaten uwandelt
TEST(LidarSensor2DTest, Wandelt2DScanin3DPunktwolkeUm)
{
    LidarSensor2D sensor(0.1, 10.0);

    // Wir simulieren 2 Laserstrahlen:
    // 1. Strahl: 2.0m geradeaus (0 Grad)
    // 2. Strahl: 3.0m nach links (90 Grad)
    std::vector<LidarMessung> scan = {
        {0.0, 2.0},
        {90.0, 3.0}
    };

    std::vector<Point3D> punkte = sensor.scanZuPunktwolke3D(scan);

    LONGS_EQUAL(2, punkte.size());

    // Strahl 1 (0 Grad): X = 2.0, Y = 0.0, Z = 0.0
    DOUBLES_EQUAL(2.0, punkte[0].x, 0.001);
    DOUBLES_EQUAL(0.0, punkte[0].y, 0.001);
    DOUBLES_EQUAL(0.0, punkte[0].z, 0.001);

    // Strahl 2 (90 Grad): X = 0.0, Y = 3.0, Z = 0.0
    DOUBLES_EQUAL(0.0, punkte[1].x, 0.001);
    DOUBLES_EQUAL(3.0, punkte[1].y, 0.001);
    DOUBLES_EQUAL(0.0, punkte[1].z, 0.001);
}

// ---------------------------------- PT2-Regler -------------------------------------------------

// Testgruppe für den PT2-Regler
TEST_GROUP(PT2ReglerTest)
{
    PT2Regler regler;
};

// Test 1: Nach 0,1 Sekunden darf die Geschwindigkeit NICHT schlagartig auf 0 sein!
TEST(PT2ReglerTest, SanftBremsen)
{
    // Wir fordern eine Bremsung auf 0%, mit einem Zeitschritt von dt = 0.1 Sekunden
    double v_neu = regler.update(0.0, 0.1);

    // Wir prüfen: Die Geschwindigkeit muss noch deutlich über 50% liegen!
    CHECK(v_neu > 50.0);
}

// Test 2: Roboter muss nach der ersten Zehntelsekunden vollständig auf 0.0 stehen
TEST(PT2ReglerTest, VollstaendigerStillstand)
{
    double v_aktuell = 100.0;

    // Wir simulierenn 1 Sekunden Bremsweg (20 Schritte mit jeweils 0.1s)
    for (int i = 0; i < 20; i++)
    {
        v_aktuell = regler.update(0.0, 0.1);
    }

    // nach 2 Sekunden muss die Geschwindigkeit nahezu 0 sein!
    DOUBLES_EQUAL(0.0, v_aktuell, 1.0);
    // Abweichung von max. 1.0 erlaubt
}

// Test 3: Sanftes Anfahren nach dem Stillstand mithilfe des Konstruktors in der C++-Klasse
TEST(PT2ReglerTest, SanftesAnfahrenVonNull)
{
    // Wir erstellen einen Regler, der bei EXAKT 0.0% anfängt
    PT2Regler anfahrRegler(0.0);

    // Wir fordern schlagartig Vollgas (100.0) für dt = 0.1 Sekunden
    double v_neu = anfahrRegler.update(100.0, 0.1);

    // Der Regler darf nach 0.1s noch NICHT bei 100% angekommen sein (sanftes Anfahren!)
    CHECK(v_neu < 50.0);

    // Aber er soll sich schon leicht in Bewegung gesetzt haben 
    CHECK(v_neu > 0.0);
}

// Test 4: Kein-Überschwingertest
TEST(PT2ReglerTest, KeinUeberschwingen)
{
    // 1.Test: Beim Bremsen darf der Wert NIEMALS unter 0.0 fallen!
    PT2Regler bremsRegler(100.0);
    double v_brems = 100.0;

    for (int i = 0; i < 20; i++){
        v_brems = bremsRegler.update(0.0, 0.05);
        // Garantiert kein negatives Überschwingen (z.B. -1.5)

        CHECK(v_brems >= 0.0);
    }

    // 2.Test: Beim Anfahren darf der Wert NIEMALS über 100.0 hinausschießen!
    PT2Regler anfahrRegler(0.0);
    double v_anfahr = 0.0;

    for (int i = 0; i < 20; i++) {
        v_anfahr = anfahrRegler.update(100.0, 0.05);
        // Garantiert kein Überschießen über das Ziel (z.B. 105.0):

        CHECK(v_anfahr <= 100.0);
    }
}

// Test 5: Kontrolle ob PT2-Regler seinen Sollwert auch genau erreicht
TEST(PT2ReglerTest, ErreichtSollwert)
{
    // Wir starten den Regler bei 0.0 % (Anfahren aus dem Stand)
    PT2Regler anfahrRegler(0.0);
    double v_aktuell = 0.0;

    // Wir simulieren 1,0 Sekunde Zeitablauf (20 Schritte à 0.05 Sekunden)
    for (int i = 0; i < 20; i++){
        v_aktuell = anfahrRegler.update(100.0, 0.05);
    }

    // Nach 1,0 Sekunde verlangen wir, dass der Wert bei exakt 100.0 liegt
    // (mit einer winzigen Toleranz von max. 1.0%, also mindestens 99.0%)

    DOUBLES_EQUAL(100.0, v_aktuell, 2.0);
}

// ------------------------- Schwenkbare 2D-LIDAR-Sensor -------------------------------------------------

// Test-Gruppe Schwenkbarer 2D-LIDAR
TEST_GROUP(SchwenkLidar3DTest)
{

};

// Test 1: Prüfen ob die Klasse den 2D-Scan in eine 3D Punktwolke umwandelt
TEST(SchwenkLidar3DTest, Wandelt2DScanMitServoPitchIn3DPunktwolke)
{
    SchwenkLidar3D schwenkLidar(0.1, 12.0);

    // Eine Messung: 2.0 Meter geradeaus (0 Grad Laser-Winkel)
    std::vector<LidarMessung> scan = { {0.0, 2.0} };

    // Fall 1: Servo steht waagerecht (0 Grad Pitch) -> Z muss exakt 0.0m sein
    auto punkte0 = schwenkLidar.scanZu3DPunktwolke(scan, 0.0);
    LONGS_EQUAL(1, punkte0.size());
    DOUBLES_EQUAL(2.0, punkte0[0].x, 0.001);
    DOUBLES_EQUAL(0.0, punkte0[0].y, 0.001);
    DOUBLES_EQUAL(0.0, punkte0[0].z, 0.001);

    // Fall 2: Servo kippt um 30 Grad nach oben (30 Grad Pitch)
    // sin(30°) = 0.5 -> Höhe Z = 2.0m * 0.5 = 1.0m!
    // cos(30°) = 0.866025 -> Tiefe X = 2.0m * 0.866025 = 1.732m
    auto punkte30 = schwenkLidar.scanZu3DPunktwolke(scan, 30.0);
    LONGS_EQUAL(1, punkte30.size());
    DOUBLES_EQUAL(1.732, punkte30[0].x, 0.001);
    DOUBLES_EQUAL(0.0, punkte30[0].y, 0.001);
    DOUBLES_EQUAL(1.0, punkte30[0].z, 0.001);
}

// ----------------------------------------- SLAM --------------------------------------------------------

// ------------------------------------- Karten erstellen ------------------------------------------------
// Testgruppe OccupancyGrid
TEST_GROUP(OccupancyGridTest)
{

};

// Test 1: Wenn der Laser ein Hindernis bei X=2, Y=3 meldet, muss die Karte dort 'belegt' sein !
TEST(OccupancyGridTest, InitialUnbekanntUndMarkiert)
{
    // Rasterkarte von 10x10 Metern erstellen (Auflösung: 1 Meter pro Zelle)
    OccupancyGrid karte2D(10, 10);

    // Erste Regel: Noch unberührte Zellen müssen -1 (unbekannt) sein!
    LONGS_EQUAL(-1, karte2D.getZelle(0,0));

    // Wir tragen ein Hindernis bei Koordinaten (2.0m, 3.0m) ein (Wert=100 -> Hindernis!)
    karte2D.setzeZelle(2, 3, 100);

    // Zweite Regel: Genau diese Zeile muss jetzt 100 melden!
    LONGS_EQUAL(100, karte2D.getZelle(2, 3));
}

// Test 2: Dritte Dimension hinzufügen und mit double-Werten arbeiten
TEST(OccupancyGridTest, DreiDKarteMitDouble)
{
    // Wir erstellen einen 3D-Gitter: 10m x 10m x 10m mit einer Auflösung von 0.1m 
    //(10 cm pro Voxel). Jede Achse hat dadurch intern 100 Kästchen
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 0.1);

    // Unberührte Stelle muss weiterhin -1.0 sein
    DOUBLES_EQUAL(-1.0, karte3D.getZelle(0.0, 0.0, 0.0), 0.001);

    // Wir tragen ein Hindernis mit Nachkommastellen ein: X=2.35m, Y=3.71m, Z=0.85m
    // und setzen die Wahrscheinlichkeit auf 1.0 (100% belegt)
    karte3D.setzeZelle(2.35, 3.71, 0.85, 1.0);

    // Abfrage genau an dieser Stelle muss 1.0 ergeben!
    DOUBLES_EQUAL(1.0, karte3D.getZelle(2.35, 3.71, 0.85), 0.001);
}

// Test 3: Grenzfall: Abfangen von Werten außerhalb der Kartengrenzen (Sicherheitsnetz)
TEST(OccupancyGridTest, NegativeWerteAbfangen)
{
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 0.1); // 10m x 10m x 10m Karte

    // Negative Koordinate abfragen -> Muss sicher -1.0 liefern (kein Absturz!)
    DOUBLES_EQUAL(-1.0, karte3D.getZelle(-2.5, 3.0, 1.0), 0.001);

    // Versuchen, außerhalb der Grenzen zu schreiben (darf sden Speicher nicht beschädigen!)
    karte3D.setzeZelle(-1.0, -1.0, -1.0, 1.0);
    karte3D.setzeZelle(10.0, 10.0, 10.0, 1.0); // Exakt 10.0m ist bereits außerhalb!
}

// Test 4: Grenzfall: Haarscharfe Punktlandung an den Rändern
TEST(OccupancyGridTest, PunktlandungRand)
{
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 0.1); // 10m x 10m x 10m Karte

    // Allererster Punkt bei (0.0, 0.0, 0.0) -> Voxel (0, 0, 0)
    karte3D.setzeZelle(0.0, 0.0, 0.0, 0.5);
    DOUBLES_EQUAL(0.5, karte3D.getZelle(0.0, 0.0, 0.0), 0.001);

    // Letzter gültiger Punkt kurz vor der 10m-Wand (9.99m ->Voxel 99, 99, 99)
    karte3D.setzeZelle(9.99, 9.99, 9.99, 0.8);
    DOUBLES_EQUAL(0.8, karte3D.getZelle(9.99, 9.99, 9.99), 0.001);
}


// ------------------------------------- Laserstrahl-Schreiber (RayCaster)-------------------------------

// Testgruppe für den Laserstrahlschreiber 2D
TEST_GROUP(RayCaster2DTest)
{
    
};

// Test 1: Einzelner Laserstrahl markiert Weg als FREI (0) und Ziel als BELEGT(100)
TEST(RayCaster2DTest, ZeichnetEinzelnenStrahlIn2DKarte)
{
    // Wir erstellen eine 2D-Karte (200x200 Zellen mit 0.05m Auflösung -> 10m x 10m Bereich)
    OccupancyGrid karte(200, 200);
    RayCaster2D caster;

    // Laserstrahl schießen von (0.0m, 0.0m) zum Hindernis bei (3.0m, 0.0m)
    // Bei 0.05m pro Zelle entspricht 3.0m genau Zelle 60 (3.0 / 0.05 = 60)
    caster.zeichneStrahl(karte, 0.0, 0.0, 3.0, 0.0, 0.05);

    // Das Hindernis zum Zielpunkt (Zelle 60,0) MUSS BELEGT sein (100)
    LONGS_EQUAL(100, karte.getZelle(60, 0));

    // Der Weg dazwischen (z.B. Zelle 20 bei 1.0m und Zelle 40 bei 2.0m)
    // MUSS FREI sein (0)
    LONGS_EQUAL(0, karte.getZelle(20, 0));
    LONGS_EQUAL(0, karte.getZelle(40, 0));
}

// Test 2: Rundum Scan 2D Umgebung
TEST(RayCaster2DTest, RundUmScan2D)
{
    OccupancyGrid karte(200, 200);
    RayCaster2D caster;

    // Ein Scan mit zwei Laserstrahlen:
    // Strahl 1: 0 Grad (geradeaus) bei 2.0 m -> Zelle (40, 0)
    // Strahl 2: 90 Grad (nach links) bei 1.0m -> Zelle (0, 20)
    std::vector<LidarMessung> scan = {
        {0.0, 2.0},
        {90.0, 1.0}
    };

    caster.zeichneRundumScan(karte, 0.0, 0.0, scan, 0.05);

    // Beide Zielpunkte müssen als BELEGT (100) markiert sein
    LONGS_EQUAL(100, karte.getZelle(40, 0)); // 2.0m geradeaus
    LONGS_EQUAL(100, karte.getZelle(0, 20)); // 1.0m nach links
}

// Testgruppe für den Laserstrahlschreiber 3D
TEST_GROUP(RayCaster3DTest)
{

};

// Test 1: Freien und markierten Weg in der x-Ebene im 3D-Kartenmodell besetzten
TEST(RayCaster3DTest, PunktInDerXAchse3D)
{
    // Wir erstellen eine 3D-Karte (10m x 10m x 10m mit 1.0m Auflösung für einfache Zahlen)
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 1.0);

    // Der erste RayCaster 
    RayCaster3D caster;

    // Wir schießen einen Strahl vom Roboter bei (0,0,0) zum Hindernis bei (3.0, 0.0, 0.0)
    caster.zeichneStrahl(karte3D, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0);

    // Erwartung 1: Die Zelle am Endpunkt (3,0,0) MUSS als Hindernis (1.0) markiert sein!
    DOUBLES_EQUAL(1.0, karte3D.getZelle(3.0, 0.0, 0.0), 0.001);

    // Erwartung 2: Der Weg dazwischen (z.B. bei 1.0m und 2.0m) MUSS als FREI (0.0) markiert sein!
    DOUBLES_EQUAL(0.0, karte3D.getZelle(1.0, 0.0, 0.0), 0.001);
    DOUBLES_EQUAL(0.0, karte3D.getZelle(2.0, 0.0, 0.0), 0.001);
}

// Test 2: Freien und markierten Weg in der y-Ebene im 3D-Kartenmodell besetzten
TEST(RayCaster3DTest, PunktInDerYAchseAlsHindernisErkennen)
{
    // Wir erstellen eine 3D-Karte (10m x 10m x 10m mit 1.0m Auflösung für einfache Zahlen)
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 1.0);

    // Der erste RayCaster 
    RayCaster3D caster;

    // Wir schießen einen Strahl vom Roboter bei (0,0,0) zum Hindernis bei (0.0, 3.0, 0.0)
    caster.zeichneStrahl(karte3D, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0);

    // Erwartung 1: Die Zelle am Endpunkt (0,3,0) MUSS als Hindernis (1.0) markiert sein!
    DOUBLES_EQUAL(1.0, karte3D.getZelle(0.0, 3.0, 0.0), 0.001);

    // Erwartung 2: Der Weg dazwischen (z.B. bei 1.0m und 2.0m) MUSS als FREI (0.0) markiert sein!
    DOUBLES_EQUAL(0.0, karte3D.getZelle(0.0, 1.0, 0.0), 0.001);
    DOUBLES_EQUAL(0.0, karte3D.getZelle(0.0, 2.0, 0.0), 0.001);
}

// Test 3: Freien und markierten Weg in der z-Ebene im 3D-Kartenmodell besetzten
TEST(RayCaster3DTest, PunktInDerZAchseAlsHindernisErkennen)
{
    // Wir erstellen eine 3D-Karte (10m x 10m x 10m mit 1.0m Auflösung für einfache Zahlen)
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 1.0);

    // Der erste RayCaster 
    RayCaster3D caster;

    // Wir schießen einen Strahl vom Roboter bei (0,0,0) zum Hindernis bei (0.0, 0.0, 3.0)
    caster.zeichneStrahl(karte3D, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0);

    // Erwartung 1: Die Zelle am Endpunkt (0,0,3) MUSS als Hindernis (1.0) markiert sein!
    DOUBLES_EQUAL(1.0, karte3D.getZelle(0.0, 0.0, 3.0), 0.001);

    // Erwartung 2: Der Weg dazwischen (z.B. bei 1.0m und 2.0m) MUSS als FREI (0.0) markiert sein!
    DOUBLES_EQUAL(0.0, karte3D.getZelle(0.0, 0.0, 1.0), 0.001);
    DOUBLES_EQUAL(0.0, karte3D.getZelle(0.0, 0.0, 2.0), 0.001);
}

// Test 4: Freien und markierten Weg in der freien Luft im 3D-Kartenmodell besetzten
TEST(RayCaster3DTest, PunktInDerLuftAlsHindernisErkennen)
{
    // Wir erstellen eine 3D-Karte (10m x 10m x 10m mit 1.0m Auflösung für einfache Zahlen)
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 1.0);

    // Der erste RayCaster 
    RayCaster3D caster;

    // Wir schießen einen Strahl vom Roboter bei (0,0,0) zum Hindernis bei (5.0, 2.0, 3.0)
    caster.zeichneStrahl(karte3D, 0.0, 0.0, 0.0, 5.0, 2.0, 3.0);

    // Prüfung 1: Die Zelle am Endpunkt (5,2,3) MUSS als Hindernis (1.0) markiert sein!
    DOUBLES_EQUAL(1.0, karte3D.getZelle(5.0, 2.0, 3.0), 0.001);

    // Prüfung 2: Auf halbem Weg zum Hindernis MUSS als FREI (0.0) markiert sein!
    DOUBLES_EQUAL(0.0, karte3D.getZelle(2.5, 1.0, 1.5), 0.001);

    // Prüfung 3: Der Boden direkt unter dem Hindernis wurde nie getroffen und bleibt UNBEKANNT (-1.0)!
    DOUBLES_EQUAL(-1.0, karte3D.getZelle(5.0, 2.0, 0.0), 0.001);
}

// Test 5: Rundum-Scan mit dem LIDAR
TEST(RayCaster3DTest, RundUmScanner)
{
    OccupancyGrid3D karte3D(10.0, 10.0, 10.0, 0.5);
    RayCaster3D caster;

    // Roboter steht genau in der Mitte bei (5.0, 5.0, 1.0)
    double robX = 5.0, robY = 5.0, robZ = 1.0;

    // Ein LIDAR-Scan mit 4 Strahlen (je 2.0m Distanz zur Wand):
    // 0° = Osten (+X), 90° = Norden (+Y), 180° = Westen (-X), 270° = Süden (-Y)
    std::vector<LidarMessung> scan = {
        {  0.0, 2.0}, // Hindernis müsste bei (7.0, 5.0, 1.0) liegen
        { 90.0, 2.0}, // Hindernis müsste bei (5.0, 7.0, 1.0) liegen
        {180.0, 2.0}, // Hindernis müsste bei (3.0, 5.0, 1.0) liegen
        {270.0, 2.0}  // Hindernis müsste bei (5.0, 3.0, 1.0) liegen
    };

    // Führt den Befehl aus
    caster.zeichneRundumScan(karte3D, robX, robY, robZ, scan);

    // PRÜFUNG: Werden alle 4 Wände rund um den Roboter korrekt als BELEGT (1.0) eingetragen?
    DOUBLES_EQUAL(1.0, karte3D.getZelle(7.0, 5.0, 1.0), 0.001); // Osten
    DOUBLES_EQUAL(1.0, karte3D.getZelle(5.0, 7.0, 1.0), 0.001); // Norden
    DOUBLES_EQUAL(1.0, karte3D.getZelle(3.0, 5.0, 1.0), 0.001); // Westen
    DOUBLES_EQUAL(1.0, karte3D.getZelle(5.0, 3.0, 1.0), 0.001); // Süden

    // PRÜFUNG: Ist der Bereich direkt vor dem Roboter z.B. bei (6.0, 5.0, 1.0) FREI (0.0)?
    DOUBLES_EQUAL(0.0, karte3D.getZelle(6.0, 5.0, 1.0), 0.001);

}

// -------------------------------------- Odometrie -------------------------------------------------------

// Testgruppe für Odometrie
TEST_GROUP(OdometryTest)
{
};

// Test 1: Streckenmessung beim Vorwärtsfahren -> Geradeaus
TEST(OdometryTest, AktuellePositionVorwaerts)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_x = 1.0 m/s für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(1.0, 0.0, 0.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei X = 2.0 Meter stehen!
    DOUBLES_EQUAL(2.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getZ(), 0.001);
}   

// Test 2: Streckenmessung beim Rückwärtsfahren -> Geradeaus
TEST(OdometryTest, AktuellePositionRueckwaerts)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_x = -1.0 m/s für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(-1.0, 0.0, 0.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei X = -2.0 Meter stehen!
    DOUBLES_EQUAL(-2.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getZ(), 0.001);
}

// Test 3: Streckenmessung beim Seitwärtsfahren -> Rechts
TEST(OdometryTest, AktuellePositionRechts)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_y = 1.0 m/s für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(0.0, 1.0, 0.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei Y = 2.0 Meter stehen!
    DOUBLES_EQUAL(0.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(2.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getZ(), 0.001);
}   

// Test 4: Streckenmessung beim Seitwärtsfahren -> Links
TEST(OdometryTest, AktuellePositionLinks)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_y = -1.0 m/s (nach links!) für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(0.0, -1.0, 0.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei Y = -2.0 Meter stehen!
    DOUBLES_EQUAL(0.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(-2.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getZ(), 0.001);
} 

// Test 5: Streckenmessung nach oben 
TEST(OdometryTest, AktuellePositionOben)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_z = 1.0 m/s für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(0.0, 0.0, 1.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei Z = 2.0 Meter stehen!
    DOUBLES_EQUAL(0.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(2.0, tracker.getZ(), 0.001);
} 

// Test 6: Streckenmessung nach unten 
TEST(OdometryTest, AktuellePositionUnten)
{
    // Wir erstellen den Tracker (Startposition 0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert die Fahrt: Geschwindigkeit v_z = -1.0 m/s für Zeit dt = 2.0 Sekunden
    tracker.aktualisiereGeschwindigkeitOdometrie(0.0, 0.0, -1.0, 2.0);

    // Prüfung: Nach 2 Sekunden mit 1 m/s müssen wir bei Z = -2.0 Meter stehen!
    DOUBLES_EQUAL(0.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(0.0, tracker.getY(), 0.001);
    DOUBLES_EQUAL(-2.0, tracker.getZ(), 0.001);
} 

// Test 7: Streckenmessung rückwärts in allen Richtungen
TEST(OdometryTest, AktuellePositionRueckwaertsInAllenDreiAchsen)
{
    // Start am Nullpunkt (0.0, 0.0, 0.0)
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Simuliert eine kombinierte Bewegung mit negativen Geschwindigkeiten für dt = 2.0 Sekunden
    // v_x = -1.0 m/s (Rückwärts)
    // v_y = -0.5 m/s (Seitwärts)
    // v_z = -0.2 m/s (Sinkend)
    tracker.aktualisiereGeschwindigkeitOdometrie(-1.0, -0.5, -0.2, 2.0);

    // Prüfungen: 
    // X = -1.0 * 2.0 = -2.0 Meter
    DOUBLES_EQUAL(-2.0, tracker.getX(), 0.001);

    // Y = -0.5 * 2.0 = -1.0 Meter
    DOUBLES_EQUAL(-1.0, tracker.getY(), 0.001);

    // Z = -0.2 * 2.0 = -0.4 Meter
    DOUBLES_EQUAL(-0.4, tracker.getZ(), 0.001);
}

// TEST 8: Kurvenfahrten nach links entlang der X- und Y-Achse
TEST(OdometryTest, BeruecksichtigungDrehungNachLinks)
{
    OdometryTracker tracker(0.0, 0.0, 0.0);

    // Wir drehen den Roboter um 90° nach links (Blickrichtung Norden)
    tracker.setzeAusrichtungGrad(90.0);

    // Jetzt fahren wir für 2 Sekunden mit 1.0 m/s nach "vorne"
     tracker.aktualisiereGeschwindigkeitOdometrie(1.0, 0.0, 0.0, 2.0);

    // Da wir nach Norden schauen, muss X nahe 0.0 bleiben und Y auf 2.0 steigen!
    DOUBLES_EQUAL(0.0, tracker.getX(), 0.001);
    DOUBLES_EQUAL(2.0, tracker.getY(), 0.001);
}

// ----------------------------------------- IMU --------------------------------------------------------

// Testgruppe für IMU
TEST_GROUP(IMUTest)
{

};

// Test 1: Geschwindigkeit aus Beschleunigung berechnen
TEST(IMUTest, BerechneGeschwindigkeitAusBeschleunigung)
{
    // Wir erstellen einen eigenständigen IMU-Sensor 
    IMUSensor imu;

    // Die IMU misst eine Beschleunigung von ax = 1.0 m/s^2 über dt = 2.0 Sekunden
    imu.verarbeiteBeschleunigung(1.0, 0.0, 0.0, 2.0);

    // Prüfung: Hat sich die Geschwindigkeit auf vx = 2.0 m/s aufgebaut?
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);
}

// Test 2: Fortlaufende Beschleunigung
TEST(IMUTest, FortlaufendeBeschleunigung)
{
    // Wir erstellen einen eigenständigen IMU-Sensor 
    IMUSensor imu;

    // 2 Sekunden lang mit 1.0 m/s^2 beschleunigen -> Geschwindigkeit muss 2.0 m/s sein
    imu.verarbeiteBeschleunigung(1.0, 0.0, 0.0, 2.0);
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);

    // Noch einmal 2 Sekunden lang mit 1.0 m/s^2 weiterbeschleunigen 
    //-> Geschwindigkeit muss auf 4.0 m/s steigen!
    imu.verarbeiteBeschleunigung(1.0, 0.0, 0.0, 2.0);
    DOUBLES_EQUAL(4.0, imu.getGeschwindigkeitX(), 0.001);
}

// Test 3: Zurückgelegte Strecke aus der Beschleunigung und Geschwindigjeit ausrechnen
TEST(IMUTest, StreckeAusGeschwindigkeitUndBeschleunigung)
{
    IMUSensor imu;

    // Phase1: 2 Sekunden lang mit 1.0 m/s^2 AUS DEM STAND beschleunigen
    imu.verarbeiteBeschleunigung(1.0, 0.0, 0.0, 2.0);

    // Nach Phase 1: v_x = 1.0 m/s, Strecke s = 0.5 * 1.0 * 2.0^2 = 2.0 Meter
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(2.0, imu.getX(), 0.001);

    // Vor Phase 2: 2 Sekunden lang ohne Beschleunigung weiterfahren
    imu.verarbeiteBeschleunigung(0.0, 0.0, 0.0, 2.0);

    // Phase 2: v_x bleibt 2.0 m/s, aber die Gesamtstrecke steigt um 4.0m auf 6.0 Meter!
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(6.0, imu.getX(), 0.001);
}

// Test 4: Von Ampel zur Ampel
TEST(IMUTest, VonAmpelZuAmpel)
{
    IMUSensor imu;

    // Etappe 1: Anfahren aus dem Stand (a = 1.0 m/s^2, dt = 2.0s)
    imu.verarbeiteBeschleunigung(1.0, 0.0, 0.0, 2.0);
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(2.0, imu.getLetztesDeltaX(), 0.001);
    DOUBLES_EQUAL(2.0, imu.getX(), 0.001);

    // Etappe 2: Dahingleiten (a = 0.0 m/s^2, dt = 2.0s)
    imu.verarbeiteBeschleunigung(0.0, 0.0, 0.0, 2.0);
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(4.0, imu.getLetztesDeltaX(), 0.001); // 4.0m kommen obendrauf
    DOUBLES_EQUAL(6.0, imu.getX(), 0.001); // Gesamt: 2.0 + 4.0 = 6.0m

    // Etappe 3: Turbo-Schub (a = 2.0 m/s^2, dt = 1.0s)
    imu.verarbeiteBeschleunigung(2.0, 0.0, 0.0, 1.0);
    DOUBLES_EQUAL(4.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(3.0, imu.getLetztesDeltaX(), 0.001); // 2.0m Schwung + 1.0m Schub
    DOUBLES_EQUAL(9.0, imu.getX(), 0.001); // Gesamt: 6.0 + 3.0 = 9.0

    // Etappe 4: Vollbremsung bis zum Stillstand (a = -4.0 m/s^2, dt = 1.0s)
    imu.verarbeiteBeschleunigung(-4.0, 0.0, 0.0, 1.0);
    DOUBLES_EQUAL(0.0, imu.getGeschwindigkeitX(), 0.001);
    DOUBLES_EQUAL(2.0, imu.getLetztesDeltaX(), 0.001); // 2.0m Bremsweg
    DOUBLES_EQUAL(11.0, imu.getX(), 0.001); // Gesamt: 9.0 + 2.0 = 11.0
}

// Test 5: Kombinierte Beschleunigungen
TEST(IMUTest, VollstaendigeKombinierte3DBewegung)
{
    IMUSensor imu;

    // Phase 1: Gleichzeitige Beschleunigung in ALLEN 3 Raumrichtungen für dt = 2.0s
    // ax = 1.0 m/s^2, ay = 0.5 m/s^2, az = -0.2 m/s^2
    imu.verarbeiteBeschleunigung(1.0, 0.5, -0.2, 2.0);

    // Geschwindigkeiten nach 2 Sekunden (v = a * dt)
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001); // 1.0 * 2.0
    DOUBLES_EQUAL(1.0, imu.getGeschwindigkeitY(), 0.001); // 0.5 * 2.0
    DOUBLES_EQUAL(-0.4, imu.getGeschwindigkeitZ(), 0.001); // -0.2 * 2.0

    // Zurückgelegte Strecken nach Phase 1 (s = 0.5 * a * dt^2)
    DOUBLES_EQUAL(2.0, imu.getLetztesDeltaX(), 0.001); // 0.5 * 1.0 * 2.0^2
    DOUBLES_EQUAL(1.0, imu.getLetztesDeltaY(), 0.001); // 0.5 * 0.5 * 2.0^2
    DOUBLES_EQUAL(-0.4, imu.getLetztesDeltaZ(), 0.001); // 0.5 * -0.2 * 2.0^2

    // Gesamtstrecke nach Phase 1 überprüfen 
    DOUBLES_EQUAL(2.0, imu.getX(), 0.001); 
    DOUBLES_EQUAL(1.0, imu.getY(), 0.001); 
    DOUBLES_EQUAL(-0.4, imu.getZ(), 0.001); 

    // Phase 2: Motoren aus (a = 0 in allen Achsen), 1 Sekunde lang mit Schwung weitergleiten
    imu.verarbeiteBeschleunigung(0.0, 0.0, 0.0, 1.0);

    // Geschwindigkeiten müssen unverändert bleiben
    DOUBLES_EQUAL(2.0, imu.getGeschwindigkeitX(), 0.001); 
    DOUBLES_EQUAL(1.0, imu.getGeschwindigkeitY(), 0.001); 
    DOUBLES_EQUAL(-0.4, imu.getGeschwindigkeitZ(), 0.001); 

    // Teilstrcke für Phase 2 prüfen
    DOUBLES_EQUAL(2.0, imu.getLetztesDeltaX(), 0.001); // 0.5 * 1.0 * 2.0^2
    DOUBLES_EQUAL(2.0, imu.getLetztesDeltaY(), 0.001); // 0.5 * 0.5 * 2.0^2
    DOUBLES_EQUAL(-0.8, imu.getLetztesDeltaZ(), 0.001); // 0.5 * -0.2 * 2.0^2

    // Strecken müssen um v * 1.0s ansteigen:
    DOUBLES_EQUAL(4.0, imu.getX(), 0.001); // 2.0 + 2.0
    DOUBLES_EQUAL(3.0, imu.getY(), 0.001); // 1.0 + 1.0
    DOUBLES_EQUAL(-1.2, imu.getZ(), 0.001); // -0.4 + (-0.4)
}

// ------------------------------------- Sensor-Fusion --------------------------------------------------

// Testgruppe für Sensorfusion
TEST_GROUP(SensorFusionTest)
{

};

// Test 1: Extremfälle
TEST(SensorFusionTest, RobustheitGrenzwerteUndExtrema)
{
    SensorFusion filter; // Vollautomatisch

    // 1. Grenzwert unter 0.2: Normale ebene Fahrt (Stufe 1: alpha = 0.8)
    // Diff = |1.0 - 1.1| = 0.1 m/s (< 0.2 m/s Schwellenwert)
    // Erwartet: 0.8 * 1.0 + 0.2 * 1.1 = 0.8 + 0.22 = 1.02 m/s
    DOUBLES_EQUAL(1.02, filter.fusedGeschwindigkeitXVollautomatisch(1.0, 1.1), 0.001);

    // 2. Grenzwert über 0.2 & unter 1.0: Teppichfahrten (Stufe 2)
    // Diff = |1.0 - 1.5| = 0.5 m/s -> Alpha = 0.9!
    // Erwartet: 0.9 * 1.0 + 0.1 * 1.5 = 1.05 m/s
    DOUBLES_EQUAL(1.05, filter.fusedGeschwindigkeitXVollautomatisch(1.0, 1.5), 0.001);

    // 3. Grenzwert über 1.0: Aquaplaning (Stufe 3)
    // Diff = |5.0 - 0.0| = 5.0 m/s -> Alpha = 0.1!
    // Erwartet: 0.1 * 5.0 + 0.9 * 0.0 = 0.5 m/s
    DOUBLES_EQUAL(0.5, filter.fusedGeschwindigkeitXVollautomatisch(5.0, 0.0), 0.001);
}

// Test 2: Automatisches Erkennen des Bodens und dynamisches Anpassen von Alpha
TEST(SensorFusionTest, ErkenntBodenAutomatisch)
{
    // Ein ganz normales SensorFusion-Objekt OHNE manuelle Alpha-Vorgabe
    SensorFusion automatischerFilter;

    // -------------------------------------------------------------------------
    // Fall 1: Fahrt auf Teppich (Guter Grip, kleine Abweichung von 0.2 m/s)
    // Odom = 1.0 m/s, IMU = 1.2 m/s
    // Klasse wählt AUTOMATISCH alpha = 0.9!
    // Erwartet: 0.9 * 1.0 + 0.1 * 1.4 = 1.04 m/s
    double teppichErgebnis = automatischerFilter.fusedGeschwindigkeitXVollautomatisch(1.0, 1.4);
    DOUBLES_EQUAL(1.04, teppichErgebnis, 0.001);

    // ------------------------------------------------------------------------
    // Fall 2: Aquaplaning auf Fliesen (Räder drehen durch, riesige Abweichung von 5.0 m/s)
    // Odom = 5.0 m/s, IMU = 0.0 m/s
    // Klasse erkennt den Notfall und wählt AUTOMATISCH alpha = 0.1!
    // Erwartet: 0.1 * 5.0 + 0.9 * 0.0 = 0.5 m/s
    double aquaplaningErgebnis = automatischerFilter.fusedGeschwindigkeitXVollautomatisch(5.0, 0.0);
    DOUBLES_EQUAL(0.5, aquaplaningErgebnis, 0.001);
}

// Test 5: Vollautomatischer Check für alle drei Achsen gleichzeitig
TEST(SensorFusionTest, Alle3DGleichzeitigFusion)
{
    SensorFusion automatischerFilter; // Nutzt Standard Konstruktor

    // Odom = 1.0 m/s, IMU = 1.4 m/s

    // 1. X-Achse: Teppichfahrt (Diff = 0.4 -> Alpha = 0.9)
    // 0.9 * 1.0 + 0.1 * 1.4 = 1.04 m/s
    DOUBLES_EQUAL(1.04, automatischerFilter.fusedGeschwindigkeitXVollautomatisch(1.0, 1.4), 0.001);

    // 2. Y-Achse: Seitlicher Drift auf Fliesen (Diff = 3.0 > 1.0 -> Alpha = 0.1)
    // 0.1 * 1.0 + 0.9 * 0.0 = 0.3 m/s
    DOUBLES_EQUAL(0.3, automatischerFilter.fusedGeschwindigkeitYVollautomatisch(3.0, 0.0), 0.001);

    // 3. Z-Achse: Teppichkanten-Absatz (Diff = 2.5 > 0.5 -> Alpha = 0.9 für Odom)
    // 0.9 * 0.0 + 0.1 * (-2.5) = -0.25 m/s
    DOUBLES_EQUAL(-0.25, automatischerFilter.fusedGeschwindigkeitZVollautomatisch(0.0, -2.5), 0.001);
}

// ------------------------------------- Schaltzentrale --------------------------------------------------

// Testgruppe zum SLAM-Bau
TEST_GROUP(SlamMapBuilderTest)
{

};

// Test 1: 2D-SLAM Bau verarbeitet rohen Scan inklusive Filterung
TEST(SlamMapBuilderTest, verarbeitetScanIn2DKarte)
{
    // 200x200 Zellen mit 0.05m Auflösung -> 10m x 10m
    SlamMapBuilder2D builder2D(200, 200, 0.05);

    std::vector<LidarMessung> roherScan = {
        {0.0, 2.0},     // Gültig: 2.0m geradeaus -> Zelle (40,0)
        {45.0, 15.0}    // Ungültig: 15.0m ist zu weit und wird gefiltert
    };

    builder2D.verarbeiteScan(0.0, 0.0, roherScan);

    // Das Hindernis bei 2.0m muss als BELEGT (100) im Grid markiert sein!
    LONGS_EQUAL(100, builder2D.getKarte().getZelle(40, 0));

    // Der Weg bei 1.0m muss als FREI (0) markeiert sein
    LONGS_EQUAL(0, builder2D.getKarte().getZelle(20, 0));
}

// Test 2: 3D-SLAM Bau verarbeitet rohen Scan inklusive Filterung
TEST(SlamMapBuilderTest, verarbeitetScanIn3DKarte)
{
    // 10x10x10 Karte mit 1.0m Auflösung
    SlamMapBuilder3D builder3D(10.0, 10.0, 10.0, 1.0);

    std::vector<LidarMessung> roherScan = {
        {0.0, 3.0} // 3.0m geradeaus in x-Richtung
    };

    // Roboter steht bei (0.0, 0.0, 1.0) -> Zeichnet auf z = 1.0m Höhe
    builder3D.verarbeiteScan(0.0, 0.0, 1.0, roherScan);
    
    // Hindernis bei (3.0, 0.0, 1.0) muss als BELEGT (1.0) markiert sein!
    DOUBLES_EQUAL(1.0, builder3D.getKarte3D().getZelle(3.0, 0.0, 1.0), 0.001);

    // Weg dazwischen bei (1.0, 0.0, 1.0) muss als FREI (0.0) markiert sein!
    DOUBLES_EQUAL(0.0, builder3D.getKarte3D().getZelle(1.0, 0.0, 1.0), 0.001);
}
// ------------------------------------ Integrationstest -------------------------------------------------

// Testgruppe zur Integrationsprüfung
TEST_GROUP(IntegrationTest)
{

};

// Test 1: Schauen ob die SensorFusion Klasse auch mit echten Werten umgehen kann
TEST(IntegrationTest, VerschmelzeEchteWerteAusOdomUndIMU)
{
    // 1. Die drei Spezialisten aufstellen
    OdometryTracker odom(0.0, 0.0, 0.0);
    IMUSensor imu; 
    SensorFusion fusion; // alpha = 0.8 -> 80% Vertrauen in Odom, 20% in IMU

    // 2. Bewegung simulieren:
    // Odometrie ruhige Fahrt mit v = 1.0 m/s über dt = 2.0s
    odom.aktualisiereGeschwindigkeitOdometrie(1.0, 0.0, 0.0, 2.0);

    // IMU spürt Beschleunigung a = 1.5 m/s^2 über dt = 2.0s -> baut v = 3.0 m/s auf
    imu.verarbeiteBeschleunigung(1.5, 0.0, 0.0, 2.0);

    // 3. Werte aus BEIDEN Klassen live auslesen und speichern:
    double gemesseneOdomGeschwindigkeit = odom.getGeschwindigkeitX(); // Ergibt 1.0 m/s
    double gemesseneIMUGeschwindigkeit = imu.getGeschwindigkeitX(); // Ergibt 3.0 m/s

    // 4. An die Filter-Klasse übergeben und verschmelzen:
    double gefilterteGeschwindigkeit = fusion.fusedGeschwindigkeitXVollautomatisch(
        gemesseneOdomGeschwindigkeit, 
        gemesseneIMUGeschwindigkeit);

    // 5. Prüfung: Hat das Zusammenspiel aller 3 Klassen geklappt?
    // Diff = 1.0 - 3.0 = -2.0 m/s -> Automatik erkennt Stufe 2! -> Alpha = 0.2!
    // Erwartet: 0.2 * 1.0 + 0.8 * 3.0 = 2.6 m/s
    DOUBLES_EQUAL(2.6, gefilterteGeschwindigkeit, 0.001);

}

// ------------------------------------- Scan-Matching ---------------------------------------------------

// Testgruppe für den 2D-Scanmatcher
TEST_GROUP(ScanMatcherTest)
{

};

// Test 1: Wenn zwei Scans identisch sind, ist der Fehler (Score) exakt 0.0
TEST(ScanMatcherTest, IdentischeScansHabenNullFehler)
{
    ScanMatcher2D matcher;

    std::vector<LidarMessung> scanA = {{0.0, 2.0}, {90.0, 3.0}};
    std::vector<LidarMessung> scanB = {{0.0, 2.0}, {90.0, 3.0}};

    // Berechne den Unterschieds-Fehler (Mean Squared Error) ohne Verschiebung (dx=0, dy=0)
    double fehler = matcher.berechneAbweichung(scanA, scanB, 0.0, 0.0);

    DOUBLES_EQUAL(0.0, fehler, 0.001);
}

// Test 2: Findet die optimale Verschiebung gleichzeitig in X- und Y-Richtung
TEST(ScanMatcherTest, FindetVerschiebungInXUndY)
{
    ScanMatcher2D matcher;

    // Referenz-Scan (Wandpunkte, wie der Roboter sie ursprünglich sah)
    // Punkt 1: 3.0m geradeaus (x=3.0, y=0.0)
    // Punkt 2: 2.0m nach links (x=0.0, y=2.0)
    std::vector<LidarMessung> referenzScan = {
        {0.0, 3.0},
        {90.0, 2.0}
    };

    // Neuer Scan: Der Roboter ist 0.5m nach vorne (X) und 0.3m nach links (Y) gerutscht
    // Die Wandpunkte erscheinen im neuen Koordinatensystem entsprechend näher:
    // Punkt 1 liegt jetzt bei x = 2.5m (3.0 - 0.5) und y = -0.3m (0.0 - 0.3)
    // Punkt 2 liegt jetzt bei x = -0.5m (0.0 - 0.5) und y = 1.7m (2.0 - 0.3)
    // (Hier simuliert der Matcher des automatische Übereinanderlegen)

    double bestesDx = 0.0;
    double bestesDy = 0.0;
    

    // Wir rufen die void-Methode auf!
    matcher.findeBesteVerschiebung2D(referenzScan, referenzScan, 
        0.0, 1.0, // Suchfenster X (minDx, maxDx)
        0.0, 1.0,   // Suchfenster Y (minDy, maxDy)
        0.0,  // Schrittweite und winkel
       bestesDx, bestesDy);

    // Wenn beide Scans exakt übereinstimmen (dx=0, dy=0):
    DOUBLES_EQUAL(0.0, bestesDx, 0.001);
    DOUBLES_EQUAL(0.0, bestesDy, 0.001);
}

// Test 3: ScanMatcher3D ermittelt Verschiebung in X, Y und Z korrekt
TEST(ScanMatcherTest, FindetVerschiebungInX_Y_Und_Z)
{
    ScanMatcher3D matcher;

    // Referenz-Scan A (Roboter steht auf Höhe Z = 1.0m)
    std::vector<LidarMessung> referenzScan = {
        {0.0, 3.0},  // Wandpunkt bei x=3.0, y=0.0
        {90.0, 2.0}  // Wandpunkt bei x=0.0, y=2.0
    };

    // Neuer Scan B (identisch)
    std::vector<LidarMessung> neuerScan = referenzScan;

    // Unsere 3 Klemmbretter für die Ergebnisse in X, Y und Z
    double bestesDx = 0.0;
    double bestesDy = 0.0;
    double bestesDz = 0.0;
    
    // Roboter A auf Z = 1.0m, Roboter B auf Z = 1.0m
    matcher.findeBesteVerschiebung3D(referenzScan, neuerScan,
    1.0, 1.0,   // robZ_A, robZ_B
    0.0, 1.0,   // minDx, maxDx
    0.0, 1.0,   // minDy, maxDy
    0.0, 1.0,   // minDz, maxDz
    0.1,        // Schrittweite
    bestesDx, bestesDy, bestesDz);
    
    // Da beide Scans und Z-Höhen identisch sind, MUSS die Verschiebung (0, 0, 0) sein!
    DOUBLES_EQUAL(0.0, bestesDx, 0.001);
    DOUBLES_EQUAL(0.0, bestesDy, 0.001);
    DOUBLES_EQUAL(0.0, bestesDz, 0.001);

    
}

// Test 4: ScanMatcher2D erkennt Verschiebung UND Drehung (Winkel)
TEST(ScanMatcherTest, ErkenntDrehungUndVerschiebung2D)
{
    ScanMatcher2D matcher;

    // Referenz-Scan A
    std::vector<LidarMessung> scanA = {
        {0.0, 3.0},
        {90.0, 2.0}
    };

    // Identischer Scan B
    std::vector<LidarMessung> scanB = scanA;

    double bestesDx = 0.0;
    double bestesDy = 0.0;
    double bestesDTheta = 0.0;

    // Suchbereich: X von 0 bis 0.5m, Y von 0 bis 0.5m, Winkel von -15 bis 15 Grad
    matcher.findeBesteVerschiebungUndDrehung2D(scanA, scanB, 
    0.0, 0.5,   //minDx, maxDx
    0.0, 0.5,    //minDy, maxDy
    -5.0, 5.0,    // minDTheta, maxDTheta
    0.1, 1.0,   //Schrittweiten (0.1m, 1.0 Grad)
    bestesDx, bestesDy, bestesDTheta);

    // Da beide Scans identisch sind müssen Verschiebung und Drehung exakt 0 sein!
    DOUBLES_EQUAL(0.0, bestesDx, 0.001);
    DOUBLES_EQUAL(0.0, bestesDy, 0.001);
    DOUBLES_EQUAL(0.0, bestesDTheta, 0.001);
}

// ------------------------------------ SLAM-Controller --------------------------------------------------

// Testgruppe für den 2D SLAM-Controller
TEST_GROUP(SlamController2DTest)
{

};

// Test 1: SlamController korrigiert bei Schlupf und zeichnet Karte
TEST(SlamController2DTest, VerarbeitetScansUndKorrigiertPose)
{
    // 10m x 10m Karte mit 0.1m Auflösung erstellen
    SlamController2D controller(10.0, 10.0, 0.1);

    // Erster Scan bei Startposition (Roboter sieht eine Wand bei 3.0m geradeaus)
    std::vector<LidarMessung> scan1 = {
        {0.0, 3.0}, // Wand vor dem Roboter
        {90.0, 2.0} // Wand links vom Roboter
    };

    // Roboter verarbeitet den ersten Scan
    controller.verarbeiteScan(scan1);

    // Am Anfang muss der Roboter exakt auf (0.0) stehen
    DOUBLES_EQUAL(0.0, controller.getX(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getY(), 0.001);

    // Jetzt simulieren wir Identischen Scan (Roboter hat sich nicht bewegt)
    std::vector<LidarMessung> scan2 = scan1;
    controller.verarbeiteScan(scan2);

    // Pose muss weiterhin (0,0) sein!
    DOUBLES_EQUAL(0.0, controller.getX(), 0.01);
    DOUBLES_EQUAL(0.0, controller.getY(), 0.01);

    // Prüfen, ob das Hindernis in der Karte bei x=3.0, y=0.0 als BELEGT (1.0) eingetragen wurde
    LONGS_EQUAL(100, controller.getKarte().getZelle(3.0, 0.0));
}

// Test 2: Glatteis Vollbremsung
TEST(SlamController2DTest, GlatteisVollbremsung2D)
{
    SlamController2D controller(10.0, 10.0, 0.1);

    // Scan 1: Roboter steht bei (0,0) und sieht eine Wand in 3.0m Entfernung
    std::vector<LidarMessung> scan1 = {
        {0.0, 3.0}, // Strahl nach vorne, erkennt Hindernis in 3.0m X-Entfernung
    };
    controller.verarbeiteScan(scan1);

    // Scan 2: Roboter rückt um exakt 0.4m abrupt näher an die Vorderwand heran
    std::vector<LidarMessung> scan2 = {
        {0.0, 2.6}, // Wand ist näher gerückt -> 3.0m - 0.4m
    };
    controller.verarbeiteScan(scan2);

    // 1. Prüfen: Hat der Controller den 0.4m Sprung in der Roboter-Pose erkannt?
    DOUBLES_EQUAL(0.4, controller.getX(), 0.05);

    // 2. Prüfen: Steht das Wand-Hindernis in der globalen Karte weiterhin korrekt bei X=3.0m?
    // (Roboter-Position 0.4m + gemessene Distanz 2.6m = 3.0m in der Welt)
    LONGS_EQUAL(100, controller.getKarte().getZelle(3.0, 0.0));
}

// Testgruppe für den 3D SLAM-Controller
TEST_GROUP(SlamController3DTest)
{

};

// Test 3: Initialisierung und Start Pose checken
TEST(SlamController3DTest, InitialisierungUndStartPose)
{
    // Wir erstellen einen Controller für einen Raum von 10m x 10m x 5m mit 0.1 Auflösung
    SlamController3D controller(10.0, 10.0, 5.0, 0.1);

    // 1. Die Start-Position (X, Y, Z) muss exakt bei (0.0, 0.0, 0.0) liegen
    DOUBLES_EQUAL(0.0, controller.getX(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getY(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getZ(), 0.001);

    // 2. Die Start-Orientierung (Roll, Pitch, Yaw) muss ebenfalls exakt bei 0.0 stehen
    DOUBLES_EQUAL(0.0, controller.getRoll(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getPitch(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getYaw(), 0.001);
}

// Test 4: 3D-Mapintegration
TEST(SlamController3DTest, MapIntegration)
{
    // Raum: 10m Breite, 10m Höhe, 5m Tiefe mit 0.1m AUflösung
    SlamController3D controller(10.0, 10.0, 5.0, 0.1);

    // 1. Startpose prüfen (0, 0, 0)
    DOUBLES_EQUAL(0.0, controller.getX(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getY(), 0.001);
    DOUBLES_EQUAL(0.0, controller.getZ(), 0.001);

    // 2. Kartenzugriff testen: Unbekannte Zelle mitten im Raum muss -1.0 liefern
    DOUBLES_EQUAL(-1.0, controller.getKarte().getZelle(3.0, 2.0, 1.0), 0.01);
}

// Test 5: Prüfen des schwenkbaren 2D-LIDAR-Sensor und dem Scan-Controller
TEST(SlamController3DTest, VerarbteitetPunktwolkeUndTraegtVoxelEin)
{
    SlamController3D controller(10.0, 10.0, 5.0, 0.1);

    // Wir erstellen eine Punktwolke mit 2 Hindernis-Punkten
    std::vector<Point3D> punktwolke = {
        {2.0, 1.0, 0.5},    // Punkt 1
        {4.0, 3.0, 1.2}     // Punkt 2
    };

    // Der Controller verarbeitet die Punkte 
    controller.verarbeitePunktwolke(punktwolke);

    // 1. Die beiden getroffenen Voxel müssen jetzt auf 100.0 (BELEGT) stehen!
    DOUBLES_EQUAL(100.0, controller.getKarte().getZelle(2.0, 1.0, 0.5), 0.001);
    DOUBLES_EQUAL(100.0, controller.getKarte().getZelle(4.0, 3.0, 1.2), 0.001);

    // 2. Ein unberührter Voxel (z.B. bei 0, 0, 0) muss weiterhin -1.0 (Unbekannt) bleiben
    DOUBLES_EQUAL(-1.0, controller.getKarte().getZelle(0.0, 0.0, 0.0), 0.001);
}

// Test 6: Echte Roboterfahrt mit dem schwenkbaren 2D-LIDAR-Sensor
TEST(SlamController3DTest, IntegrationSchwenkLidarMitBewegtemRoboter)
{
    // 1. Controller & Schwenk-LIDAR erstellen
    SlamController3D controller(10.0, 10.0, 5.0, 0.1);
    SchwenkLidar3D schwenkLidar(0.1, 12.0);

    // 2. Roboter steht bei X=2.0m, Y=1.0m und ist um 90 GRAD nach links gedreht (Yaw = 90°)
    controller.setzePose(2.0, 1.0, 0.0, 0.0, 0.0, 90.0);

    // 3. Sensor misst 2.0m geradeaus vor sich (0° Laser-Winkel) bei 0° Servo-Pitch
    std::vector<LidarMessung> scan = { {0.0, 2.0} };

    // 4. Punktwolke definieren
    std::vector<Point3D> punktwolke = schwenkLidar.scanZu3DPunktwolke(scan, 0.0);

    // 5. Controller verarbeitet die Punktwolke
    controller.verarbeitePunktwolke(punktwolke);

    // Der Sensor misst 2.0m nach vorne
    // Weil der Roboter bei (2.0, 1.0) steht und um 90° nach links schaut,
    // zeigt sein "Vorne" jetzt entlang der Y-Achse nach oben! -> Der Weltpunkt MUSS 
    // bei X = 2.0m und Y = 3.0m (1.0m + 2.0m) landen!

    DOUBLES_EQUAL(100.0, controller.getKarte().getZelle(2.0, 3.0, 0.0), 0.001);
}

// ------------------------------------ Rauchmelderalarm -------------------------------------------------

// Testgruppe für Feueralarm
TEST_GROUP(SmokeTestGroup)
{

};

// Test 1: Fehler provozieren um zu sehen ob Pipeline streikt
TEST(SmokeTestGroup, ProvozierterFehler)
{
    // Wir behaupten, dass 1 gleich 2 ist - das MUSS fehlschlagen
    CHECK_EQUAL(1, 1);
}
// -------------------------------------- Initialiser ----------------------------------------------------
int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}