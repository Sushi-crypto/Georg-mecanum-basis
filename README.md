# 🤖 Georg Mecanum Basis - Autonome Robotersteuerung

Willkommen zur Dokumentation des **Georg-Mecanum-Basis** Projekts! Hierbei handelt es sich um eine hochmoderne, modular aufgebaute C++ Software-Architektur zur autonomen Navigation holonomer Mecanum-Roboter.

Das System wurde von Grund auf nach den Prinzipien des **Test-Driven Developments (TDD)** entwickelt, per **CppUTest** abgesichert und über eine automatisierte **GitHub Actions CI/CD-Pipeline** integriert.

---

## 🏛️ Architektur-Übersicht

Das Gesamtsystem vereint Ortung (SLAM), Pfadplanung (Bug3) und Bewegungsglättung (PT2-Filter) in einer zentralen Fassade (`RobotSystem`), die Steuerbefehle direkt an die Hardware-Abstraktionsschicht (`IHalDrive` / `HalMecanum`) weitergibt.

```
                  [ Ziel-Koordinaten (X, Y) + LIDAR-Scan ]
                                     │
                                     ▼
                     ┌───────────────────────────────┐
                     │       SlamController2D        │
                     │  (Eigenortung & Kartierung)   │
                     └───────────────┬───────────────┘
                                     │ (X, Y, Theta)
                                     ▼
                     ┌───────────────────────────────┐
                     │        Bug3Controller         │
                     │ (Autonome Pfadplanung & FSM)  │
                     └───────────────┬───────────────┘
                                     │ (vx, vy, omega)
                                     ▼
                     ┌───────────────────────────────┐
                     │     3x PT2Regler (Filter)     │
                     │  (pt2Vx, pt2Vy, pt2Omega)      │
                     └───────────────┬───────────────┘
                                     │ (Geglättete Geschwindigkeiten)
                                     ▼
                     ┌───────────────────────────────┐
                     │    IHalDrive / HalMecanum     │
                     │  (4-Rad Mecanum Kinematik)    │
                     └───────────────────────────────┘
```

---

## 🧩 Die Hauptkomponenten im Detail

### 1. `RobotSystem` (Die Zentrale Fassade)
`RobotSystem` verbindet alle Teilsysteme miteinander. Es bietet zwei Betriebsmodi:
* **Manuelle/Gefilterte Steuerung (`verarbeiteScanUndFahre`):** Nimmt externe Geschwindigkeitsbefehle entgegen, verarbeitet den LIDAR-Scan für SLAM und glättet `vx`, `vy` und `omega` über PT2-Filter.
* **Vollautonome Zielanfahrt (`fahreZumZiel`):** Der Roboter steuert selbstständig ein Ziel $(X, Y)$ an, weicht Hindernissen fließend aus und richtet sich kontinuierlich aus.

### 2. `Bug3Controller` (Autonome 2D-Navigation & Elchtest)
Der `Bug3Controller` basiert auf einem Zustandsautomaten (Finite State Machine) mit zwei Zuständen:
* **`GOAL_SEEKING`:** Berechnet den 2D-Richtungsvektor $(dx, dy)$ zum Ziel für fließende Vorwärts- und Seitwärtsfahrten sowie eine proportionale Ausrichtung ($\omega$).
* **`WALL_FOLLOWING`:** 
  * **Elchtest-Prinzip:** Reduziert die Vorwärtsgeschwindigkeit ($v_x$) proportional zum Abstand vor dem Hindernis, während der Seitwärtstrieb ($v_y$) aufgebaut wird.
  * **Adaptive Notfall-Lenkung (Gain Scheduling):** Fällt $v_x < 0.2 \text{ m/s}$, steigert der Regler die P-Verstärkung ($k_P$) dynamisch von $0.5$ auf bis zu $2.5$ und dreht die Roboter-Nase ($\omega$) aktiv vom Hindernis weg.

### 3. `PT2Regler` (S-Kurven-Beschleunigung)
Ein zweistufiges Verzögerungsglied (Zwei-Eimer-Modell) mit der Zeitkonstanten $T = 0.2\text{ s}$. 
* Drei unabhängige Instanzen (`pt2Vx`, `pt2Vy`, `pt2Omega`) filtern abrupte Geschwindigkeitsänderungen.
* Verwandelt rechteckige Soll-Sprünge in butterweiche **S-Kurven-Beschleunigungsprofile**, was Motorruckeln und Schlupf an den Mecanum-Rädern verhindert.

### 4. `SlamController2D` (Kartenaufbau & Ortung)
Verbindet den `ScanMatcher2D` zur Drift-Korrektur mit einem `RayCaster2D` und einem 2D-Belegungsraster (`OccupancyGrid`).
* Verarbeitet fortlaufend LIDAR-Messungen (`std::vector<LidarMessung>`).
* Aktualisiert die geschätzte Roboter-Pose $(X, Y, \Theta)$ in der Welt.

### 5. `IHalDrive` / `HalMecanum` (Hardware-Abstraktion)
Schnittstelle zur Radansteuerung. Rechnet die geglätteten 2D-Geschwindigkeiten ($v_x, v_y, \omega$) in die individuellen Raddrehschwindigkeiten der vier Mecanum-Räder um.

---

## 🧪 Test-Driven Development (TDD) & CI/CD

Das gesamte Projekt ist vollständig durch automatisierte Unit-Tests abgesichert:
* **Framework:** CppUTest
* **Test-Suiten:** `Bug3Test.cpp`, `RobotSystemTest.cpp`, `PT2Test.cpp`, `SlamTest.cpp`
* **Coverage:** Abgedeckt sind 2D-Holonomie, PT2-Filterstufen, Wandfolgemodus, adaptive Notfall-Lenkung sowie fehlerfreie Zielanfahrten.

### Build & Ausführung der Tests

```bash
# Projekt kompilieren
make

# Test-Suite ausführen
./smoke_test
```

Bei jedem `git push` wird die CI/CD-Pipeline via GitHub Actions ausgeführt und validiert den Code automatisch.

---

*Erstellt mit Begeisterung für präzise C++ Robotik & TDD!* 🚀
