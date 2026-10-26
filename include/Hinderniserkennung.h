#pragma once

// Klasse für die Hinderniserkennung
class Hinderniserkennung
{
public:
    int berechneGeschwindigkeitVy(int distanzCm)
    {
        // 1. Grünes Licht: Abstand ist größer als 50 cm -> Vollgas!
        if(distanzCm > 50)
        {
            return 100;
        }

        // 2. Gelbes Licht: Warnzone (zwischen 21 cm und 50 cm) -> Halbe Kraft!
        else if(distanzCm > 20)
        {
            return 50;
        }

        // 3. Rotes Licht: Bei Hindernissen (<= 20 cm) -> Stopp!
        return 0;
    }
};