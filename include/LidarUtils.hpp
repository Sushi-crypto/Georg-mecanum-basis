#pragma once
#include <cmath>
#include <vector>
#include "LidarMessung.h"

class LidarUtils {
public:
    /**
     * @brief Fügt die Messwerte von zwei um +- 45° gedrehten LIDAR-Sensoren
     * in ein gemeinsames Roboter.Koordinatensystem zusammen
     */

    static inline double normalisiereWinkelGrad(double winkelGrad) {
        while (winkelGrad > 180.0) winkelGrad -= 360.0;
        while (winkelGrad < -180.0) winkelGrad += 360.0;
        return winkelGrad;
    }

    static inline std::vector<LidarMessung> fuegeDualLidarZusammen(
        const std::vector<LidarMessung>& scanLinks, // Sensor bei +45°
        const std::vector<LidarMessung>& scanRechts) // Sensor bei -45°
    {
        std::vector<LidarMessung> kombiniert;
        kombiniert.reserve(scanLinks.size() + scanRechts.size());

        // 1. Linken Sensor um +45° eindrehen
        for (const auto& messung : scanLinks) {
            LidarMessung m = messung;
            m.winkelGrad = normalisiereWinkelGrad(messung.winkelGrad + 45.0);
            kombiniert.push_back(m);
        }

        // 2. Rechten Sensor um -45° eindrehen
        for (const auto& messung : scanRechts) {
            LidarMessung m = messung;
            m.winkelGrad = normalisiereWinkelGrad(messung.winkelGrad - 45.0);
            kombiniert.push_back(m);
        }

        return kombiniert;
    }
};
