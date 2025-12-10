#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stm32_interface.h"

enum class LidarPointStatus {
    TOO_NEAR,   // < 0.2 м
    VALID,      // 0.2 - 2.5 м
    TOO_FAR,    // > 2.5 м
    INVALID     // Нет данных
};

struct FreeSpaceInfo {
    std::array<float, 8> sector_distances;  // 8 секторов по 45°
    std::array<bool, 8> sector_free;        // Свободен ли сектор
};

struct LidarConfig {
    float min_range = 0.2f;      // Минимальная рабочая дистанция (м)
    float max_range = 2.5f;      // Максимальная рабочая дистанция (м)
    float safety_margin = 0.3f;  // Запас безопасности (м)
    
    // Сектор, закрытый корпусом (градусы)
    float body_block_start = 150.0f;
    float body_block_end = 210.0f;
    
    // Параметры движения
    float base_speed = 0.3f;     // Базовая скорость (м/с)
    float rotation_speed = 0.5f; // Скорость поворота (рад/с)
    
    // Параметры алгоритма жука
    float wall_follow_distance = 0.5f;  // Дистанция следования вдоль стены
    float wall_lost_threshold = 1.0f;   // Порог потери стены
};

class Movement {
public:

    enum class State {
        CORRIDOR,       // Движение по коридору
        AVOID_OBSTACLE, // Обход препятствия
        TURN_AROUND,    // Разворот в тупике
        EXPLORE         // Исследование
    };
    Movement(const LidarConfig& config);
    
    MotorCommand processScan(const std::vector<float>& ranges);
    
    State getState() const { return state_; }
private:
    LidarConfig config_;
    State state_;

    // Для следования вдоль стены
    bool wall_on_right_;
    int obstacle_avoidance_counter_;
    float exploration_direction_;
    
    // Анализ окружения
    FreeSpaceInfo analyzeFreeSpace(const std::vector<float>& ranges);
    std::vector<float> getFrontSector(const std::vector<float>& ranges, float width_deg);
    bool checkEmergencyStop(const FreeSpaceInfo& info);
    
    // Определение типа окружения
    bool isCorridor(const std::vector<float>& ranges);
    bool isDeadEnd(const FreeSpaceInfo& info);
    bool hasObstacleInFront(const FreeSpaceInfo& info);
    
    // Поведения
    MotorCommand followCorridor(const std::vector<float>& ranges);
    MotorCommand avoidObstacle(const std::vector<float>& ranges);
    MotorCommand turnAround(const std::vector<float>& ranges);
    MotorCommand explore(const std::vector<float>& ranges);
    
    // Вспомогательные функции
    float findBestOpening(const std::vector<float>& ranges);
    float getSideDistance(const std::vector<float>& ranges, bool right_side);
    float getFrontDistance(const std::vector<float>& ranges);
    bool isBodyBlockedAngle(int angle) const;
    float degreeToRadian(float deg) const { return deg * M_PI / 180.0f; }
    float radianToDegree(float rad) const { return rad * 180.0f / M_PI; }
};

#endif