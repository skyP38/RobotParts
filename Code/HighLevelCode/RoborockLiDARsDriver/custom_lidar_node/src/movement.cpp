#include "movement.h"
#include <iostream>

Movement::Movement(const LidarConfig& config)
    : config_(config)
    , state_(State::EXPLORE)
    , wall_on_right_(true)
    , obstacle_avoidance_counter_(0)
    , exploration_direction_(0.0f)
{}

MotorCommand Movement::processScan(const std::vector<float>& ranges) {
    // 1. Анализ окружения
    auto free_space = analyzeFreeSpace(ranges);
    auto front_sector = getFrontSector(ranges, 45.0f);  // ±22.5 градуса

    // 2. Экстренная остановка
    if (checkEmergencyStop(free_space)) {
        RCLCPP_WARN(rclcpp::get_logger("movement"), "EMERGENCY STOP!");
        return MotorCommand(0, 0);
    }

    // 3. Выбор поведения на основе окружения
    if (isCorridor(ranges)) {
        return followCorridor(ranges);
    } 
    else if (isDeadEnd(free_space)) {
        return turnAround(ranges);
    }
    else if (hasObstacleInFront(free_space)) {
        return avoidObstacle(ranges);
    }
    else {
        return explore(ranges);
    }
}

FreeSpaceInfo Movement::analyzeFreeSpace(const std::vector<float>& ranges) {
    FreeSpaceInfo info;
    
    // Анализируем все секторы
    for (int sector = 0; sector < 8; sector++) {
        float min_dist = config_.max_range;
        int start_angle = sector * 45;
        int end_angle = start_angle + 45;
        
        for (int angle = start_angle; angle < end_angle; angle++) {
            int idx = angle % 360;
            if (!isBodyBlockedAngle(idx) && !std::isnan(ranges[idx])) {
                min_dist = std::min(min_dist, ranges[idx]);
            }
        }
        
        info.sector_distances[sector] = min_dist;
        info.sector_free[sector] = (min_dist > config_.wall_follow_distance * 1.5f);
    }
    
    return info;
}

std::vector<float> Movement::getFrontSector(const std::vector<float>& ranges, float width_deg) {
    std::vector<float> sector;
    int half_width = static_cast<int>(width_deg / 2);
    
    for (int offset = -half_width; offset <= half_width; offset++) {
        int idx = (offset + 360) % 360;
        if (!isBodyBlockedAngle(idx)) {
            sector.push_back(ranges[idx]);
        }
    }
    
    return sector;
}

bool Movement::checkEmergencyStop(const FreeSpaceInfo& info) {
    // Проверяем ближайшие секторы спереди
    if (info.sector_distances[0] < config_.min_range * 1.5f ||
        info.sector_distances[7] < config_.min_range * 1.5f) {
        return true;
    }
    return false;
}

bool Movement::isCorridor(const std::vector<float>& ranges) {
    float left_avg = getSideDistance(ranges, false);
    float right_avg = getSideDistance(ranges, true);
    float front_avg = getFrontDistance(ranges);
    
    // Коридор, если есть параллельные стены по бокам и свободно впереди
    bool has_left_wall = (left_avg < config_.wall_follow_distance * 2.0f);
    bool has_right_wall = (right_avg < config_.wall_follow_distance * 2.0f);
    bool front_clear = (front_avg > config_.wall_follow_distance * 1.5f);
    
    return (has_left_wall && has_right_wall && front_clear);
}

bool Movement::isDeadEnd(const FreeSpaceInfo& info) {
    // Тупик, если нет свободного пространства впереди (0 и 7) и сбоку (1 и 6)
    return !info.sector_free[0] && !info.sector_free[1] && !info.sector_free[6] && !info.sector_free[7];
}

bool Movement::hasObstacleInFront(const FreeSpaceInfo& info) {
    // Препятствие впереди, если центральные секторы заняты
    return !info.sector_free[0] || !info.sector_free[7];
}


MotorCommand Movement::followCorridor(const std::vector<float>& ranges) {
    state_ = State::CORRIDOR;
    
    float left_dist = getSideDistance(ranges, false);
    float right_dist = getSideDistance(ranges, true);
    float front_dist = getFrontDistance(ranges);
    
    float linear_speed = config_.base_speed;
    float angular_speed = 0.0f;
    
    // Пропорциональный регулятор для удержания в центре коридора
    float corridor_center_error = left_dist - right_dist;
    angular_speed = corridor_center_error * 0.5f;
    
    // Замедление перед поворотом или сужением
    if (front_dist < 2.0f) {
        linear_speed *= (front_dist / 2.0f);
    }
    
    // Если одна стена ближе другой более чем в 2 раза, немного поворачиваем
    if (left_dist > right_dist * 2.0f) {
        angular_speed -= 0.2f;  // Поворачиваем вправо
    } else if (right_dist > left_dist * 2.0f) {
        angular_speed += 0.2f;  // Поворачиваем влево
    }
    
    angular_speed = std::clamp(angular_speed, -config_.rotation_speed * 0.5f, config_.rotation_speed * 0.5f);
    
    return MotorCommand(linear_speed, angular_speed);
}

MotorCommand Movement::avoidObstacle(const std::vector<float>& ranges) {
    state_ = State::AVOID_OBSTACLE;
    obstacle_avoidance_counter_++;
    
    FreeSpaceInfo info = analyzeFreeSpace(ranges);
    
    // Ищем самый свободный сектор
    float best_direction = 0.0f;
    float max_distance = 0.0f;
    
    for (int sector = 0; sector < 8; sector++) {
        if (info.sector_distances[sector] > max_distance) {
            max_distance = info.sector_distances[sector];
            best_direction = sector * 45.0f - 180.0f;  // В градусах, -180..180
        }
    }
    
    // Конвертируем в радианы и нормализуем
    best_direction = degreeToRadian(best_direction);
    
    // Плавный поворот к свободному пространству
    float angular_speed = best_direction * 2.0f;
    float linear_speed = config_.base_speed * 0.5f;
    
    // Если свободно прямо, едем быстрее
    if (std::abs(best_direction) < M_PI/6) {  // < 30 градусов
        linear_speed = config_.base_speed * 0.8f;
    }
    
    // Ограничения
    angular_speed = std::clamp(angular_speed, -config_.rotation_speed, config_.rotation_speed);
    
    // Сбрасываем счетчик, если вышли из режима избегания
    if (obstacle_avoidance_counter_ > 50) {  // ~1 секунда
        obstacle_avoidance_counter_ = 0;
    }
    
    return MotorCommand(linear_speed, angular_speed);
}

MotorCommand Movement::turnAround(const std::vector<float>& ranges) {
    state_ = State::TURN_AROUND;
    
    // Ищем самый свободный сектор (кроме заднего)
    FreeSpaceInfo info = analyzeFreeSpace(ranges);
    
    int best_sector = 4;  // По умолчанию назад (180°)
    float max_distance = 0.0f;
    
    for (int sector = 2; sector < 6; sector++) {  // Смотрим секторы 90°-270°
        if (info.sector_distances[sector] > max_distance) {
            max_distance = info.sector_distances[sector];
            best_sector = sector;
        }
    }
    
    // Вычисляем угол поворота
    float target_angle = (best_sector * 45.0f) - 180.0f;  // В градусах
    target_angle = degreeToRadian(target_angle);
    
    // Поворачиваем на месте
    float angular_speed = std::copysign(config_.rotation_speed * 0.8f, target_angle);
    
    return MotorCommand(0, angular_speed);
}

MotorCommand Movement::explore(const std::vector<float>& ranges) {
    state_ = State::EXPLORE;
    
    float front_dist = getFrontDistance(ranges);
    float linear_speed = config_.base_speed * 0.7f;
    float angular_speed = exploration_direction_;
    
    // Если впереди свободно, едем прямо
    if (front_dist > config_.wall_follow_distance * 2.0f) {
        angular_speed *= 0.5f;  // Постепенно выравниваемся
        
        // Случайный поворот для исследования
        if (rand() % 100 < 10) {  // 10% chance
            exploration_direction_ = (rand() % 2 == 0) ? 0.2f : -0.2f;
            angular_speed = exploration_direction_;
        }
    } else {
        // Ищем свободное направление
        exploration_direction_ = findBestOpening(ranges);
        angular_speed = exploration_direction_;
        linear_speed *= 0.5f;
    }
    
    // Обновляем направление исследования
    exploration_direction_ = exploration_direction_ * 0.9f + angular_speed * 0.1f;
    
    return MotorCommand(linear_speed, angular_speed);
}

float Movement::findBestOpening(const std::vector<float>& ranges) {
    // Анализируем 8 секторов по 45 градусов
    float best_score = -9999.0f;
    float best_angle = 0.0f;
    
    for (int sector = 0; sector < 8; sector++) {
        float sector_score = 0.0f;
        int start_angle = sector * 45;
        
        for (int offset = 0; offset < 45; offset += 5) {
            int angle = (start_angle + offset) % 360;
            int idx = angle;
            
            if (!isBodyBlockedAngle(idx) && !std::isnan(ranges[idx])) {
                float dist = ranges[idx];
                
                // Баллы за дистанцию (чем дальше, тем лучше)
                sector_score += dist;
                
                // Бонус за передние направления (0° и 360°)
                if (angle < 30 || angle > 330) {
                    sector_score += dist * 0.5f;
                }
                
                // Штраф за слишком близкие объекты
                if (dist < config_.min_range * 2.0f) {
                    sector_score -= 10.0f;
                }
            }
        }
        
        if (sector_score > best_score) {
            best_score = sector_score;
            best_angle = sector * 45.0f - 180.0f;  // В градусах [-180, 180]
        }
    }
    
    // Конвертируем в радианы и нормализуем
    return degreeToRadian(best_angle) * 0.5f;
}

// функции для получения расстояний
float Movement::getSideDistance(const std::vector<float>& ranges, bool right_side) {
    int center = right_side ? 270 : 90;
    float sum = 0.0f;
    int count = 0;
    
    for (int offset = -15; offset <= 15; offset += 5) {
        int idx = (center + offset + 360) % 360;
        if (!isBodyBlockedAngle(idx) && !std::isnan(ranges[idx])) {
            sum += ranges[idx];
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : config_.max_range;
}

float Movement::getFrontDistance(const std::vector<float>& ranges) {
    float min_dist = config_.max_range;
    
    for (int offset = -15; offset <= 15; offset += 3) {
        int idx = (offset + 360) % 360;
        if (!isBodyBlockedAngle(idx) && !std::isnan(ranges[idx])) {
            min_dist = std::min(min_dist, ranges[idx]);
        }
    }
    
    return min_dist;
}

bool Movement::isBodyBlockedAngle(int angle) const {
    float angle_deg = static_cast<float>(angle);
    
    if (config_.body_block_start < config_.body_block_end) {
        // Обычный случай: сектор не пересекает 0°
        return (angle_deg >= config_.body_block_start && 
                angle_deg <= config_.body_block_end);
    } else {
        // Сектор пересекает 0° (например, от 330° до 30°)
        return (angle_deg >= config_.body_block_start || 
                angle_deg <= config_.body_block_end);
    }
}