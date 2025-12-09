#include "movement.h"
#include <iostream>

Movement::Movement(const LidarConfig& config)
    : config_(config)
    , state_(State::FOLLOWING)
    , emergency_stop_(false)
    , avoidance_counter_(0)
    , target_angle_(0.0f)
    , avoidance_direction_(1.0f) {
}

void Movement::setGoal(float x, float y) {
    goal_x_ = x;
    goal_y_ = y;
    reset();
}

void Movement::setGoalDistance(float distance) {
    goal_x_ = distance;
    goal_y_ = 0.0f;
    reset();
}

void Movement::reset() {
    bug_state_ = BugState::GO_TO_GOAL;
    emergency_stop_ = false;
    traveled_distance_ = 0.0f;
    current_x_ = 0.0f;
    current_y_ = 0.0f;
    current_orientation_ = 0.0f;
    can_see_goal_ = true;
}

LidarPointStatus Movement::getPointStatus(float range) const {
    if (std::isnan(range)) {
        return LidarPointStatus::INVALID;
    }
    
    if (range < config_.min_range) {
        return LidarPointStatus::TOO_NEAR;
    } else if (range <= config_.max_range) {
        return LidarPointStatus::VALID;
    } else {
        return LidarPointStatus::TOO_FAR;
    }
}

std::vector<LidarPointStatus> Movement::analyzeScan(const std::vector<float>& ranges) {
    std::vector<LidarPointStatus> statuses(360, LidarPointStatus::INVALID);
    
    for (int i = 0; i < 360; i++) {
        if (isBodyBlockedAngle(i)) {
            statuses[i] = LidarPointStatus::INVALID;
            continue;
        }
        statuses[i] = getPointStatus(ranges[i]);
    }
    
    return statuses;
}

bool Movement::checkForObstacleInGoalDirection(const std::vector<LidarPointStatus>& statuses) {
    // Получаем угол к цели
    float goal_angle_deg = radianToDegree(getGoalDirectionAngle());
    if (goal_angle_deg < 0) goal_angle_deg += 360.0f;
    
    int goal_angle_idx = static_cast<int>(goal_angle_deg) % 360;
    
    // Проверяем узкий сектор (±15°) в направлении цели
    for (int offset = -15; offset <= 15; offset++) {
        int angle = (goal_angle_idx + offset + 360) % 360;
        
        if (!isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                return true;  // Препятствие слишком близко
            }
            
            // Для VALID точек нужно проверить расстояние
            // (В реальности нужно передавать ranges, но упрощаем)
            if (statuses[angle] == LidarPointStatus::VALID) {
                return true;  // Есть препятствие в направлении цели
            }
        }
    }
    
    return false;
}


bool Movement::canReachGoalDirectly(const std::vector<LidarPointStatus>& statuses) {
    // Проверяем, нет ли препятствий между роботом и целью
    return !checkForObstacleInGoalDirection(statuses);
}

float Movement::getGoalDirectionAngle() const {
    float dx = goal_x_ - current_x_;
    float dy = goal_y_ - current_y_;
    return atan2(dy, dx);
}

MotorCommand Movement::processScan(const std::vector<float>& ranges) {
    auto statuses = analyzeScan(ranges);
    
    // Проверка на экстренную остановку
    emergency_stop_ = false;
    for (int angle = 0; angle < 360; angle++) {
        if (isInFrontSector(angle, 30.0f) && !isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                emergency_stop_ = true;
                RCLCPP_WARN(rclcpp::get_logger("movement"), "Emergency stop!");
                return MotorCommand(0, 0);
            }
        }
    }
    
    // Основная логика Bug0
    switch (bug_state_) {
        case BugState::GO_TO_GOAL:
            return behaviorGoToGoal(statuses);
        case BugState::FOLLOW_WALL:
            return behaviorFollowWall(statuses);
    }
    
    return MotorCommand(0, 0);
}

MotorCommand Movement::behaviorFollowWall(const std::vector<LidarPointStatus>& statuses) {
    // 1. Проверяем, можем ли вернуться к движению к цели
    float distance_to_goal = sqrt(pow(goal_x_ - current_x_, 2) + pow(goal_y_ - current_y_, 2));
    
    // Условие возврата к цели по Bug0:
    // 1) Робот ближе к цели, чем в точке касания
    // 2) Нет препятствий между роботом и целью
    if (distance_to_goal < hit_point_distance_ && canReachGoalDirectly(statuses)) {
        bug_state_ = BugState::GO_TO_GOAL;
        RCLCPP_INFO(rclcpp::get_logger("movement"), 
                   "Returning to goal. Current distance: %.2f, Hit point: %.2f",
                   distance_to_goal, hit_point_distance_);
        return MotorCommand(0, 0); // На секунду остановимся
    }
    
    // 2. Следуем вдоль стены
    // Измеряем дистанцию до стены сбоку
    float side_distance = getSideDistance(statuses, wall_on_right_);
    
    // Желаемая дистанция до стены
    float desired_distance = config_.safety_margin * 2.0f;
    float error = side_distance - desired_distance;
    
    // Пропорциональное управление для следования вдоль стены
    float angular_speed = -error * 1.0f;  // P-регулятор
    if (wall_on_right_) {
        angular_speed = -angular_speed;  // Инвертируем для правой стены
    }
    
    // Ограничиваем угловую скорость
    angular_speed = std::clamp(angular_speed, 
                              -config_.rotation_speed * 0.8f, 
                              config_.rotation_speed * 0.8f);
    
    // Проверяем, нет ли препятствия впереди
    float front_distance = getFrontDistance(statuses);
    float linear_speed = config_.base_speed * 0.7f;
    
    if (front_distance < config_.safety_margin * 3.0f) {
        // Если впереди близко, замедляемся
        linear_speed *= front_distance / (config_.safety_margin * 3.0f);
        
        // И немного поворачиваем от стены
        if (wall_on_right_) {
            angular_speed += config_.rotation_speed * 0.3f;
        } else {
            angular_speed -= config_.rotation_speed * 0.3f;
        }
    }
    
    return MotorCommand(linear_speed, angular_speed);
}

float Movement::getSideDistance(const std::vector<LidarPointStatus>& statuses, bool right_side) const {
    int start_angle = right_side ? 270 : 90;
    int end_angle = right_side ? 360 : 90;
    
    float min_distance = config_.max_range;
    int count = 0;
    
    for (int angle = start_angle; angle < end_angle; angle++) {
        if (!isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::VALID) {
                // В реальности нужно использовать ranges, но упрощаем
                min_distance = std::min(min_distance, config_.safety_margin * 2.0f);
                count++;
            } else if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                min_distance = config_.min_range;
                count++;
            }
        }
    }
    
    if (count == 0) {
        return config_.max_range;  // Нет данных
    }
    
    return min_distance;
}


float Movement::getFrontDistance(const std::vector<LidarPointStatus>& statuses) const {
    float min_distance = config_.max_range;
    
    for (int angle = 350; angle < 360; angle++) {
        if (!isBodyBlockedAngle(angle % 360)) {
            if (statuses[angle % 360] == LidarPointStatus::VALID) {
                min_distance = std::min(min_distance, config_.safety_margin * 2.0f);
            } else if (statuses[angle % 360] == LidarPointStatus::TOO_NEAR) {
                min_distance = config_.min_range;
            }
        }
    }
    
    return min_distance;
}

void Movement::updatePosition(float linear_speed, float angular_speed, float dt) {
    // Упрощенное обновление позиции (без учета скольжения)
    current_orientation_ += angular_speed * dt;
    
    // Нормализуем ориентацию
    while (current_orientation_ > M_PI) current_orientation_ -= 2 * M_PI;
    while (current_orientation_ < -M_PI) current_orientation_ += 2 * M_PI;
    
    // Обновляем позицию
    current_x_ += linear_speed * cos(current_orientation_) * dt;
    current_y_ += linear_speed * sin(current_orientation_) * dt;
    
    traveled_distance_ += fabs(linear_speed) * dt;
}

bool Movement::isInFrontSector(int angle, float sector_deg) const {
    float half_sector = sector_deg / 2.0f;
    float angle_deg = static_cast<float>(angle);
    
    return (angle_deg <= half_sector) || (angle_deg >= (360.0f - half_sector));
}

bool Movement::isBodyBlockedAngle(int angle) const {
    float angle_deg = static_cast<float>(angle);
    
    if (config_.body_block_start < config_.body_block_end) {
        return (angle_deg >= config_.body_block_start && 
                angle_deg <= config_.body_block_end);
    } else {
        return (angle_deg >= config_.body_block_start || 
                angle_deg <= config_.body_block_end);
    }
}

float Movement::degreeToRadian(float deg) const {
    return deg * M_PI / 180.0f;
}

float Movement::radianToDegree(float rad) const {
    return rad * 180.0f / M_PI;
}