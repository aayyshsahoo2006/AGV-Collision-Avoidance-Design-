#ifndef SIMULATION_OBSTACLE_HPP
#define SIMULATION_OBSTACLE_HPP

#include "Vector2.hpp"
#include "Pose.hpp"
#include <string>

namespace agv {
namespace simulation {

enum class ObstacleType {
    STATIC,
    DYNAMIC_LINEAR,
    CROSSING
};

class Obstacle {
public:
    Obstacle(int id,
             const Vector2& initial_pos,
             const Vector2& velocity = {0.0, 0.0},
             double radius = 0.3,
             ObstacleType type = ObstacleType::STATIC);

    void update(double dt);

    int getId() const { return id_; }
    const Vector2& getPosition() const { return position_; }
    void setPosition(const Vector2& pos) { position_ = pos; }

    const Vector2& getVelocity() const { return velocity_; }
    void setVelocity(const Vector2& vel) { velocity_ = vel; }

    double getRadius() const { return radius_; }
    ObstacleType getType() const { return type_; }

    std::string toString() const;

private:
    int id_;
    Vector2 position_;
    Vector2 velocity_;
    double radius_{0.3};
    ObstacleType type_{ObstacleType::STATIC};
};

}
}

#endif
