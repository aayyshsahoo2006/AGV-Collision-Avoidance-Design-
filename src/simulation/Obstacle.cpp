#include "../../include/simulation/Obstacle.hpp"
#include <sstream>

namespace agv {
namespace simulation {

Obstacle::Obstacle(int id,
                   const Vector2& initial_pos,
                   const Vector2& velocity,
                   double radius,
                   ObstacleType type)
    : id_(id), position_(initial_pos), velocity_(velocity), radius_(radius), type_(type) {}

void Obstacle::update(double dt) {
    if (dt <= 0.0) return;
    position_ += velocity_ * dt;
}

std::string Obstacle::toString() const {
    std::ostringstream ss;
    ss << "Obstacle[ID=" << id_
       << ", pos=" << position_.toString()
       << ", vel=" << velocity_.toString()
       << ", radius=" << radius_ << "m]";
    return ss.str();
}

}
}
