#ifndef SIMULATION_VECTOR2_HPP
#define SIMULATION_VECTOR2_HPP

#include <cmath>
#include <string>
#include <sstream>

namespace agv {
namespace simulation {

/**
 * @brief 2D Cartesian vector representing position, direction, or velocity.
 */
struct Vector2 {
    double x{0.0};
    double y{0.0};

    constexpr Vector2() = default;
    constexpr Vector2(double x_val, double y_val) : x(x_val), y(y_val) {}

    Vector2 operator+(const Vector2& other) const { return {x + other.x, y + other.y}; }
    Vector2 operator-(const Vector2& other) const { return {x - other.x, y - other.y}; }
    Vector2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Vector2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vector2& operator+=(const Vector2& other) { x += other.x; y += other.y; return *this; }
    Vector2& operator-=(const Vector2& other) { x -= other.x; y -= other.y; return *this; }
    Vector2& operator*=(double scalar) { x *= scalar; y *= scalar; return *this; }

    double lengthSquared() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSquared()); }
    double distanceTo(const Vector2& other) const { return (*this - other).length(); }
    double dot(const Vector2& other) const { return x * other.x + y * other.y; }

    Vector2 normalized() const {
        double len = length();
        if (len < 1e-9) return {0.0, 0.0};
        return *this / len;
    }

    std::string toString() const {
        std::ostringstream ss;
        ss << "(" << x << ", " << y << ")";
        return ss.str();
    }
};

}
}

#endif
