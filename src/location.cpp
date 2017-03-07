#include <cmath>

#include "location.h"

loc::Distance::Distance(double x, double y) {
    this->x = x;
    this->y = y;
}

double loc::Distance::GetMagnitude() {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

double loc::Distance::GetBearing() {
    return std::fmod(std::atan2(this->x, this->y), 2 * PI);
}

loc::Distance loc::Distance::GetTransform(double angle) {
    double x = this->x * std::cos(angle) - this->y * std::sin(angle);
    double y = this->x * std::sin(angle) + this->y * std::cos(angle);

    return loc::Distance(x, y);
}

static loc::Distance FromMagnitude(double dist, double bearing) {
    double x = dist * std::sin(bearing);
    double y = dist * std::cos(bearing);

    return loc::Distance(x, y);
}

loc::Distance loc::operator+(const loc::Distance &dist, loc::Distance other) {
    return loc::Distance(dist.x + other.x, dist.y + other.y);
}

loc::Distance loc::operator-(const loc::Distance &dist, loc::Distance other) {
    return loc::Distance(dist.x - other.x, dist.y - dist.y);
}

loc::Distance loc::operator*(const loc::Distance &dist, double k) {
    return loc::Distance(dist.x * k, dist.y * k);
}

loc::Distance loc::operator*(double k, const loc::Distance &dist) {
    return loc::Distance(dist.x * k, dist.y * k);
}

loc::Distance loc::operator/(const loc::Distance &dist, double k) {
    return loc::Distance(dist.x / k, dist.y / k);
}

loc::Distance3D::Distance3D(double x, double y, double z) : Distance(x, y) {
    this->z = z;
}

double loc::Distance3D::GetMagnitude() {
    return std::sqrt(
        std::pow(this->x, 2) + std::pow(this->y, 2) + std::pow(this->z, 2)
    );
}

loc::Distance3D loc::Distance3D::GetTransform(double angle) {
    double x = this->x * std::cos(angle) - this->y * std::sin(angle);
    double y = this->x * std::sin(angle) + this->y * std::cos(angle);

    return loc::Distance3D(x, y, this->z);
}

static loc::Distance3D FromMagnitude(double dist, double bearing, double alt) {
    double x = dist * std::sin(bearing);
    double y = dist * std::cos(bearing);

    return loc::Distance3D(x, y, alt);
}

loc::Distance3D operator+(loc::Distance3D &dist, loc::Distance3D other){
    return loc::Distance3D(
        dist.x + other.x, dist.y + dist.y, dist.z + other.z
    );
}

loc::Distance3D loc::operator+(loc::Distance3D &dist, loc::Distance other) {
    return loc::Distance3D(dist.x + other.x, dist.y + dist.y, dist.z);
}

loc::Distance3D loc::operator+(loc::Distance other,
        const loc::Distance3D &dist) {
    return loc::Distance3D(dist.x + other.x, dist.y + dist.y, dist.z);
}

loc::Distance3D loc::operator-(loc::Distance3D &dist, loc::Distance3D other) {
    return loc::Distance3D(
        dist.x - other.x, dist.y - dist.y, dist.z - other.z
    );
}

loc::Distance3D loc::operator-(loc::Distance3D &dist, loc::Distance other) {
    return loc::Distance3D(dist.x - other.x, dist.y - dist.y, dist.z);
}

loc::Distance3D loc::operator-(loc::Distance other,
        const loc::Distance3D &dist) {
    return loc::Distance3D(other.x - dist.x, other.y - dist.y, -dist.z);
}

loc::Distance3D loc::operator*(loc::Distance3D &dist, double k) {
    return loc::Distance3D(dist.x * k, dist.y * k, dist.z * k);
}

loc::Distance3D loc::operator*(double k, const loc::Distance3D &dist) {
    return loc::Distance3D(dist.x * k, dist.y * k, dist.z * k);
}

loc::Distance3D loc::operator/(loc::Distance3D &dist, double k) {
    return loc::Distance3D(dist.x / k, dist.y / k, dist.z / k);
}
