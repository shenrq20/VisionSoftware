#include "coordinate.h"

CartesianCoordinate::CartesianCoordinate() = default;

CartesianCoordinate::CartesianCoordinate(const std::vector<double>& position) {
    x_ = position[0];
    y_ = position[1];
    z_ = position[2];
    a_ = position[3];
    b_ = position[4];
    c_ = position[5];
}

CartesianCoordinate::CartesianCoordinate(double x, double y, double z, double a, double b, double c) {

}

CartesianCoordinate::~CartesianCoordinate() = default;


