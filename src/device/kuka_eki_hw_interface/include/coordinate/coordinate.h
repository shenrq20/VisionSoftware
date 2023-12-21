#ifndef INC_POSITION_H
#define INC_POSITION_H

#include <vector>

class CartesianCoordinate {
public:
    double x_{}, y_{}, z_{}, a_{}, b_{}, c_{};

    CartesianCoordinate();

    explicit CartesianCoordinate(const std::vector<double>& position);

    CartesianCoordinate(double x, double y, double z, double a, double b, double c);

    ~CartesianCoordinate();

};


#endif //INC_POSITION_H
