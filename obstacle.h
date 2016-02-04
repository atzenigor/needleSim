#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <vector>

#define MARGINE 0.1
class Obstacle{
public:
    Obstacle(Eigen::Vector4d center, double size);
    double getSize();
    Eigen::Vector4d& getCenter();

    bool contains(Eigen::Vector4d point);
private:
    Eigen::Vector4d _center;
    double _size;
};

#endif // OBSTACLE_H
