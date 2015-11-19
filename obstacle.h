#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <vector>

class Obstacle{
public:
    Obstacle(const Eigen::Vector4d &center, double size);
    double getSize();
    Eigen::Vector4d& getCenter();

  //  bool checkCollision(Eigen::Vector4d point);
private:
    Eigen::Vector4d _center;
    double _size;
};

#endif // OBSTACLE_H
