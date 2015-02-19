#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <vector>

class Obstacle{
public:
    Obstacle(const Eigen::Vector4d &position);
    Eigen::Vector4d& getPosition();
  //  bool checkCollision(Eigen::Vector4d point);
    void draw();

private:
    Eigen::Vector4d position_;
};

#endif // OBSTACLE_H
