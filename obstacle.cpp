#include <QtOpenGL>
#include <QGLWidget>
#include <math.h>

#include "obstacle.h"

Obstacle::Obstacle(const Eigen::Vector4d& position):
    position_(position){
}

Eigen::Vector4d& Obstacle::getPosition(){
    return position_;
}

/*
bool checkCollision(Eigen::Vector4d point){
    if () return true;
    else return false;
}
*/

void draw(){
}

