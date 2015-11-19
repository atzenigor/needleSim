#include <QtOpenGL>
#include <QGLWidget>
#include <math.h>

#include "obstacle.h"

Obstacle::Obstacle(const Eigen::Vector4d& center, double size):
    _center(center),
    _size(size){
}

double Obstacle::getSize(){
    return _size;
}
Eigen::Vector4d& Obstacle::getCenter(){
    return _center;
}
/*
bool checkCollision(Eigen::Vector4d point){
    if () return true;
    else return false;
}
*/


