#include <QtOpenGL>
#include <QGLWidget>
#include <math.h>

#include "obstacle.h"

using namespace Eigen;
Obstacle::Obstacle(Vector4d center, double size):
    _center(center),
    _size(size){
}

double Obstacle::getSize(){
    return _size;
}
Vector4d& Obstacle::getCenter(){
    return _center;
}

bool Obstacle::contains(Vector4d point){
    if ( ((point - _center).array().abs() < (_size + MARGINE)).all() )
        return true;
    else return false;
}



