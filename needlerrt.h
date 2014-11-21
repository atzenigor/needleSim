#ifndef NEEDLERRT_H
#define NEEDLERRT_H
#include "needletree.h"
#include "eigen3/Eigen/Dense"

class Needlerrt{
private:
  NeedleTree tree;
  bool finished;

public:
  static void randomFill(Eigen::Vector4d& v); //should be private
  Needlerrt();
  void makeStep();
  bool isFinished();
  void insertObstacle();
  void getPath();
  void getTree();

};


#endif // NEEDLERRT_H
