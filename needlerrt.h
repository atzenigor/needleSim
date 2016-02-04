#ifndef NEEDLERRT_H
#define NEEDLERRT_H
#include "needletree.h"
#include "obstacle.h"
#include "eigen3/Eigen/Dense"
#define _USE_MATH_DEFINES
#include <math.h>

#define MIN_R 0.05  //must be double
#define K_CONST 0.2

// boundary of world space
#define BOUND_XY 2.0
#define BOUND_Z_MAX 2.0
#define BOUND_Z_MIN 0.0
#define SAMPLING_BIAS 2  // 1 the boundary and the variance coincide, the graither the more bias

#include "obstacle.h"
#include "math.h"

class Needlerrt{
private:
  NeedleTree _tree;
  bool _finished;
  NVertex *_goal_vertex_ptr;
  Eigen::Vector4d _goal_center;
  double _goal_ray_squared;
  double _goal_ray;
  std::vector<Obstacle> _list_of_obstacles;
  std::vector<NVertex*>::iterator _vertex_it;

  Eigen::Vector4d getRandomPoint();
  Eigen::Vector4d getRandomPointGoalBiased();
  void getReachable(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices);
  NVertex *nearestNeigbor(std::vector<NVertex*>& reachable, Eigen::Vector4d& point);
  bool isValidEdge(NVertex *vertex);
  bool isGoalVertex(NVertex *vertex);

public:
  Needlerrt();
  ~Needlerrt();
  void makeStep();
  bool isFinished();
  void setGoalArea(Eigen::Vector4d center,double ray);
  double getSizeGoalArea();
  const Eigen::Vector4d& getCenterGoalArea();

  void insertObstacle(Obstacle& obstacle);
  const std::vector<Obstacle>& getObstacles();
  NVertex *getGoalVertex();
  NeedleTree &getNeedleTree();
};



#endif // NEEDLERRT_H
