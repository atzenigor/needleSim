#ifndef NEEDLERRT_H
#define NEEDLERRT_H
#include "needletree.h"
#include "obstacle.h"
#include "eigen3/Eigen/Dense"
#define _USE_MATH_DEFINES
#include <math.h>

#define MIN_R 0.1  //must be a double
#define K_CONST 0.3

// boundary of world space
#define WORLD_BOUND_XY_MAX 1.0
#define WORLD_BOUND_XY_MIN -1.0
#define WORLD_BOUND_Z_MAX 1.0
#define WORLD_BOUND_Z_MIN 0.0

#include "obstacle.h"
#include "math.h"

class Needlerrt{
private:
  NeedleTree _tree;
  bool _finished;
  NVertex *_goal_vertex_ptr;
  Eigen::Vector4d _goal_center;
  double _goal_ray_squared;
  std::vector<Obstacle> _list_of_obstacles;
  std::vector<NVertex*>::iterator _vertex_it;

  Eigen::Vector4d getRandomPoint();
  void fillReachableVertices(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices);
  NVertex *nearestNeigbor(std::vector<NVertex*>& reachable, Eigen::Vector4d& point);
  bool isValidEdge(NVertex *vertex);
  bool isGoalVertex(NVertex *vertex);

public:
  Needlerrt();
  ~Needlerrt();
  void makeStep();
  bool isFinished();
  void setGoalArea(Eigen::Vector4d center,double ray);

  void insertObstacle(Obstacle obstacle);
  std::vector<Obstacle> getObstacles();

  NeedleTree &getNeedleTree();
};



#endif // NEEDLERRT_H
