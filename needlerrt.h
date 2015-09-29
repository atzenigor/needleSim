#ifndef NEEDLERRT_H
#define NEEDLERRT_H
#include "needletree.h"
#include "obstacle.h"
#include "eigen3/Eigen/Dense"

#define MIN_R 0.5  //must be a double
#define K_CONST 5.0

// boundary of world space
#define WORLD_BOUND_XY_MAX 1
#define WORLD_BOUND_XY_MIN -1
#define WORLD_BOUND_Z_MAX 1
#define WORLD_BOUND_Z_MIN 0

#include "obstacle.h"
#include "math.h"

struct Arc{
  const Eigen::Vector4d& start;
  const Eigen::Vector4d& end;
};

class Needlerrt{
private:
  NeedleTree _tree;
  bool finished_;
  NVertex *goal_vertex_ptr_;
  Eigen::Vector4d goal_center_;
  double goal_ray_squared_;
  std::vector<Obstacle*> list_of_obstacle_;
  std::vector<NVertex*>::iterator vertex_it_;

  static void fillRandom(Eigen::Vector4d& v);
  void fillReachableVertices(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices);
  NVertex *nearestNeigbor(std::vector<NVertex*>& reachable, Eigen::Vector4d& point);
  bool isValidEdge(NVertex *vertex);
  bool isGoalVertex(NVertex *vertex);

public:
  Needlerrt();
  ~Needlerrt();
  void makeStep();
  bool isFinished();
  void setGoalArea(Eigen::Vector4d& center,double ray);
  void insertObstacle(Obstacle& obstacle);

  NeedleTree &getNeedleTree();
};



#endif // NEEDLERRT_H
