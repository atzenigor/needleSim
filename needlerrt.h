#ifndef NEEDLERRT_H
#define NEEDLERRT_H

//#define PX 0.83
//#define PY 0.15
//#define PZ 0.53

#define BOUND_XY 1.0 // the min is -BOUND_XY
#define BOUND_Z_MAX 1.0
#define BOUND_Z_MIN 0.0
#define MIN_R 0.15  //must be double
#define VARIANCE 0.2 //the variance for the random variable that tunes the bias towards the goal.

#define _USE_MATH_DEFINES

#include "needletree.h"
#include "obstacle.h"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <random>
#include <cmath>

class RandomManager{
private:
    double _variance;
    std::mt19937 _gen;
    std::normal_distribution<> _disx;
    std::normal_distribution<> _disy;
    std::normal_distribution<> _disz;
public:
    RandomManager(Eigen::Vector4d center, double variance);
    double get_random_x();
    double get_random_y();
    double get_random_z();
};

class Needlerrt{
private:
    float bound_xy = 1.0; // the min is -BOUND_XY
    float bound_z_max = 1.0;
    float bound_z_min = 0.0;
    float min_r = 0.15;  //must be double
    float variance; //the variance for the random variable that tunes the bias towards the goal.


  NeedleTree _tree;
  bool _finished;
  NVertex *_goal_vertex_ptr;
  Eigen::Vector4d _goal_center;
  double _goal_ray_squared;
  double _goal_ray;
  float path_length = -1;
  RandomManager * _rm;  // it is initialized when setGoalArea(..) is called
  std::vector<Obstacle> _list_of_obstacles;

  Eigen::Vector4d getRandomPoint();
  Eigen::Vector4d getRandomPointGoalBiased();
  void getReachable(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices);
  NVertex *nearestNeigbor(std::vector<NVertex*>& reachable, Eigen::Vector4d& point);
  bool isValidEdge(NVertex *vertex);
  bool isGoalVertex(NVertex *vertex);

public:
  double px,py,pz;

  Needlerrt();
  ~Needlerrt();
  void makeStep();
  bool isFinished();
  void setGoalArea(Eigen::Vector4d center,double ray);
  double getSizeGoalArea();
  const Eigen::Vector4d& getCenterGoalArea();

  void insertObstacle(Obstacle obstacle);
  const std::vector<Obstacle>& getObstacles();
  NVertex *getGoalVertex();
  NeedleTree &getNeedleTree();

  float getPathLength();
};



#endif // NEEDLERRT_H
