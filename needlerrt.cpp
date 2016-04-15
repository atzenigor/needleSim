#include "needlerrt.h"

using namespace std;
using namespace Eigen;

RandomManager::RandomManager(Eigen::Vector4d center, double variance):
    _variance(variance),
    _gen(std::random_device{}()),
    _disx(center[0],variance),
    _disy(center[1],variance),
    _disz(center[2],variance)

{}

double RandomManager::get_random_x(){
    return _disx(_gen);
}
double RandomManager::get_random_y(){
    return _disy(_gen);
}
double RandomManager::get_random_z(){
    return _disz(_gen);
}


Needlerrt::Needlerrt():
  _tree(),
  _finished(false),
  _goal_vertex_ptr(NULL),
  _goal_center(),
  _goal_ray_squared(0.0),
  _list_of_obstacles()
{}

void Needlerrt::insertObstacle(Obstacle& o){
    _list_of_obstacles.push_back(o);
}

const vector<Obstacle>& Needlerrt::getObstacles(){
    return _list_of_obstacles;
}

//Clears the list of obstacle
Needlerrt::~Needlerrt(){
  _list_of_obstacles.clear();
  delete _rm;
}


// Provide a random vector with uniform probability distibytion on the world space
Vector4d Needlerrt::getRandomPoint(){
      random_device rd;
      mt19937 gen(rd());
      uniform_real_distribution<> disxy(-BOUND_XY,BOUND_XY);
      uniform_real_distribution<> disz(BOUND_Z_MIN,BOUND_Z_MAX);
      Vector4d v;
      v << disxy(gen),disxy(gen),disz(gen),1;
      return v;
}

Vector4d Needlerrt::getRandomPointGoalBiased(){
  Vector4d v;
  v << _rm->get_random_x(), _rm->get_random_y(), _rm->get_random_z(),1;
  while(v[0] > BOUND_XY || v[0] < -BOUND_XY)
      v[0] = _rm->get_random_x();
  while(v[1] > BOUND_XY || v[1] < -BOUND_XY)
      v[1] = _rm->get_random_y();
  while(v[2] > BOUND_Z_MAX || v[2] < -BOUND_Z_MIN)
      v[2] = _rm->get_random_z();

//  v << PX, PY, PZ, 1;

  return v;
}

// Find all the reachable vertices that can reach the given random point
void Needlerrt::getReachable(const Vector4d& random_point, vector<NVertex*>& list_of_reachable_vertices){
  list_of_reachable_vertices.clear();
  for (vector<NVertex*>::iterator it = this->_tree.getListOfVertex().begin() ; it != _tree.getListOfVertex().end(); ++it){
      Vector4d p = (*it)->getInvTransMatrix() * random_point;
      double squres = pow(p[0],2)+pow(p[1],2);
//      if(p.z() >= sqrt(2*MIN_R*sqrt(squres)-squres))
      if(pow(p.z(), 2) >= 2*MIN_R*sqrt(squres)-squres)
          list_of_reachable_vertices.push_back(*it);
    }
}

NVertex* Needlerrt::nearestNeigbor(vector<NVertex*>& reachable, Vector4d& point){
  double new_norm;
  double norm = (reachable[0]->getPosition()-point).squaredNorm();
  NVertex* curr_near_vertex = reachable[0];
  for (vector<NVertex*>::iterator it = reachable.begin()+1 ; it != reachable.end(); ++it){
      new_norm = ((*it)->getPosition() - point).squaredNorm();
      if( norm > new_norm){
        norm = new_norm;
        curr_near_vertex = *it;
      }
    }
  return curr_near_vertex;
}

//check if the arc defined by the vertex is free from collision
bool Needlerrt::isValidEdge(NVertex* vertex){
  for(vector<Vector4d>::const_iterator it = vertex->getDiscretized().begin(); it != vertex->getDiscretized().end(); ++it)
    {
      for(uint i=0;i<_list_of_obstacles.size();i++){
          if (_list_of_obstacles[i].contains(*it))
            return 0;
        }
    }
  //if it goes out of the for loop it means that no collisions occured.
  return 1;

}
bool Needlerrt::isGoalVertex(NVertex *vertex){
  return (vertex->getPosition() - _goal_center).squaredNorm() < _goal_ray_squared;
}

void Needlerrt::setGoalArea(Vector4d center, double ray){
  _goal_center= center;
  _goal_ray = ray;
  _goal_ray_squared= ray * ray;

  _rm = new RandomManager(center, VARIANCE);
}
double Needlerrt::getSizeGoalArea(){
    return _goal_ray;
}

const Vector4d& Needlerrt::getCenterGoalArea(){
    return _goal_center;
}

void Needlerrt::makeStep(){
  if (_finished)
    return;
  vector<NVertex*> reachable;
  Vector4d random_point;
  NVertex *nearest_neigbor;
  Vector4d p;
  UParam u;
  Matrix4d g;
  Matrix4d gwn;
  NVertex* vertex_ptr = NULL;
  do
  {
      if(vertex_ptr != NULL)
          delete vertex_ptr;

      for(reachable.clear(); reachable.empty(); getReachable(random_point, reachable))
          random_point = getRandomPointGoalBiased();
//          random_point = getRandomPoint();

      nearest_neigbor = nearestNeigbor(reachable, random_point);
      p = nearest_neigbor->getInvTransMatrix() * random_point;
      u.theta = atan2(p.y(),p.x());
      double stheta =  sin(u.theta);
      double ctheta =  cos(u.theta);
      double px2 = pow(p.x(),2);
      double py2 = pow(p.y(),2);
      double pz2 = pow(p.z(),2);
      u.r = (px2 + py2 + pz2) / (2*sqrt(px2+py2));
      double phi = atan2(p.z(),u.r-sqrt(px2+py2));
      double cphi = cos(phi);
      double sphi = sin(phi);
      u.l = u.r * phi;

      g<<-stheta, -ctheta*cphi, ctheta*sphi, u.r*ctheta*(1-cphi),
          ctheta, -stheta*cphi, stheta*sphi , u.r*stheta*(1-cphi),
               0,         sphi,        cphi,            u.r*sphi,
               0,            0,           0,                   1;
      gwn = nearest_neigbor->getTransMatrix()*g;
      vertex_ptr = new NVertex(nearest_neigbor,gwn,u);
  }
  while(! isValidEdge(vertex_ptr));

  _tree.addNVertex(vertex_ptr);
  if(isGoalVertex(vertex_ptr)){
      _finished = 1;
      this->_goal_vertex_ptr = vertex_ptr;
  }
}
bool Needlerrt::isFinished(){
    return _finished;
}

NVertex *Needlerrt::getGoalVertex(){
    return _goal_vertex_ptr;
}

NeedleTree& Needlerrt::getNeedleTree(){
    return _tree;
}


