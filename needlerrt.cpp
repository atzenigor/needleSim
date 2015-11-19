#include "needlerrt.h"

#include <iostream>

Needlerrt::Needlerrt():
  _tree(),
  _finished(false),
  _goal_vertex_ptr(NULL),
  _goal_center(),
  _goal_ray_squared(0.0),
  _list_of_obstacles(),
  _vertex_it(NULL){
}
Needlerrt::insertObstacle(Obstacle o){
    _list_of_obstacles.push_back(o);
}

const std::vector<Obstacle>& getObstacles(){
    return _list_of_obstacles;
}

//Clears the list of obstacle
Needlerrt::~Needlerrt(){
  _list_of_obstacles.clear();
}


// Provide a random vector with uniform probability distibytion on the world space
Eigen::Vector4d Needlerrt::getRandomPoint(){
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> disxy(WORLD_BOUND_XY_MIN,WORLD_BOUND_XY_MAX);
      std::uniform_real_distribution<> disz(WORLD_BOUND_Z_MIN,WORLD_BOUND_Z_MAX);
      Eigen::Vector4d v;
      v << disxy(gen),disxy(gen),disz(gen),1;
//      v << 1.0,1.0,3.0,1.0;
      return v;
}

// Find all the reachable vertices that can reach the given random point
void Needlerrt::fillReachableVertices(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices){
  list_of_reachable_vertices.clear();
  for (std::vector<NVertex*>::iterator it = this->_tree.getListOfVertex().begin() ; it != _tree.getListOfVertex().end(); ++it){
      Eigen::Vector4d p = (*it)->getInvTransMatrix() * random_point;
      double squres = pow(p.x(),2)+pow(p.y(),2);
      if( p.z() >= sqrt(2*MIN_R*sqrt(squres)-squres))
        list_of_reachable_vertices.push_back(*it);
    }
}

NVertex* Needlerrt::nearestNeigbor(std::vector<NVertex*>& reachable, Eigen::Vector4d& point){
  double new_norm;
  double norm = (reachable[0]->getPosition()-point).squaredNorm();
  NVertex* curr_near_vertex = reachable[0];
  for (std::vector<NVertex*>::iterator it = reachable.begin()+1 ; it != reachable.end(); ++it){
      new_norm = ((*it)->getPosition() - point).squaredNorm();
      if( norm < new_norm){
        norm = new_norm;
        curr_near_vertex = *it;
      }
    }
  return curr_near_vertex;
}

//check if the arc defined by the vertex is free from collision
bool Needlerrt::isValidEdge(NVertex* vertex){
  for(uint i = 0; i< vertex->getDiscretized().size(); i++)
    {
      for(uint i=0;i<_list_of_obstacles.size();i++){
//          if (list_of_obstacle_[i]->collision(vertex->getDiscretized()[j]))
//            return 0;
        }
    }
  //if it goes out of the for loop it means that no collisions are detected.
  return 1;

}
bool Needlerrt::isGoalVertex(NVertex *vertex){
  return (vertex->getPosition() - _goal_center).squaredNorm() < _goal_ray_squared;
}

void Needlerrt::setGoalArea(Eigen::Vector4d center, double ray){
  _goal_center= center;
  _goal_ray_squared= ray * ray;
}

void Needlerrt::makeStep(){
  if (_finished)
    return;
  std::vector<NVertex*> list_of_reachable_vertices;
  Eigen::Vector4d random_point;
  NVertex *nearest_neigbor;
  Eigen::Vector4d p;
  UParam u;
  Eigen::Matrix4d g;
  Eigen::Matrix4d gwn;
  NVertex* vertex_ptr = NULL;
  do
    {
      if(vertex_ptr != NULL)
          delete vertex_ptr;
      while( list_of_reachable_vertices.empty())
        {
          random_point = getRandomPoint();
          fillReachableVertices(random_point, list_of_reachable_vertices);
        }
      nearest_neigbor = nearestNeigbor(list_of_reachable_vertices, random_point);
      p = nearest_neigbor->getInvTransMatrix() * random_point;
      u.theta = atan2(p.y(),p.x());
      double stheta =  sin(u.theta);
      double ctheta =  cos(u.theta);
      u.r = (pow(p.y(),2) +  pow(p.z()*stheta,2))/(2*stheta);
      double phi = atan2(K_CONST * p.z(),1-(K_CONST * p.y()/stheta));
      if (u.r < 0)
          u.r = -u.r;
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
      _goal_vertex_ptr = vertex_ptr;
  }
}

NeedleTree& Needlerrt::getNeedleTree(){
    return _tree;
}


