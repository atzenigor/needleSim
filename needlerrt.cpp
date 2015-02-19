#include "needlerrt.h"


// boundary of world space
#define WORLD_BOUND_XY_MAX 1
#define WORLD_BOUND_XY_MIN -1
#define WORLD_BOUND_Z_MAX 1
#define WORLD_BOUND_Z_MIN 0

Needlerrt::Needlerrt():
  tree_(),
  finished_(false),
  goal_vertex_ptr_(NULL),
  goal_center_(),
  goal_ray_squared_(0.0),
  list_of_obstacle_(),
  vertex_it_(NULL){
}

//Clears the list of obstacle
Needlerrt::~Needlerrt(){
  list_of_obstacle_.clear();
}


// Fills v with a random vector with uniform probability distibytion on the world space
void Needlerrt::fillRandom(Eigen::Vector4d& v){
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> disxy(WORLD_BOUND_XY_MIN,WORLD_BOUND_XY_MAX);
      std::uniform_real_distribution<> disz(WORLD_BOUND_Z_MIN,WORLD_BOUND_Z_MAX);
      v << disxy(gen),disxy(gen),disz(gen),1;
}

// Find all the reachable vertices that can reach the given random point
void Needlerrt::fillReachableVertices(const Eigen::Vector4d& random_point, std::vector<NVertex*>& list_of_reachable_vertices){
  for (std::vector<NVertex*>::iterator it = this->tree_.getListOfVertex().begin() ; it != tree_.getListOfVertex().end(); ++it){
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

//check if the edge from the vertex passed and its parent is free from collision.
bool Needlerrt::isValidEdge(NVertex* vertex){
  vertex->getParam();
  const double r = vertex->getParam().r;
  const double l = vertex->getParam().l;
  const double ctheta = cos(vertex->getParam().theta);
  const double stheta = sin(vertex->getParam().theta);
  double cphi;
  const double phi_max = l / r;
  const double phi_step = phi_max/20;
  Eigen::Vector4d v;
  for(double phi = 0; phi < phi_max;phi += phi_step)
    {
      cphi = cos(phi);
      v<<- r*ctheta*(1-cphi), r*stheta*(1-cphi), r*sin(phi), 1;
      for(int i=0;i<list_of_obstacle_.size();i++){
//          if (list_of_obstacle_[i]->collision(v))
//            return 0;
        }
    }
  //if it go out of the for loop it means that no collisions are detected.
  return 1;

}
bool Needlerrt::isGoalVertex(NVertex *vertex){
  return (vertex->getPosition() - goal_center_).squaredNorm() < goal_ray_squared_;
}

void Needlerrt::setGoalArea(Eigen::Vector4d& center,double ray){
  goal_center_= center;
  goal_ray_squared_= ray * ray;
}

void Needlerrt::makeStep(){
  if (finished_)
    return;
  std::vector<NVertex*> list_of_reachable_vertices;
  Eigen::Vector4d random_point;
  NVertex *nearest_neigbor;
  Eigen::Vector4d p;
  UParam u;
  Eigen::Matrix4d g;
  Eigen::Matrix4d gwn;
  NVertex* vertex_ptr;
  do
    {
      while( ! list_of_reachable_vertices.empty());
        {
          fillRandom(random_point);
          fillReachableVertices(random_point,list_of_reachable_vertices);
        }
      nearest_neigbor = nearestNeigbor(list_of_reachable_vertices,random_point);
      p = nearest_neigbor->getInvTransMatrix() * random_point;
      u.theta = atan(p.y()/p.x());
      double stheta =  sin(u.theta);
      double ctheta =  cos(u.theta);
      u.r = (pow(p.y(),2) +  pow(p.z()*stheta,2))/(2*stheta);

      double phi = atan(K_CONST * p.z()/(1-(K_CONST * p.y()/stheta)));
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
  tree_.addNVertex(vertex_ptr);
  if(isGoalVertex(vertex_ptr)){
      finished_ = 1;
      goal_vertex_ptr_ = vertex_ptr;
  }
}


void Needlerrt::beginArcIter(){
  vertex_it_ = tree_.getListOfVertex().begin() + 1;
}

bool Needlerrt::isArcIterEnd(){
  return vertex_it_ == tree_.getListOfVertex().end();
}

Arc Needlerrt::nextArc(){
  NVertex *v = *vertex_it_;
  Arc tmp = {
    v->getParent()->getPosition(),
    v->getPosition()
  };
  vertex_it_++;
  return tmp;
}
