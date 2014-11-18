#include "needletree.h"

NEdge::NEdge(NVertex& source, NVertex& dest, u &param):
  source_(source),
  dest_(dest),
  param_(param)
{}

NVertex& NEdge::getSource(){
  return source_;
}

NVertex& NEdge::getDest(){
  return dest_;
}
u NEdge::getParam(){
  return param_;
}

NVertex::NVertex(NEdge* parent, const Eigen::Matrix4d& g, const Eigen::Vector4d& position):
  parent_(parent),
  g_(g),
  position_(position)
{}

NEdge& NVertex::getParent(){
  return *parent_;
}

Eigen::Vector4d& NVertex::getPosition(){
  return position_;
}

Eigen::Matrix4d& NVertex::getTransMatrix(){
  return g_;
}

NeedleTree::NeedleTree():
  listOfVertex(),
  listOfEdges(),
  initVertex_(NULL,Eigen::Matrix<double, 4, 4>::Identity(),Eigen::Matrix<double, 4, 1>::Identity().reverse())
{


}
