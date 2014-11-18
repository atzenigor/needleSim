#include "needletree.h"

NEdge::NEdge(NVertex& source, NVertex& dest, u& param):
  source_(source),
  dest_(dest),
  param_(param)
{}

NEdge::getSource(){
  return source_;
}

NEdge::getDest(){
  return dest_;
}
NEdge::getParam(){
  return param_;
}

NVertex::NVertex(NEdge& parent, Eigen::Vector3d g):
  parent_(parent),
  g_(g)
{}

NVertex::getParent(){
  return parent_;
}

Eigen::Vector3d &NVertex::getPosition(){
  return position_;
}

Eigen::Dense &NVertex::getTransMatrix(){
  return g_;
}

NeedleTree::NeedleTree():
  listOfVertex(),
  listOfEdges()
{}

addNVertex


/*
NVertex1::NVertex1(NEdge1 &parentEdge, u& param):
  parentEdge_(parentEdge),
  param_(param)
{}

NEdge &NVertex1::getParentEdge()
{
  return parentEdge_;
}

u &NVertex1::getParam()
{
  return param_;
}


NEdge1::NEdge1(NVertex1 &source, NVertex1 &dest, Eigen::Dense& g):
  source_(source),
  dest_(dest),
  g_(g)
{}

NVertex1 &NEdge1::getSource()
{
  return source_;
}

NVertex1 &NEdge1::getDest()
{
  return dest_;
}

Eigen::Dense& NEdge1::getGMatrix()
{
  return g_;
}


NeedleTree::NeedleTree():
  listOfVertex(),
  listOfEdges()
{}

*/
