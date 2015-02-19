#include "needletree.h"


NVertex::NVertex(NVertex* parent, const Eigen::Matrix4d& g, UParam& param):
  parent_(parent),
  g_(g),
  g_inv_(g.inverse()),
  param_(param){
}

const UParam NVertex::getParam(){
  return param_;
}

NVertex *NVertex::getParent(){
  return parent_;
}

const Eigen::Vector4d& NVertex::getPosition(){
  return g_.col(3);
}

const Eigen::Matrix4d& NVertex::getTransMatrix(){
  return g_;
}

const Eigen::Matrix4d& NVertex::getInvTransMatrix(){
  return g_inv_;
}

NeedleTree::NeedleTree():
  list_of_vertex_(){
  UParam a;
  first_vertex_ = new NVertex(NULL,Eigen::Matrix4d::Identity(),a);
  this->addNVertex(first_vertex_);
}
NeedleTree::~NeedleTree(){
  this->list_of_vertex_.clear();
}

void NeedleTree::addNVertex(NVertex* v){
  list_of_vertex_.push_back(v);
}

std::vector<NVertex*>& NeedleTree::getListOfVertex(){
  return list_of_vertex_;
}
