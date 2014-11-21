#include "needletree.h"


NVertex::NVertex(NVertex* parent, const Eigen::Matrix4d& g, const Eigen::Vector4d& position, uParam& param):
  parent_(parent),
  g_(g),
  position_(position),
  param_(param){
}


uParam& NVertex::getParam(){
  return param_;
}

NVertex *NVertex::getParent(){
  return parent_;
}

Eigen::Vector4d& NVertex::getPosition(){
  return position_;
}

Eigen::Matrix4d& NVertex::getTransMatrix(){
  return g_;
}

NeedleTree::NeedleTree():
  listOfVertex_(){
  uParam a;
  initVertex_ = new NVertex(NULL,Eigen::Matrix4d::Identity(),Eigen::Vector4d::Identity().reverse(),a);
  this->addNVertex(initVertex_);
}
NeedleTree::~NeedleTree(){
  this->listOfVertex_.clear();
}

void NeedleTree::addNVertex(NVertex* v){
  listOfVertex_.push_back(v);
}
