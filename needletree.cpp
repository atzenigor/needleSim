#include "needletree.h"
#include "iostream"

using namespace std;
NVertex::NVertex(NVertex* parent, const Eigen::Matrix4d& g, UParam& param):
  _parent(parent),_g(g),_g_inv(g.inverse()),_param(param){
    if (parent != NULL){
        const double ctheta = cos(_param.theta);
        const double stheta = sin(_param.theta);
        double cphi;
        const double phi_max = _param.l / _param.r;
        const double phi_step = phi_max/DISCRETIZATION_CONST;
        Eigen::Vector4d v;
        for(double phi = 0; phi < phi_max+phi_step/2; phi += phi_step)
        {
            cphi = cos(phi);
            v << _param.r*ctheta*(1-cphi), _param.r*stheta*(1-cphi), _param.r*sin(phi), 1;
            _discretized.push_back(this->_parent->_g * v);
        }
    }
}

const UParam NVertex::getParam(){
  return _param;
}

NVertex *NVertex::getParent(){
  return _parent;
}
const std::vector<Eigen::Vector4d> &NVertex::getDiscretized(){
    return _discretized;
}

const Eigen::Vector4d NVertex::getPosition(){
  return _g.col(3);
}

const Eigen::Matrix4d& NVertex::getTransMatrix(){
  return _g;
}

const Eigen::Matrix4d& NVertex::getInvTransMatrix(){
  return _g_inv;
}

NeedleTree::NeedleTree():
  _list_of_vertex(){
  UParam a;
  _first_vertex = new NVertex(NULL,Eigen::Matrix4d::Identity(),a);
  this->addNVertex(_first_vertex);
}
NeedleTree::~NeedleTree(){
  this->_list_of_vertex.clear();
}

void NeedleTree::addNVertex(NVertex* v){
  _list_of_vertex.push_back(v);
}

std::vector<NVertex*>& NeedleTree::getListOfVertex(){
  return _list_of_vertex;
}
