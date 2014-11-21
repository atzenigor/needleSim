#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef NEEDLETREE_H
#define NEEDLETREE_H

class NEdge;
class NVertex;

struct uParam {
  double l;
  double r;
  double theta;
};

class NVertex {
private:
  NVertex * parent_;
  Eigen::Matrix4d g_; // 4x4 transformation matrix
  Eigen::Vector4d position_;
  uParam param_;  // the parameter of the arc from here to the parent.

public:
  NVertex(NVertex * parent, const Eigen::Matrix4d &g, const Eigen::Vector4d &position, uParam& param);
  NVertex * getParent();
  Eigen::Matrix4d& getTransMatrix();
  Eigen::Vector4d& getPosition();
  uParam& getParam();
};

class NeedleTree{
private:
  NVertex* initVertex_;
  std::vector<NVertex*> listOfVertex_;
public:
  NeedleTree();
  ~NeedleTree();
  void addNVertex(NVertex *v);
};


#endif // NEEDLETREE_H
