#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef NEEDLETREE_H
#define NEEDLETREE_H

class NEdge;
class NVertex;

struct UParam {
  double l;
  double r;
  double theta;
};


//It is a vertex of the tree. It contains the parameter of its edge with its parent.
class NVertex {
private:
  NVertex * parent_;
  const Eigen::Matrix4d g_; // 4x4 transformation matrix
  const Eigen::Matrix4d g_inv_;
  const UParam param_;  // the parameter of the arc from here to the parent.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NVertex(NVertex * parent, const Eigen::Matrix4d &g, UParam& param);
  NVertex * getParent();
  const Eigen::Matrix4d &getTransMatrix();
  const Eigen::Matrix4d &getInvTransMatrix();
  const Eigen::Vector4d &getPosition();
  const UParam getParam();
};

class NeedleTree{
private:
  NVertex* first_vertex_;
  std::vector<NVertex*> list_of_vertex_;
public:
  NeedleTree();
  ~NeedleTree();
  void addNVertex(NVertex *v);
  std::vector<NVertex*>& getListOfVertex();
};


#endif // NEEDLETREE_H
