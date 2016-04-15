#ifndef NEEDLETREE_H
#define NEEDLETREE_H

#include <eigen3/Eigen/Dense>
#include <vector>

#define DISCRETIZATION_CONST 9
class NEdge;
class NVertex;

struct UParam {
  double l;
  double r;
  double theta;
};


//It is a vertex of the tree. It contains the parameter the arc between its parent and him.
class NVertex {
private:
  NVertex * _parent;
  const Eigen::Matrix4d _g; // 4x4 transformation matrix
  const Eigen::Matrix4d _g_inv;
  const UParam _param;  // the parameter of the arc from this vertex to the parent.
  std::vector<Eigen::Vector4d> _discretized; // vector of points in the arc.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NVertex(NVertex * parent, const Eigen::Matrix4d &g, UParam& param);
  NVertex * getParent();
  const std::vector<Eigen::Vector4d>& getDiscretized();
  const Eigen::Matrix4d &getTransMatrix();
  const Eigen::Matrix4d &getInvTransMatrix();
  const Eigen::Vector4d getPosition();
  const UParam getParam();
};

class NeedleTree{
private:
  NVertex* _first_vertex;
  std::vector<NVertex*> _list_of_vertex;
public:
  NeedleTree();
  ~NeedleTree();
  void addNVertex(NVertex *v);
  std::vector<NVertex*>& getListOfVertex();
};


#endif // NEEDLETREE_H
