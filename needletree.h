#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef NEEDLETREE_H
#define NEEDLETREE_H

class NEdge;
class NVertex;

struct u {
  double l;
  double r;
  double theta;
};

class NEdge{
private:
  NVertex& source_;
  NVertex& dest_;
  u& param_;

public:
  NEdge(NVertex& source, NVertex& dest, u &param);
  NVertex& getSource();
  NVertex& getDest();
  u getParam();
};

class NVertex {
private:
  NEdge * parent_;
  Eigen::Matrix4d g_; // 4x4 transformation matrix
  Eigen::Vector4d position_;

public:
  NVertex(NEdge * parent, const Eigen::Matrix4d &g, const Eigen::Vector4d &position);
  NEdge& getParent();
  Eigen::Matrix4d& getTransMatrix();
  Eigen::Vector4d& getPosition();
};


class NeedleTree{
private:
  std::vector<NVertex> listOfVertex;
  std::vector<NEdge> listOfEdges;
  NVertex initVertex_;

public:
  NeedleTree();
  void addNVertex(NVertex& v);
  void addNEdge(NEdge& e);

};


#endif // NEEDLETREE_H
