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
  NEdge(NVertex& source,NVertex& dest, u& param);
  NVertex& getSource();
  NVertex& getDest();
  u& getParam();
};

class NVertex {
private:
  NEdge& parent_;
  Eigen::Dense& g_; // 4x4 transformation matrix
  Eigen::Vector3d position_;

public:
  NVertex(NEdge& parent, Eigen::Dense& g);
  NEdge& getParent();
  Eigen::Dense& getPosition();

};


class NeedleTree{
public:
  NeedleTree();
  void addNVertex(NVertex& v);
  void addNEdge(NEdge& e);
private:
  NEdge initEdge;
  std::vector<NVertex> listOfVertex;
  std::vector<NEdge> listOfEdges;
};


#endif // NEEDLETREE_H
