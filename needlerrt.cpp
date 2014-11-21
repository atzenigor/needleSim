#include "needlerrt.h"




void Needlerrt::randomFill(Eigen::Vector4d& v){
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> disxy(-1,1);
      std::uniform_real_distribution<> disz(0,1);
      v << disxy(gen),disxy(gen),disz(gen),1;
}
