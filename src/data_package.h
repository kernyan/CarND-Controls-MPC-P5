#ifndef DATA_PACKAGE_H
#define DATA_PACKAGE_H

#include "Eigen-3.3/Eigen/Core"
#include <iostream>
#include "json.hpp"
#include <vector>

using namespace std;
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;

enum DATA_INPUT {

// do not change order

  PX = 0,
  PY,
  PSI,
  VEL,
  ACC,
  STR
};

struct DataPackage {

  vector<double> WayPointX;
  vector<double> WayPointY;
  vector<double> Input;

  DataPackage(json Data);
};



#endif /* DATA_PACKAGE_H */
