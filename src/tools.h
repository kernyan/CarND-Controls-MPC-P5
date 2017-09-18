#ifndef TOOLS_H
#define TOOLS_H

#include "data_package.h"
#include <string>
#include "Eigen-3.3/Eigen/QR"

using namespace std;

string hasData(string s);
double polyeval(VectorXd coeffs, double x);
VectorXd polyfit(VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif /* TOOLS_H */
