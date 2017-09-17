#ifndef TOOLS_H
#define TOOLS_H

#include "data_package.h"
#include <string>
#include "Eigen-3.3/Eigen/QR"

using namespace std;

/*
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
*/

string hasData(string s);
double polyeval(VectorXd coeffs, double x);
VectorXd polyfit(VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif /* TOOLS_H */
