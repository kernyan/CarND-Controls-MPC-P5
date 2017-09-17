#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "data_package.h"

constexpr double LF = 2.67; // distance between front of vehicle and centre of gravity
constexpr double MAX_STEER = 25;
constexpr double MAX_STEER_RAD = 25 * M_PI / 180;

const size_t nState = 6;

void TransformWayPoint(const DataPackage &DP_in,
		VectorXd &ptsx_transf, VectorXd &ptsy_transf);

double CalculateSteer (double delta_in);
double CalculateThrottle (double acc_in);

VectorXd GetState(const DataPackage &DP_in,
    const VectorXd &coef,
    double Latency_in);

#endif /* KINEMATIC_H */
