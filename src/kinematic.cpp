#include "kinematic.h"
#include "math.h"

void TransformWayPoint(const DataPackage &DP_in,
		VectorXd &ptsx_transf, VectorXd &ptsy_transf){

  for (size_t i = 0; i < DP_in.WayPointX.size(); ++i){

    // relative distance
    double shift_x = DP_in.WayPointX[i] - DP_in.State[PX];
    double shift_y = DP_in.WayPointY[i] - DP_in.State[PY];
    double psi = DP_in.State[PSI];

    // transform to vehicle co-ordinates
    ptsx_transf[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    ptsy_transf[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
  }
}

