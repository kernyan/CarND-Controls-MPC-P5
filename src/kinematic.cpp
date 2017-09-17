#include "kinematic.h"
#include "math.h"
#include "assert.h"

void TransformWayPoint(const DataPackage &DP_in,
		VectorXd &ptsx_transf, VectorXd &ptsy_transf){

  for (size_t i = 0; i < DP_in.WayPointX.size(); ++i){

    // relative distance
    double shift_x = DP_in.WayPointX[i] - DP_in.Input[PX];
    double shift_y = DP_in.WayPointY[i] - DP_in.Input[PY];
    double psi = DP_in.Input[PSI];

    // transform to vehicle co-ordinates
    ptsx_transf[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    ptsy_transf[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
  }
}

VectorXd GetState(const DataPackage &DP_in,
    const VectorXd &coef,
    double Latency_in){

  assert(Latency_in >= 0);

  VectorXd State = VectorXd::Constant(nState, 0.0);

  if (Latency_in == 0){

    // y = a0 + a1 * x + a2 * x^2 + a3 * x ^ 3
    // for x = 0, y is simply a0
    double CTE = coef[0]; // approximating shortest dist by using y instead of |(x,y) - (x',y')|

    // current psi - desired psi
    double epsi = -atan(coef[1]); // psi - arctan(f'(x)), psi being 0 after transform

    State << 0, 0, 0, DP_in.Input[VEL], CTE, epsi;
  } else {
    assert (0); // not defined yet
  }

  return State;
}

double CalculateSteer (double delta_in){

  return delta_in/(MAX_STEER_RAD*LF);
}

double CalculateThrottle (double acc_in){

  // as we use throttle as estimator for acceleration,
  // accelaration ~= throttle in this function
  return acc_in;
}
