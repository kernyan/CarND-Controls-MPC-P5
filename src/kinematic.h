#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "data_package.h"

void TransformWayPoint(const DataPackage &DP_in,
		VectorXd &ptsx_transf, VectorXd &ptsy_transf);

#endif /* KINEMATIC_H */
