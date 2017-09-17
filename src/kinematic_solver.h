#ifndef KINEMATIC_SOLVER_H
#define KINEMATIC_SOLVER_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "data_package.h"
#include "kinematic.h"
#include "constants_indices.h"

using CppAD::AD;

class KinematicObjFunctionSet{

public:

  using Dvector = CPPAD_TESTVECTOR(double); // required to be member for ipopt
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  KinematicObjFunctionSet(VectorXd coeffs_in) :
    coeffs_ (coeffs_in) {};
  void operator()(ADvector& fg, const ADvector &vars);

  // member variables

  VectorXd coeffs_;
};

using Dvector = CPPAD_TESTVECTOR(double);
using ADvector = CPPAD_TESTVECTOR(AD<double>);

class KinematicSolver{

public:

  Dvector GetVariables (const VectorXd &State_in) const;
  Dvector GetVariableBounds (bool ForLower) const;
  Dvector GetObjFunctionBounds (const VectorXd &State_in) const;

  KinematicObjFunctionSet GetFunctor(const VectorXd &coeff_in) const;


private:
};


#endif /* KINEMATIC_SOLVER_H */
