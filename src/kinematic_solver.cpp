#include "kinematic_solver.h"


/*
 * KinematicFunctor class
 * */

AD<double> KinematicObjFunctionSet::CalculateCost(const ADvector &Vars) const{

  AD<double> Cost = 0;


  for (size_t i = 0; i < N; ++i){
    // minimize deviation from desired state
    Cost += 1000 * CppAD::pow(Vars[CTE_Idx+i ] - target_CTE, 2);
    Cost += 1000 * CppAD::pow(Vars[epsi_Idx+i] - target_epsi, 2);
    Cost += CppAD::pow(Vars[v_Idx+i]    - target_v, 2);

    if (i < N-1){
      // minimize use of actuators
      Cost += CppAD::pow(Vars[steer_Idx+i], 2);
      Cost += CppAD::pow(Vars[a_Idx+i]    , 2);
    }

    if (i < N-2){
      // minimize large changes in actuators
      Cost += CppAD::pow(Vars[steer_Idx + i + 1] - Vars[steer_Idx + i], 2);
      Cost += CppAD::pow(Vars[a_Idx     + i + 1] - Vars[a_Idx     + i], 2);
    }
  }

  return Cost;
}

void KinematicObjFunctionSet::operator()(ADvector& fg, const ADvector &Vars){

  fg[0] = CalculateCost(Vars);

  // for initial input, no time has elapsed, hence no change
  fg[1 + x_Idx   ] = Vars[x_Idx];
  fg[1 + y_Idx   ] = Vars[y_Idx];
  fg[1 + psi_Idx ] = Vars[psi_Idx];
  fg[1 + v_Idx   ] = Vars[v_Idx];
  fg[1 + CTE_Idx ] = Vars[CTE_Idx];
  fg[1 + epsi_Idx] = Vars[epsi_Idx];

  // Motion model
  // We want our solver to produce states that obeys our motion model
  // as such, we constrain the solver such that the predicted state from previous state
  // converges to the current state
  for (size_t i = 0; i < N-1; ++i){

    AD<double> x1    = Vars[x_Idx    + i + 1];
    AD<double> y1    = Vars[y_Idx    + i + 1];
    AD<double> psi1  = Vars[psi_Idx  + i + 1];
    AD<double> v1    = Vars[v_Idx    + i + 1];
    AD<double> CTE1  = Vars[CTE_Idx  + i + 1];
    AD<double> epsi1 = Vars[epsi_Idx + i + 1];

    AD<double> x0    = Vars[x_Idx    + i];
    AD<double> y0    = Vars[y_Idx    + i];
    AD<double> psi0  = Vars[psi_Idx  + i];
    AD<double> v0    = Vars[v_Idx    + i];
    AD<double> CTE0  = Vars[CTE_Idx  + i];
    AD<double> epsi0 = Vars[epsi_Idx + i];

    AD<double> steer0 = Vars[steer_Idx + i];
    AD<double> a0     = Vars[a_Idx     + i];

    AD<double> f0 = coeffs_[0]
                  + coeffs_[1] * x0
                  + coeffs_[2] * x0 * x0
                  + coeffs_[3] * x0 * x0 * x0;

    AD<double> g0 = CppAD::atan(     coeffs_[1]
                               + 2 * coeffs_[2] * x0
                               + 3 * coeffs_[3] * x0 * x0);

    fg[1 + x_Idx    + i + 1] = x1    - (x0        + v0 * CppAD::cos(psi0)*dt);
    fg[1 + y_Idx    + i + 1] = y1    - (y0        + v0 * CppAD::sin(psi0)*dt);
    fg[1 + psi_Idx  + i + 1] = psi1  - (psi0      + v0 * (-steer0) / LF*dt);
    fg[1 + v_Idx    + i + 1] = v1    - (v0        + a0 * dt);
    fg[1 + CTE_Idx  + i + 1] = CTE1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0)*dt));
    fg[1 + epsi_Idx + i + 1] = epsi1 - ((psi0 - g0) + v0 * (-steer0) / LF*dt);
  }
}

/*
 * KinematicSolver class
 * */

Dvector KinematicSolver::GetVariables (const VectorXd &State_in) const{

  Dvector Out(nVars);
  for (size_t i = 0; i < nVars; ++i){
    Out[i] = 0.0;
  }

  Out[x_Idx]    = State_in[STATE_X];
  Out[y_Idx]    = State_in[STATE_Y];
  Out[psi_Idx]  = State_in[STATE_PSI];
  Out[v_Idx]    = State_in[STATE_V];
  Out[CTE_Idx]  = State_in[STATE_CTE];
  Out[epsi_Idx] = State_in[STATE_EPSI];

  return Out;
}


Dvector KinematicSolver::GetVariableBounds (bool ForLower) const{

  Dvector Out(nVars);
  double Sign = (ForLower) ? -1.0 : 1.0;

  for (size_t i = 0; i < steer_Idx; ++i){
    Out[i] = Sign * POS_INF;
  }

  for (size_t i = steer_Idx; i < a_Idx; ++i){
    Out[i] = Sign * MAX_STEER_RAD * LF;
  }

  for (size_t i = a_Idx; i < nVars; ++i){
    Out[i] = Sign;
  }

  return Out;
}

Dvector KinematicSolver::GetObjFunctionBounds (const VectorXd &State_in) const{

  // Values that we want the solver to match in the objective functions
  // except for initial input, all future objective functions has target of 0

  Dvector Out(nObjFuncs);
  for (size_t i = 0; i < nObjFuncs; ++i){
    Out[i] = 0.0;
  }

  Out[x_Idx]    = State_in[STATE_X];
  Out[y_Idx]    = State_in[STATE_Y];
  Out[psi_Idx]  = State_in[STATE_PSI];
  Out[v_Idx]    = State_in[STATE_V];
  Out[CTE_Idx]  = State_in[STATE_CTE];
  Out[epsi_Idx] = State_in[STATE_EPSI];

  return Out;
}

KinematicObjFunctionSet KinematicSolver::GetFunctor(const VectorXd &coeff_in) const{

  return KinematicObjFunctionSet(coeff_in);
}
