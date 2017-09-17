#include "kinematic_solver.h"

Dvector KinematicSolver::GetVariables (const VectorXd &State_in) const{

  Dvector Out(nVars);
  for (size_t i = 0; i < nVars; ++i){
    Out[i] = 0;
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
