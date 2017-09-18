#include "kinematic_solver.h"
#include "MPC.h"


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;

  KinematicSolver KSolver;

  auto Vars    = KSolver.GetVariables(state);
  auto VarsLwB = KSolver.GetVariableBounds(true  /*ForLower*/);
  auto VarsUpB = KSolver.GetVariableBounds(false /*ForLower*/);

  Dvector ObjLwB = KSolver.GetObjFunctionBounds(state);
  Dvector ObjUpB = ObjLwB;

  auto Functor = KSolver.GetFunctor(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, KinematicObjFunctionSet>(
      options,
      Vars,
      VarsLwB,
      VarsUpB,
      ObjLwB,
      ObjUpB,
      Functor,
      solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> result;
  result.push_back(solution.x[steer_Idx]);
  result.push_back(solution.x[a_Idx]);

  // MPC predicted path
  for (size_t i = 0; i < N-1; ++i){
    result.push_back(solution.x[x_Idx + i + 1]);
    result.push_back(solution.x[y_Idx + i + 1]);
  }

  return result;
}
