#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <limits>

using CppAD::AD;
using Eigen::VectorXd;
using std::numeric_limits;

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 6;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int v_start = psi_start + N;
int cte_start = v_start + N;
int epsi_start = cte_start + N;
int delta_start = epsi_start + N;
int a_start = delta_start + (N-1);

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * TODO: implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = 6 * N + 2 * (N-1);
  /**
   * TODO: Set the number of constraints
   */
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /**
   * TODO: Set lower and upper limits for variables.
   */
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -numeric_limits<double>::infinity();
    vars_upperbound[i] = numeric_limits<double>::infinity();
  }

  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */
  return {};
}
