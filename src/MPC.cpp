#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"

using CppAD::AD;
using Eigen::VectorXd;

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 10;
double dt = 0.25;

// start positions for the state variables
uint x_start = 0;
uint y_start = x_start + N;
uint v_start = y_start + N;
uint psi_start = v_start + N;
uint cte_start = psi_start + N;
uint epsi_start = cte_start + N;

// Start positions for the actuators
uint delta_start = epsi_start + N;
uint a_start = delta_start + N;

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
    
    // Define the cost function
    fg[0] = 0;

    AD<double> v_ref = 35;

    // Punish deviation from reference speed to prevent the car from stopping
    // Punish cross-track error (deviation from the ideal lane)
    // Punish psi error (wrong angle of sight)
    for (int t = 0; t < N; ++t) {
      fg[0] += CppAD::pow(vars[v_start + t] - v_ref, 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Set up the constraints
    for(int t = 1; t  < N; ++t){
      // Values for t (leading +1 needed as fg vector starts with cost at fg[0])
      CppAD::AD<double>  x0 = vars[1 + x_start + t - 1];
      CppAD::AD<double>  y0 = vars[1 + y_start + t - 1];
      CppAD::AD<double>  v0 = vars[1 + v_start + t - 1];
      CppAD::AD<double>  psi0 = vars[1 + psi_start + t - 1];
      CppAD::AD<double>  cte0 = vars[1 + cte_start + t - 1];
      CppAD::AD<double>  epsi0 = vars[1 + epsi_start + t - 1];
      CppAD::AD<double>  delta0 = vars[1 + delta_start + t - 1];
      CppAD::AD<double>  a0 = vars[1 + a_start + t - 1];

      // Values for t+1
      CppAD::AD<double>  x1 = vars[1 + x_start + t - 1];
      CppAD::AD<double>  y1 = vars[1 + y_start + t - 1];
      CppAD::AD<double>  v1 = vars[1 + v_start + t - 1];
      CppAD::AD<double>  psi1 = vars[1 + psi_start + t - 1];
      CppAD::AD<double>  cte1 = vars[1 + cte_start + t - 1];
      CppAD::AD<double>  epsi1 = vars[1 + epsi_start + t - 1];

      // Function value and its derivative
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 + coeffs[3] * x0*x0*x0;
      AD<double> f0_diff = coeffs[1]+ 2* coeffs[2] * x0 + 3 * coeffs[3] * x0*x0;
      AD<double> psides0 = CppAD::atan(f0_diff);

      // Defining constraint values to compare with the prev. defined ones
      // As we defined the contraints to be 0, the solver needs to
      // finde such delta and a values that the following fg values
      // match 0 for all states after the initial one.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0));
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0));
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + psi_start + t] = psi1 -  (psi0 - (v0/Lf) * delta0 * dt);
      fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psides0 + (v0/Lf) * delta0 * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {

  const std::string TRACK_DATA_FILENAME = "../lake_track_waypoints.csv";

  // Load the input file
  std::ifstream track_data_file;
  track_data_file.open(TRACK_DATA_FILENAME);

  // Check if file was found
  if(!track_data_file.is_open()) {
    std::cout << "Exiting program as track data file not found in location: " << TRACK_DATA_FILENAME << std::endl;
    exit(-1);
  }

  // Read in the track data
  std::string str_p_x;
  std::string str_p_y;
  double p_x;
  double p_y;
  std::vector<double> track_data_x_;
  std::vector<double> track_data_y_;

  while(track_data_file.good()) {

    // Read in two cells of the line
    getline(track_data_file, str_p_x, ',');
    getline(track_data_file, str_p_y, ',');
    
    // Convert to double and save
    std::stringstream(str_p_x) >> p_x;
    std::stringstream(str_p_y) >> p_y;

    track_data_x_.push_back(p_x);
    track_data_y_.push_back(p_y);
  }
  std::cout << "Parsed " << track_data_x_.size() << " waypoints from file " << TRACK_DATA_FILENAME << std::endl;

  // Compute the coefficients by fitting a 3 order polynomial to the waypoints
  VectorXd coeffs_ = polyfit(Eigen::Map<Eigen::VectorXd>(track_data_x_.data(), track_data_x_.size()), 
                              Eigen::Map<Eigen::VectorXd>(track_data_y_.data(), track_data_y_.size()),
                              3);

}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Read in the initial state
  double x = state(0);
  double y = state(1);
  double v = state(2);
  double psi = state(3);
  double cte = state(4);
  double epsi = state(5);

  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = N * 6 + (N-1) * 2;
  /**
   * Set the number of constraints
   */
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);



  /**
   * Set lower and upper limits for variables.
   */
  for (int t = 0; t < N; ++t) {
    vars_lowerbound[x_start + t] = std::numeric_limits<double>::min();
    vars_upperbound[x_start + t] = std::numeric_limits<double>::max();

    vars_lowerbound[y_start + t] = std::numeric_limits<double>::min();
    vars_upperbound[y_start + t] = std::numeric_limits<double>::max();

    vars_lowerbound[v_start + t] = std::numeric_limits<double>::min();
    vars_upperbound[v_start + t] = std::numeric_limits<double>::max();

    vars_lowerbound[psi_start + t] = -M_PI;
    vars_upperbound[psi_start + t] = M_PI;

    vars_lowerbound[cte_start + t] = std::numeric_limits<double>::min();
    vars_upperbound[cte_start + t] = std::numeric_limits<double>::max();

    vars_lowerbound[cte_start + t] = std::numeric_limits<double>::min();
    vars_upperbound[cte_start + t] = std::numeric_limits<double>::max();

    vars_lowerbound[delta_start + t] = -deg2rad(25);
    vars_upperbound[delta_start + t] = deg2rad(25);

    vars_lowerbound[a_start + t] = -1.;
    vars_upperbound[a_start + t] = 1.;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Lower bounds for initial state differ
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  // Upper bounds for initial state differ
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[psi_start] = psi;
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