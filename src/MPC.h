#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/cppad.hpp>

using CppAD::AD;

class MPC {
 private:

 bool has_prev_vars_;
 CppAD::vector<double> prev_vars_;

 // Coefficients of polynomial fitted to the waypoints
 //Eigen::VectorXd coeffs_;


 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);
};

#endif  // MPC_H
