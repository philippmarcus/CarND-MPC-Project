#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <cppad/cppad.hpp>

using Eigen::VectorXd;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x);

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

#endif  // HELPERS_H