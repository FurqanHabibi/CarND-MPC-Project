#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/cppad.hpp>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
inline double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
inline CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Get polynomial derivative
inline Eigen::VectorXd polyderivative(Eigen::VectorXd coeffs) {
  Eigen::VectorXd coeffs_derivative(coeffs.size() - 1);
  for (int i = 1; i < coeffs.size(); i++) {
    coeffs_derivative[i - 1] = coeffs[i] * i;
  }
  return coeffs_derivative;
}

inline vector<vector<double>> transform_map2car(double car_theta, double car_x, double car_y, const vector<double> &ptsx, const vector<double> &ptsy) {
  assert(ptsx.size() == ptsy.size());
  assert(ptsx.size() > 0);

  // construct the transform matrices
  double cos_theta = cos(car_theta);
  double sin_theta = sin(car_theta);
  double sin2_cos2_theta = sin_theta * sin_theta + cos_theta * cos_theta;
  double sin_term = sin_theta / sin2_cos2_theta;
  double cos_term = cos_theta / sin2_cos2_theta;
  Eigen::Matrix2d transform_mat;
  transform_mat <<  cos_term, -sin_term,
                    sin_term, cos_term;
  Eigen::MatrixXd pts_mat(ptsx.size(), 2);
  pts_mat.col(0) << Eigen::ArrayXd::Map(ptsx.data(), ptsx.size()) - car_x;
  pts_mat.col(1) << Eigen::ArrayXd::Map(ptsy.data(), ptsy.size()) - car_y;

  // perform matrix multiplication
  Eigen::MatrixXd result_mat = pts_mat * transform_mat;
  Eigen::ArrayXd result_x_ar = result_mat.col(0);
  Eigen::ArrayXd result_y_ar = result_mat.col(1);

  // convert result back to vector
  vector<double> result_x(result_x_ar.size());
  Eigen::ArrayXd::Map(&result_x[0], result_x_ar.size()) = result_x_ar;
  vector<double> result_y(result_y_ar.size());
  Eigen::ArrayXd::Map(&result_y[0], result_y_ar.size()) = result_y_ar;

  // return the resulting x and y points
  vector<vector<double>> result;
  result.push_back(result_x);
  result.push_back(result_y);
  return result;
}

inline double calculate_cte(const vector<double> &ptsx, const vector<double> &ptsy) {
  double a = ptsy[0] - ptsy[1];
  double b = ptsx[1] - ptsx[0];
  double c = (ptsx[0] - ptsx[1]) * ptsy[0] + (ptsy[1] - ptsy[0]) * ptsx[0];
  
  double cte = abs(c) / sqrt(a * a + b * b);
  double y_intersect = -b * c / (a * a + b * b);
  if (y_intersect < 0) {
    cte *= -1;
  }

  return cte;
}

#endif /* HELPER_FUNCTIONS_H_ */