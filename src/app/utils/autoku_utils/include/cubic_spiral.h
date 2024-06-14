/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
#ifndef __CUBIC_SPIRAL_H__
#define __CUBIC_SPIRAL_H__
#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <array>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/LU"

// #include <glog/logging.h>
// #include <gtest/gtest.h>

using namespace std;

/**
 * @file structs.h
 **/

struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  double speed;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
};

struct SpiralConfig {
  int simpson_size = 9;
  double newton_raphson_tol = 0.01;
  int newton_raphson_max_iter = 20;
};

/**
 * @file integral.h
 * @brief Functions to compute integral.
 */

double IntegrateBySimpson(const std::vector<double>& funv_vec, const double dx,
                          const std::size_t nsteps);

double IntegrateByTrapezoidal(const std::vector<double>& funv_vec,
                              const double dx, const std::size_t nsteps);
/**
 * @brief Compute the integral of a target single-variable function
 *        from a lower bound to an upper bound, by 5-th Gauss-Legendre method
 * Given a target function and integral lower and upper bound,
 * compute the integral approximation using 5th order Gauss-Legendre
 * integration.
 * The target function must be a smooth function.
 * Example:
 * target function: auto func = [](const double& x) {return x * x;};
 *                  double integral = gauss_legendre(func, -2, 3);
 * This gives you the approximated integral of function x^2 in bound [-2, 3]
 *
 * reference: https://en.wikipedia.org/wiki/Gaussian_quadrature
 *            http://www.mymathlib.com/quadrature/gauss_legendre.html
 *
 * @param func The target single-variable function
 * @param lower_bound The lower bound of the integral
 * @param upper_bound The upper bound of the integral
 * @return The integral result
 */
double IntegrateByGaussLegendre(const std::function<double(double)>& func,
                                const double lower_bound,
                                const double upper_bound);

/*
 * @file: spiral_equations.h
 */

class SpiralEquations {
 public:
  /**
   * @brief: convert p parameters to a parameters for cubic spiral
   * @params: [in] sg - the final length of spiral path from start to end point
   *            [in] p  - vector of params: p0, p1, p2, p3
   *                      p0 = kappa at s = 0
   *                      p1 = kappa at s = sg / 3
   *                      p2 = kappa at s = 2 * sg / 3
   *                      p3 = kappa at s = sg
   * @return: [out] a - parameter vec of cubic kappa spiral
   **/
  static std::array<double, 4> p_to_k3(const double sg,
                                       const std::array<double, 4>& p);

  /**
   * @brief: convert p parameters to a parameters for quintic spiral
   * @params: [in] sg - the final length of spiral path from start to end point
   *            [in] p  - vector of params: p0, p1, p2, p3, p4, p5
   *
   *                      p0 = kappa  at s = 0
   *                      p1 = dkappa at s = 0
   *                      p2 = ddkappa at s = 0
   *                      p3 = kappa at s = sg / 3
   *                      p4 = kappa at s = 2 * sg / 3
   *
   * @return: [out] a - parameter vec of quintic kappa spiral
   **/
  static std::array<double, 6> p_to_k5(const double sg,
                                       const std::array<double, 6>& p);

  /* ------------------------  kappa, theta, dkappa functions
   * --------------------- */

  // set 1: kappa, theta, dkappa with a vector (normal vector) as parameter
  // input
  /**
   * @brief : cubic kappa function with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief : cubic theta function with regular parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief : derivative of cubic kappa function (dkappa) with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k3_a(const double s,
                                 const std::array<double, 4>& a);

  /**
   * @brief : quintic kappa function with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief : quintic theta function with regular parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief : derivative of quintic kappa function (dkappa) with regular
   *parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k5_a(const double s,
                                 const std::array<double, 6>& a);

  // set 2 - kappa, theta dkappa funcs with p parameter

  /**
   * @brief : cubic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double kappa_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief : cubic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given p
   **/
  static double theta_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief : derivative of cubic kappa function (dkappa) with p parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given p
   **/
  static double dkappa_func_k3(const double s, const double sg,
                               const std::array<double, 4>& p);

  /**
   * @brief : quintic kappa function with p parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given p
   **/
  static double kappa_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief : quintic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: theta value with respect to s, theta(s) given p
   **/
  static double theta_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief : derivative of quintic kappa function (dkappa) with regular
   *parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given p params
   **/
  static double dkappa_func_k5(const double s, const double sg,
                               const std::array<double, 6>& p);

  /*** ------------------------- Partial Derivatives
   * -------------------------------------*/
  // Partial deriavatives of theta with respect to p1, p2, sg (p3, p4 sg for
  // quintic version)

  /**
   * @brief: calculate partial derivative given sg (final curve length) at s
   *location;
   * @params: [in] s  - s location with respect to path's own SL coordinate
   *            [in] sg - final length of path
   *            [in] p  - p params
   * @return partial theta / partial p1, p2 or p3
   **/

  // ----------------------------------- Cubic Version
  // ----------------------------------

  /**
   * @brief: partial derivative theta with respect to p1
   **/
  static double partial_theta_p1_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p2
   **/
  static double partial_theta_p2_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k3(const double s, const double sg,
                                    const std::array<double, 4>& p);

  //  ---------------------------------- Quintic Version
  //  ---------------------------------

  /**
   * @brief: partial derivative theta with respect to p3
   **/
  static double partial_theta_p3_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p4
   **/
  static double partial_theta_p4_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k5(const double s, const double sg,
                                    const std::array<double, 6>& p);

 private:
  SpiralEquations() = default;
};

/**
 * @file: spiral_base.h
 * @brief: spiral path base class
 **/

class SpiralBase {
 public:
  SpiralBase(const std::uint32_t order);
  virtual ~SpiralBase() = default;

  /**
   *   @brief: set configuration if desired (default setting was defined in
   *constructor)
   **/
  void SetSpiralConfig(const SpiralConfig& spiral_config);
  /**
   *   @brief : default process of calculating path without lookup table
   *   @return: errors of final state: fitted value vs true end point
   **/
  virtual bool GenerateSpiral(const PathPoint& start, const PathPoint& end) = 0;

  /**
   *   @brief: output methods
   **/
  const std::vector<double>& p_params() const;
  const SpiralConfig& spiral_config() const;
  const PathPoint& start_point() const;
  const PathPoint& end_point() const;
  void set_start_point(const PathPoint& start);
  void set_end_point(const PathPoint& end);
  double sg() const;
  double error() const;

  /**
   *   @brief : get path vector with sampling size n
   *   @return: sequence of sampling points
   **/
  virtual bool GetSampledSpiral(const std::uint32_t n,
                                std::vector<PathPoint>* path_points) const = 0;
  virtual bool GetTimeSampledSpiral(const std::vector<double> s, const std::vector<double> v,
                                    std::vector<PathPoint>* path_points) const = 0;

 private:
  const PathPoint* start_point_;
  const PathPoint* end_point_;
  std::vector<double> p_params_;
  double sg_;
  double error_;
  SpiralConfig spiral_config_;

 protected:
  void set_sg(const double sg);
  void set_error(const double error);

  bool ResultSanityCheck() const;

  template <typename T>
  void PrependToPParams(T begin, T end) {
    std::copy(begin, end, p_params_.begin());
  }
  static constexpr double s_two_pi_ = 2 * M_PI;
};


/**
 * @file  : cubic_spiral.h
 * @brief : path class includes the basic parameters for defining a path from
 *initial
 *            point to end point
 * @model description :
 *            x_p (s) = int_0^s cos( theta_p (s)) ds
 *            y_p (s) = int_0^s sin( theta_p (s)) ds
 *            theta_p (s) = a s + b s^2 / 2 + c s^3 / 3 + d s^4 / 4
 *            kappa_p (s) = a + b s + c s^2 + d s^3
 * @solver: Solve boundary shooting problem with newton raphson method
 *            (default) initialized step for newton: 8, tol = 10^-2, max_iter =
 *10
 **/

class CubicSpiral : public SpiralBase {
 public:
  CubicSpiral();
  ~CubicSpiral() = default;
  bool GenerateSpiral(const PathPoint& start, const PathPoint& end);
  bool GetSampledSpiral(const std::uint32_t n,
                        std::vector<PathPoint>* path_points) const override;
  bool GetTimeSampledSpiral(const std::vector<double> s, const std::vector<double> v,
                            std::vector<PathPoint>* path_points) const override;
};

#endif // __CUBIC_SPIRAL_H__