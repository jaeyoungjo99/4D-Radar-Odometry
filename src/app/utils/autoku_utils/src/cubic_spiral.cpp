/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#include "cubic_spiral.h"

/**
 * @file: spiral_base.cpp
 **/

SpiralBase::SpiralBase(const std::uint32_t order)
    : p_params_(order + 1, 0.0),
      sg_(0.0),
      error_(std::numeric_limits<double>::infinity()) {}

void SpiralBase::SetSpiralConfig(const SpiralConfig& spiral_config) {
  spiral_config_ = spiral_config;
}

// set params
void SpiralBase::set_start_point(const PathPoint& start) {
  start_point_ = &start;
}
void SpiralBase::set_end_point(const PathPoint& end) { end_point_ = &end; }

// output params
const PathPoint& SpiralBase::start_point() const { return *start_point_; }

const PathPoint& SpiralBase::end_point() const { return *end_point_; }

double SpiralBase::sg() const { return sg_; }

double SpiralBase::error() const { return error_; }

const std::vector<double>& SpiralBase::p_params() const { return p_params_; }
const SpiralConfig& SpiralBase::spiral_config() const { return spiral_config_; }

void SpiralBase::set_sg(const double sg) { sg_ = sg; }

void SpiralBase::set_error(const double error) { error_ = error; }

bool SpiralBase::ResultSanityCheck() const {
  for (const auto& p : p_params_) {
    if (std::isnan(p)) {
      return false;
    }
  }
  return (sg_ > 0);
}

/**
 * @file integral.cpp
 **/

double IntegrateBySimpson(const std::vector<double>& func, const double dx,
                          const std::size_t nsteps) {
  // CHECK_EQ(1, nsteps & 1);
  double sum1 = 0.0;
  double sum2 = 0.0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    if ((i & 1) != 0) {
      sum1 += func[i];
    } else {
      sum2 += func[i];
    }
  }
  return dx / 3.0 * (4.0 * sum1 + 2.0 * sum2 + func[0] + func[nsteps - 1]);
}

double IntegrateByTrapezoidal(const std::vector<double>& func, const double dx,
                              const std::size_t nsteps) {
  double sum = 0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    sum += func[i];
  }
  return dx * sum + 0.5 * dx * (func[0] + func[nsteps - 1]);
}

double IntegrateByGaussLegendre(const std::function<double(double)>& func,
                                const double lower_bound,
                                const double upper_bound) {
  const double t = (upper_bound - lower_bound) * 0.5;
  const double m = (upper_bound + lower_bound) * 0.5;

  std::array<double, 5> w;
  w[0] = 0.5688888889;
  w[1] = 0.4786286705;
  w[2] = 0.4786286705;
  w[3] = 0.2369268851;
  w[4] = 0.2369268851;

  std::array<double, 5> x;
  x[0] = 0.0;
  x[1] = 0.5384693101;
  x[2] = -0.5384693101;
  x[3] = 0.9061798459;
  x[4] = -0.9061798459;

  double integral = 0.0;
  for (size_t i = 0; i < 5; ++i) {
    integral += w[i] * func(t * x[i] + m);
  }

  return integral * t;
}

/*
 * @file: spiral_equations.cpp
 */

// coef transformation k3 indicates cubic spiral, k5 indecate quintic spiral
std::array<double, 4> SpiralEquations::p_to_k3(const double sg,
                                               const std::array<double, 4>& p) {
  std::array<double, 4> result = {
      // p params to cubic Poly params
      p[0], -(11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3]) / (2.0 * sg),
      (18.0 * p[0] - 45.0 * p[1] + 36.0 * p[2] - 9.0 * p[3]) / (2.0 * sg * sg),
      -(9 * p[0] - 27.0 * p[1] + 27.0 * p[2] - 9.0 * p[3]) /
          (2.0 * sg * sg * sg)};
  return result;
}

std::array<double, 6> SpiralEquations::p_to_k5(const double sg,
                                               const std::array<double, 6>& p) {
  double sg2 = sg * sg;
  double sg3 = sg2 * sg;

  std::array<double, 6> result = {
      // p params to quintic params
      p[0],
      p[1],
      p[2] / 2.0,
      -(575 * p[0] - 648 * p[3] + 81 * p[4] - 8 * p[5] + 170 * p[1] * sg +
        22 * p[2] * sg2) /
          (8 * sg3),
      (333 * p[0] - 405 * p[3] + 81 * p[4] - 9 * p[5] + 90 * p[1] * sg +
       9 * p[2] * sg2) /
          (2 * sg2 * sg2),
      (-765 * p[0] + 972 * p[3] - 243 * p[4] + 36 * p[5] - 198 * p[1] * sg -
       18 * p[2] * sg2) /
          (8 * sg2 * sg3)};
  return result;
}

// kappa, theta, dkappa funcs without transformation
double SpiralEquations::kappa_func_k3_a(const double s,
                                        const std::array<double, 4>& a) {
  return ((a[3] * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralEquations::theta_func_k3_a(const double s,
                                        const std::array<double, 4>& a) {
  return (((a[3] * s / 4 + a[2] / 3) * s + a[1] / 2) * s + a[0]) * s;
}

double SpiralEquations::dkappa_func_k3_a(const double s,
                                         const std::array<double, 4>& a) {
  return (3 * a[3] * s + 2 * a[2]) * s + a[1];
}

double SpiralEquations::kappa_func_k5_a(const double s,
                                        const std::array<double, 6>& a) {
  return ((((a[5] * s + a[4]) * s + a[3]) * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralEquations::theta_func_k5_a(const double s,
                                        const std::array<double, 6>& a) {
  return (((((a[5] * s / 6 + a[4] / 5) * s + a[3] / 4) * s + a[2] / 3) * s +
           a[1] / 2) *
              s +
          a[0]) *
         s;
}

double SpiralEquations::dkappa_func_k5_a(const double s,
                                         const std::array<double, 6>& a) {
  return (((5 * a[5] * s + 4 * a[4]) * s + 3 * a[3]) * s + 2 * a[2]) * s + a[1];
}

// kappa, theta, dkappa funcs with p to a transformation
double SpiralEquations::kappa_func_k3(const double s, const double sg,
                                      const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_k3(sg, p);
  return ((a[3] * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralEquations::theta_func_k3(const double s, const double sg,
                                      const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_k3(sg, p);
  return (((a[3] * s / 4 + a[2] / 3) * s + a[1] / 2) * s + a[0]) * s;
}

double SpiralEquations::dkappa_func_k3(const double s, const double sg,
                                       const std::array<double, 4>& p) {
  std::array<double, 4> a = p_to_k3(sg, p);
  return (3 * a[3] * s + 2 * a[2]) * s + a[1];
}

double SpiralEquations::kappa_func_k5(const double s, const double sg,
                                      const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_k5(sg, p);
  return ((((a[5] * s + a[4]) * s + a[3]) * s + a[2]) * s + a[1]) * s + a[0];
}

double SpiralEquations::theta_func_k5(const double s, const double sg,
                                      const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_k5(sg, p);
  return (((((a[5] * s / 6 + a[4] / 5) * s + a[3] / 4) * s + a[2] / 3) * s +
           a[1] / 2) *
              s +
          a[0]) *
         s;
}

double SpiralEquations::dkappa_func_k5(const double s, const double sg,
                                       const std::array<double, 6>& p) {
  std::array<double, 6> a = p_to_k5(sg, p);
  return (((5 * a[5] * s + 4 * a[4]) * s + 3 * a[3]) * s + 2 * a[2]) * s + a[1];
}

double SpiralEquations::partial_theta_p1_k3(const double s, const double sg) {
  double sog = s / sg;
  return ((sog * 3.375 - 7.5) * sog + 4.5) * sog * s;
}

double SpiralEquations::partial_theta_p2_k3(const double s, const double sg) {
  double sog = s / sg;
  return ((6.0 - 3.375 * sog) * sog - 2.25) * sog * s;
}

double SpiralEquations::partial_theta_sg_k3(const double s, const double sg,
                                            const std::array<double, 4>& p) {
  double sog = s / sg;

  return ((3.375 * (p[0] - 3.0 * p[1] + 3.0 * p[2] - p[3]) * sog -
           3.0 * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3])) *
              sog +
          0.25 * (11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3])) *
         sog * sog;
}

double SpiralEquations::partial_theta_p3_k5(const double s, const double sg) {
  double sog = s / sg;
  // double ssog3 = s * sog * sog * sog;
  // double ssog4 = ssog3 * sog;
  return ((20.25 * sog - 40.5) * sog + 20.25) * sog * sog * sog * s;
  // return 20.25 * ssog3 - 40.5 * ssog4 + 20.25 * ssog4 * sog;
}

double SpiralEquations::partial_theta_p4_k5(const double s, const double sg) {
  double sog = s / sg;
  return ((-5.0625 * sog + 8.1) * sog - 2.53125) * sog * sog * sog * s;
}

double SpiralEquations::partial_theta_sg_k5(const double s, const double sg,
                                            const std::array<double, 6>& p) {
  double s2 = s * s;
  double sog = s / sg;
  double sog2 = sog * sog;
  double sog3 = sog2 * sog;
  double sog4 = sog2 * sog2;
  double sog5 = sog4 * sog;
  return (53.90625 * p[0] - 60.75 * p[3] + 7.59375 * p[4] - 0.75 * p[5]) *
             sog4 +
         10.625 * p[1] * s * sog3 + 0.6875 * p[2] * s2 * sog2 +
         (-133.2 * p[0] + 162 * p[3] - 32.4 * p[4] + 3.6 * p[5]) * sog5 +
         (-27) * p[1] * s * sog4 - 1.8 * p[2] * s2 * sog3 +
         (79.6875 * p[0] - 101.25 * p[3] + 25.3125 * p[4] - 3.75 * p[5]) *
             sog5 * sog +
         16.5 * p[1] * s * sog5 + 1.125 * p[2] * s2 * sog4;
}

/**
 * @file  : cubic_spiral.cpp
 **/

CubicSpiral::CubicSpiral() : SpiralBase(3) {
  // generate an order 3 cubic spiral path with four parameters
}

bool CubicSpiral::GenerateSpiral(const PathPoint& start, const PathPoint& end) {
  set_start_point(start);
  set_end_point(end);

  // starting p[oint
  double x_s = start_point().x;
  double y_s = start_point().y;
  double theta_s = std::fmod(start_point().theta, s_two_pi_);

  if (theta_s < 0) {
    theta_s += s_two_pi_;
  }

  // end point
  double x_t = end_point().x - x_s;
  double y_t = end_point().y - y_s;

  // with position and rotation transformation
  double x_g = std::cos(theta_s) * x_t + std::sin(theta_s) * y_t;
  double y_g = -std::sin(theta_s) * x_t + std::cos(theta_s) * y_t;
  double theta_g = std::fmod(end_point().theta, s_two_pi_);
  theta_g -= theta_s;

  while (theta_g < -M_PI) {
    theta_g += s_two_pi_;
  }

  while (theta_g > +M_PI) {
    theta_g -= s_two_pi_;
  }
  std::array<double, 4> p_shoot;
  double sg =
      (theta_g * theta_g / 5.0 + 1.0) * std::sqrt(x_g * x_g + y_g * y_g);
  p_shoot[0] = start_point().kappa;
  p_shoot[1] = 0.0;
  p_shoot[2] = 0.0;
  p_shoot[3] = end_point().kappa;

  // intermediate params
  Eigen::Matrix<double, 3, 1> q_g;
  q_g << x_g, y_g, theta_g;            // goal, x(p, sg), y(p, sg), theta(p, sg)
  Eigen::Matrix<double, 3, 3> jacobi;  // Jacobian matrix for newton method

  // simpson integrations func values in Jacobian
  // integration point initialization:
  double ds =
      sg / (spiral_config().simpson_size - 1);  // bandwith for integration
  // basic theta value vectors:
  std::vector<double> theta(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_theta(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_theta(spiral_config().simpson_size, 0.0);
  // partial derivatives vectors for Jacobian
  std::vector<double> ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> ptp_sg(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_sg(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_sg(spiral_config().simpson_size, 0.0);

  // newton iteration difference (col) vectors
  Eigen::Matrix<double, 3, 1> delta_q;  // goal difference
  Eigen::Matrix<double, 3, 1> delta_p;  // parameter difference
  Eigen::Matrix<double, 3, 1>
      q_guess;        // q with current paramter, delta_q = q_g - q_guess
  double diff = 0.0;  // absolute error for q iteration stop

  for (int32_t nt = 0; nt < spiral_config().newton_raphson_max_iter; ++nt) {
    // calculate parameters for simpson integration
    double s = 0.0;

    for (int32_t i = 0; i < spiral_config().simpson_size; ++i) {
      theta[i] = SpiralEquations::theta_func_k3(s, sg, p_shoot);

      cos_theta[i] = std::cos(theta[i]);
      sin_theta[i] = std::sin(theta[i]);

      ptp_p1[i] = SpiralEquations::partial_theta_p1_k3(s, sg);
      ptp_p2[i] = SpiralEquations::partial_theta_p2_k3(s, sg);
      ptp_sg[i] = SpiralEquations::partial_theta_sg_k3(s, sg, p_shoot);

      sin_ptp_p1[i] = sin_theta[i] * ptp_p1[i];
      sin_ptp_p2[i] = sin_theta[i] * ptp_p2[i];
      sin_ptp_sg[i] = sin_theta[i] * ptp_sg[i];

      cos_ptp_p1[i] = cos_theta[i] * ptp_p1[i];
      cos_ptp_p2[i] = cos_theta[i] * ptp_p2[i];
      cos_ptp_sg[i] = cos_theta[i] * ptp_sg[i];
      s += ds;
    }

    // update Jacobian and delta q
    jacobi(0, 0) =
        -IntegrateBySimpson(sin_ptp_p1, ds, spiral_config().simpson_size);
    jacobi(0, 1) =
        -IntegrateBySimpson(sin_ptp_p2, ds, spiral_config().simpson_size);
    jacobi(0, 2) =
        cos_theta[spiral_config().simpson_size - 1] -
        IntegrateBySimpson(sin_ptp_sg, ds, spiral_config().simpson_size);

    jacobi(1, 0) =
        IntegrateBySimpson(cos_ptp_p1, ds, spiral_config().simpson_size);
    jacobi(1, 1) =
        IntegrateBySimpson(cos_ptp_p2, ds, spiral_config().simpson_size);
    jacobi(1, 2) =
        sin_theta[spiral_config().simpson_size - 1] +
        IntegrateBySimpson(cos_ptp_sg, ds, spiral_config().simpson_size);

    jacobi(2, 0) = ptp_p1[spiral_config().simpson_size - 1];
    jacobi(2, 1) = ptp_p2[spiral_config().simpson_size - 1];
    jacobi(2, 2) = ptp_sg[spiral_config().simpson_size - 1];

    q_guess(0) =
        IntegrateBySimpson(cos_theta, ds, spiral_config().simpson_size);
    q_guess(1) =
        IntegrateBySimpson(sin_theta, ds, spiral_config().simpson_size);
    q_guess(2) = theta[spiral_config().simpson_size - 1];

    delta_q = q_g - q_guess;

    diff =
        std::fabs(delta_q(0)) + std::fabs(delta_q(1)) + std::fabs(delta_q(2));

    if (diff < spiral_config().newton_raphson_tol) {
      break;
    }

    // solve by lu decomposition
    delta_p = jacobi.lu().solve(delta_q);
    // update p, sg, ds
    p_shoot[1] += delta_p(0);
    p_shoot[2] += delta_p(1);
    sg += delta_p(2);
    ds = sg / (spiral_config().simpson_size - 1);
  }

  PrependToPParams(p_shoot.begin(), p_shoot.end());
  set_sg(sg);
  set_error(diff);

  return diff < spiral_config().newton_raphson_tol && ResultSanityCheck();
}

bool CubicSpiral::GetSampledSpiral(const std::uint32_t n,
                                   std::vector<PathPoint>* path_points) const {
  // CHECK_NOTNULL(path_points);

  // initialization
  if (n < 2 || error() > spiral_config().newton_raphson_tol) {
    return false;
  }

  path_points->resize(n);

  std::vector<PathPoint>& result = *path_points;
  const double ds = sg() / (n - 1);

  std::array<double, 4> p_value;
  std::copy_n(p_params().begin(), 4, p_value.begin());

  result[0].x = start_point().x;
  result[0].y = start_point().y;
  result[0].theta = start_point().theta;
  result[0].kappa = start_point().kappa;
  result[0].dkappa = SpiralEquations::dkappa_func_k3(0, sg(), p_value);

  // calculate path x, y using iterative trapezoidal method
  // initialization
  double s = ds;
  // calculate heading kappa along the path
  std::array<double, 4> a_params = SpiralEquations::p_to_k3(sg(), p_value);

  for (std::uint32_t i = 1; i < n; ++i) {
    result[i].s = s;
    result[i].theta =
        SpiralEquations::theta_func_k3_a(s, a_params) + result[0].theta;
    result[i].kappa = SpiralEquations::kappa_func_k3_a(s, a_params);
    result[i].dkappa = SpiralEquations::dkappa_func_k3_a(s, a_params);
    s += ds;
  }

  // integration x, y along the path
  double dx = 0;
  double dy = 0;

  for (std::uint32_t k = 1; k < n; ++k) {
    dx = (dx / k) * (k - 1) +
         (std::cos(std::fmod(result[k].theta, s_two_pi_)) +
          std::cos(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    dy = (dy / k) * (k - 1) +
         (std::sin(std::fmod(result[k].theta, s_two_pi_)) +
          std::sin(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    result[k].x = result[k].s * dx + result[0].x;
    result[k].y = result[k].s * dy + result[0].y;
  }

  return true;
}

bool CubicSpiral::GetTimeSampledSpiral(const std::vector<double> s,
                                       const std::vector<double> v,
                                       std::vector<PathPoint>* path_points) const {
  // CHECK_NOTNULL(path_points);

  std::vector<double> new_s = s;
  double s_start = new_s.front();
  double s_end = new_s.back(); 

  for (double& ss : new_s) {
    ss = ((ss - s_start) / (s_end - s_start)) * sg();
  }

  // initialization
  if (new_s.size() < 2 || error() > spiral_config().newton_raphson_tol) {
    return false;
  }

  path_points->resize(new_s.size());

  std::vector<PathPoint>& result = *path_points;

  std::array<double, 4> p_value;
  std::copy_n(p_params().begin(), 4, p_value.begin());

  result[0].x = start_point().x;
  result[0].y = start_point().y;
  result[0].s = new_s.at(0);
  result[0].speed = v.at(0);
  result[0].theta = start_point().theta;
  result[0].kappa = start_point().kappa;
  result[0].dkappa = SpiralEquations::dkappa_func_k3(0, sg(), p_value);

  // calculate path x, y using iterative trapezoidal method

  // calculate heading kappa along the path
  std::array<double, 4> a_params = SpiralEquations::p_to_k3(sg(), p_value);
  
  for (std::uint32_t i = 1; i < s.size(); ++i) {
    double current_s = new_s.at(i);
    result[i].s = current_s;
    result[i].theta =
        SpiralEquations::theta_func_k3_a(current_s, a_params) + result[0].theta;
    result[i].kappa = SpiralEquations::kappa_func_k3_a(current_s, a_params);
    result[i].dkappa = SpiralEquations::dkappa_func_k3_a(current_s, a_params);
    result[i].speed = v.at(i);
  }

  // integration x, y along the path
  double dx = 0;
  double dy = 0;

  for (std::uint32_t k = 1; k < s.size(); ++k) {
    dx = (dx / k) * (k - 1) +
         (std::cos(std::fmod(result[k].theta, s_two_pi_)) +
          std::cos(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    dy = (dy / k) * (k - 1) +
         (std::sin(std::fmod(result[k].theta, s_two_pi_)) +
          std::sin(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    result[k].x = result[k].s * dx + result[0].x;
    result[k].y = result[k].s * dy + result[0].y;
  }

  return true;
}

