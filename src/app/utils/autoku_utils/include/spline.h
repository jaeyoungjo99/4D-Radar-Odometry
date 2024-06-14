/*
 * spline.h
 *
 * simple cubic spline interpolation library without external
 * dependencies
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2011, 2014 Tino Kluge (ttk448 at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 */

#ifndef SPLINE_H
#define SPLINE_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>
#include <math.h>

namespace tk{
// band matrix solver
class band_matrix
{
private:
    std::vector< std::vector<double> > m_upper;  // upper band
    std::vector< std::vector<double> > m_lower;  // lower band
public:
    band_matrix() {};                             // constructor
    band_matrix(int dim, int n_u, int n_l);       // constructor
    ~band_matrix() {};                            // destructor
    void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
    int dim() const;                             // matrix dimension
    int num_upper() const
    {
        return m_upper.size()-1;
    }
    int num_lower() const
    {
        return m_lower.size()-1;
    }
    // access operator
    double & operator () (int i, int j);            // write
    double   operator () (int i, int j) const;      // read
    // we can store an additional diogonal (in m_lower)
    double& saved_diag(int i);
    double  saved_diag(int i) const;
    void lu_decompose();
    std::vector<double> r_solve(const std::vector<double>& b) const;
    std::vector<double> l_solve(const std::vector<double>& b) const;
    std::vector<double> lu_solve(const std::vector<double>& b,
                                 bool is_lu_decomposed=false);

};

// spline interpolation
class spline
{
public:
    enum bd_type {
        first_deriv = 1,
        second_deriv = 2
    };

private:
    std::vector<double> m_x,m_y;            // x,y coordinates of points
    // interpolation parameters
    // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
    std::vector<double> m_a,m_b,m_c;        // spline coefficients
    double  m_a0, m_b0, m_c0;                     // for left extrapol
    bd_type m_left, m_right;
    double  m_left_value, m_right_value;
    bool    m_force_linear_extrapolation;

public:
    // set default boundary condition to be zero curvature at both ends
    spline(): m_left(second_deriv), m_right(second_deriv),
        m_left_value(0.0), m_right_value(0.0),
        m_force_linear_extrapolation(false)
    {
        ;
    }

    // optional, but if called it has to come be before set_points()
    void set_boundary(bd_type left, double left_value,
                      bd_type right, double right_value,
                      bool force_linear_extrapolation=false);
    void set_points(const std::vector<double>& x,
                    const std::vector<double>& y, bool cubic_spline=true);
    double operator() (double x) const;
    std::vector<double> derivative (double x, int up_to = 1) const;
};

struct Spline
{
	public:
		double	max_s_;
  		double	lane_width_;
  		int		lanes_;
	private:
  		spline x_s_;
  		spline y_s_;
		double total_length_;

  		std::vector<double> maps_x_;
  		std::vector<double> maps_y_;
  		std::vector<double> maps_s_;
  		std::vector<double> maps_d_x_;
		std::vector<double> maps_d_y_;

	public:
  		Spline() {  }
  		Spline(Spline& map);
  		Spline(std::vector<double> sx,
	  		std::vector<double> sy,
	  		std::vector<double> ss,
	  		std::vector<double> d_sx,
	  		std::vector<double> d_sy  );
  		Spline& operator=(const Spline& sp);
  
  		double distance(double x1, double y1, double x2, double y2);
  		std::vector<double> ToFrenetVelocity(double vx, double vy, double frenet_s);
  		double FrenetSCycle(double s);

  		// Find the closet point on the map
  		int ClosestWaypoint(double x, double y);    
  
  		// Coordinate conversion
  		std::vector<double> ToFrenet(double x, double y);
  		std::vector<double> ToCartesian(double s, double d);

  		double SubstractS(double minuend, double subtrahend);
  		double GetSlope(double s);
  		double GetCurvature(double s);
  		inline double GetTotalLength() {return total_length_;};
};

struct Map
{
	public:
		double	max_s_;
  		double	lane_width_=3.0;
  		int	lanes_ = 3;
	private:
  		spline x_s_;
  		spline y_s_;
		double total_length_;

  		std::vector<double> maps_x_;
  		std::vector<double> maps_y_;
  		std::vector<double> maps_s_;
  		std::vector<double> maps_d_x_;
		std::vector<double> maps_d_y_;

		std::vector<int> lane_indices_;

	public:
  		Map() {  }
  		Map(const Map & map);
		Map(std::vector<double> x, std::vector<double> y);
  		Map(std::vector<double> sx,
	  		std::vector<double> sy,
	  		std::vector<double> ss,
	  		std::vector<double> d_sx,
	  		std::vector<double> d_sy  );
  
  		double distance(double x1, double y1, double x2, double y2);
  		std::vector<double> ToFrenetVelocity(double vx, double vy, double frenet_s);
  		double FrenetSCycle(double s);

  		// Find the closet point on the map
  		int ClosestWaypoint(double x, double y);    
        double ClosestDistance(double x, double y);
  
  		// Coordinate conversion
  		std::vector<double> ToFrenet(double x, double y);
  		std::vector<double> ToCartesian(double s, double d);

		std::vector<double> ToFrenetAllS( std::vector<double> state_cartesian );
		std::vector<double> ToCartesianAllS( std::vector<double> state_frenet );
		std::vector<double> ToFrenetAllT( std::vector<double> state_cartesian );
		std::vector<double> ToCartesianAllT( std::vector<double> state_frenet );

  		double SubstractS(double minuend, double subtrahend);
  		double GetSlope(double s);
  		double GetCurvature(double s);
		inline double GetMaxS() { return max_s_; }
  		inline double GetTotalLength() {return total_length_;};

	public:
		inline void SetLaneOffset(double offset) { lane_width_ = offset; }
		inline void SetNumLanes(int num) { lanes_ = num; }
		inline void SetLanes( std::vector<int> lanes ) { lane_indices_ = lanes; }
		inline int GetLaneId( int i ) { return lane_indices_[i]; }

		int GetLaneIndexXY( double x, double y );
		int GetLaneIndexN( double n );
		double GetOffset( int lane_idx );
		int GetNumLane() { return lane_indices_.size(); }
		double GetLaneWidth() { return lane_width_; }
		int GetMaxLaneID();
		int GetMinLaneID();
};
}
#endif /* SPLINE_H */
