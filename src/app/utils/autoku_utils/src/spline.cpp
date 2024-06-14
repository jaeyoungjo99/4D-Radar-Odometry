#include "spline.h"

using namespace tk;
band_matrix::band_matrix(int dim, int n_u, int n_l)
{
    resize(dim, n_u, n_l);
}
void band_matrix::resize(int dim, int n_u, int n_l)
{
    assert(dim>0);
    assert(n_u>=0);
    assert(n_l>=0);
    m_upper.resize(n_u+1);
    m_lower.resize(n_l+1);
    for(size_t i=0; i<m_upper.size(); i++) {
        m_upper[i].resize(dim);
    }
    for(size_t i=0; i<m_lower.size(); i++) {
        m_lower[i].resize(dim);
    }
}
int band_matrix::dim() const
{
    if(m_upper.size()>0) {
        return m_upper[0].size();
    } else {
        return 0;
    }
}


// defines the new operator (), so that we can access the elements
// by A(i,j), index going from i=0,...,dim()-1
double & band_matrix::operator () (int i, int j)
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diogonal, k<0 lower left part, k>0 upper right part
    if(k>=0)   return m_upper[k][i];
    else	    return m_lower[-k][i];
}
double band_matrix::operator () (int i, int j) const
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diogonal, k<0 lower left part, k>0 upper right part
    if(k>=0)   return m_upper[k][i];
    else	    return m_lower[-k][i];
}
// second diag (used in LU decomposition), saved in m_lower
double band_matrix::saved_diag(int i) const
{
    assert( (i>=0) && (i<dim()) );
    return m_lower[0][i];
}
double & band_matrix::saved_diag(int i)
{
    assert( (i>=0) && (i<dim()) );
    return m_lower[0][i];
}

// LR-Decomposition of a band matrix
void band_matrix::lu_decompose()
{
    int  i_max,j_max;
    int  j_min;
    double x;
	
    // preconditioning
    // normalize column i so that a_ii=1
    for(int i=0; i<this->dim(); i++) {
        assert(this->operator()(i,i)!=0.0);
        this->saved_diag(i)=1.0/this->operator()(i,i);
        j_min=std::max(0,i-this->num_lower());
        j_max=std::min(this->dim()-1,i+this->num_upper());
        for(int j=j_min; j<=j_max; j++) {
            this->operator()(i,j) *= this->saved_diag(i);
        }
        this->operator()(i,i)=1.0;          // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for(int k=0; k<this->dim(); k++) {
        i_max=std::min(this->dim()-1,k+this->num_lower());  // num_lower not a mistake!
        for(int i=k+1; i<=i_max; i++) {
            assert(this->operator()(k,k)!=0.0);
            x=-this->operator()(i,k)/this->operator()(k,k);
            this->operator()(i,k)=-x;                         // assembly part of L
            j_max=std::min(this->dim()-1,k+this->num_upper());
            for(int j=k+1; j<=j_max; j++) {
                // assembly part of R
                this->operator()(i,j)=this->operator()(i,j)+x*this->operator()(k,j);
            }
        }
    }
}
// solves Ly=b
std::vector<double> band_matrix::l_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_start;
    double sum;
    for(int i=0; i<this->dim(); i++) {
        sum=0;
        j_start=std::max(0,i-this->num_lower());
        for(int j=j_start; j<i; j++) sum += this->operator()(i,j)*x[j];
        x[i]=(b[i]*this->saved_diag(i)) - sum;
    }
    return x;
}
// solves Rx=y
std::vector<double> band_matrix::r_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_stop;
    double sum;
    for(int i=this->dim()-1; i>=0; i--) {
        sum=0;
        j_stop=std::min(this->dim()-1,i+this->num_upper());
        for(int j=i+1; j<=j_stop; j++) sum += this->operator()(i,j)*x[j];
        x[i]=( b[i] - sum ) / this->operator()(i,i);
    }
    return x;
}

std::vector<double> band_matrix::lu_solve(const std::vector<double>& b,
        bool is_lu_decomposed)
{
    assert( this->dim()==(int)b.size() );
    std::vector<double>  x,y;
    if(is_lu_decomposed==false) {
        this->lu_decompose();
    }
    y=this->l_solve(b);
    x=this->r_solve(y);
    return x;
}




// spline implementation
// -----------------------

void spline::set_boundary(spline::bd_type left, double left_value,
                          spline::bd_type right, double right_value,
                          bool force_linear_extrapolation)
{
    assert(m_x.size()==0);          // set_points() must not have happened yet
    m_left=left;
    m_right=right;
    m_left_value=left_value;
    m_right_value=right_value;
    m_force_linear_extrapolation=force_linear_extrapolation;
}


void spline::set_points(const std::vector<double>& x,
                        const std::vector<double>& y, bool cubic_spline)
{
    assert(x.size()==y.size());
    assert(x.size()>2);
    m_x=x;
    m_y=y;
    int   n=x.size();
    // TODO: maybe sort x and y, rather than returning an error
    //for(int i=0; i<n-1; i++) {
    //    assert(m_x[i]<m_x[i+1]);
    //}
    if(cubic_spline==true) { // cubic spline interpolation
        // setting up the matrix and right hand side of the equation system
        // for the parameters b[]
        band_matrix A(n,1,1);
        std::vector<double>  rhs(n);
        for(int i=1; i<n-1; i++) {
            A(i,i-1)=1.0/3.0*(x[i]-x[i-1]);
            A(i,i)=2.0/3.0*(x[i+1]-x[i-1]);
            A(i,i+1)=1.0/3.0*(x[i+1]-x[i]);
            rhs[i]=(y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
        }
        // boundary conditions
        if(m_left == spline::second_deriv) {
            // 2*b[0] = f''
            A(0,0)=2.0;
            A(0,1)=0.0;
            rhs[0]=m_left_value;
        } else if(m_left == spline::first_deriv) {
            // c[0] = f', needs to be re-expressed in terms of b:
            // (2b[0]+b[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
            A(0,0)=2.0*(x[1]-x[0]);
            A(0,1)=1.0*(x[1]-x[0]);
            rhs[0]=3.0*((y[1]-y[0])/(x[1]-x[0])-m_left_value);
        } else {
            assert(false);
        }
        if(m_right == spline::second_deriv) {
            // 2*b[n-1] = f''
            A(n-1,n-1)=2.0;
            A(n-1,n-2)=0.0;
            rhs[n-1]=m_right_value;
        } else if(m_right == spline::first_deriv) {
            // c[n-1] = f', needs to be re-expressed in terms of b:
            // (b[n-2]+2b[n-1])(x[n-1]-x[n-2])
            // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
            A(n-1,n-1)=2.0*(x[n-1]-x[n-2]);
            A(n-1,n-2)=1.0*(x[n-1]-x[n-2]);
            rhs[n-1]=3.0*(m_right_value-(y[n-1]-y[n-2])/(x[n-1]-x[n-2]));
        } else {
            assert(false);
        }

        // solve the equation system to obtain the parameters b[]
        m_b=A.lu_solve(rhs);

        // calculate parameters a[] and c[] based on b[]
        m_a.resize(n);
        m_c.resize(n);
        for(int i=0; i<n-1; i++) {
            m_a[i]=1.0/3.0*(m_b[i+1]-m_b[i])/(x[i+1]-x[i]);
            m_c[i]=(y[i+1]-y[i])/(x[i+1]-x[i])
                   - 1.0/3.0*(2.0*m_b[i]+m_b[i+1])*(x[i+1]-x[i]);
        }
    } else { // linear interpolation
        m_a.resize(n);
        m_b.resize(n);
        m_c.resize(n);
        for(int i=0; i<n-1; i++) {
            m_a[i]=0.0;
            m_b[i]=0.0;
            m_c[i]=(m_y[i+1]-m_y[i])/(m_x[i+1]-m_x[i]);
        }
    }
    // for left extrapolation coefficients
    m_a0 = (m_force_linear_extrapolation==false) ? m_a[0] : 0.0;
    m_b0 = (m_force_linear_extrapolation==false) ? m_b[0] : 0.0;
    m_c0 = m_c[0];

    // for the right extrapolation coefficients
    // f_{n-1}(x) = b*(x-x_{n-1})^2 + c*(x-x_{n-1}) + y_{n-1}
    double h=x[n-1]-x[n-2];
    // m_b[n-1] is determined by the boundary condition
    m_a[n-1]=0.0;
    m_c[n-1]=3.0*m_a[n-2]*h*h+2.0*m_b[n-2]*h+m_c[n-2];   // = f'_{n-2}(x_{n-1})
    if(m_force_linear_extrapolation==true)
        m_b[n-1]=0.0;
}

double spline::operator() (double x) const
{
    size_t n=m_x.size();
    // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
    std::vector<double>::const_iterator it;
    it=std::lower_bound(m_x.begin(),m_x.end(),x);
    int idx=std::max( int(it-m_x.begin())-1, 0);

    double h=x-m_x[idx];
    double interpol;
    if(x<m_x[0]) {
        // extrapolation to the left
        interpol=(m_b0*h + m_c0)*h + m_y[0];
    } else if(x>m_x[n-1]) {
        // extrapolation to the right
        interpol=(m_b[n-1]*h + m_c[n-1])*h + m_y[n-1];
    } else {
        // interpolation
        interpol=((m_a[idx]*h + m_b[idx])*h + m_c[idx])*h + m_y[idx];
    }
    return interpol;
}

std::vector<double> spline::derivative (double x, int up_to) const
{
    assert(up_to==1 ||up_to==2 || up_to==3);
    size_t n=m_x.size();
    // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
    std::vector<double>::const_iterator it;
    it=std::lower_bound(m_x.begin(),m_x.end(),x);
    int idx=std::max( int(it-m_x.begin())-1, 0);

    std::vector<double> result;
    double h=x-m_x[idx];
    double interpol;
    if(x<m_x[0]) {
        // extrapolation to the left
        interpol=2*m_b0*h + m_c0;
    } else if(x>m_x[n-1]) {
        // extrapolation to the right
        interpol=2*m_b[n-1]*h + m_c[n-1];
    } else {
        // interpolation
        interpol=(3*m_a[idx]*h + 2*m_b[idx])*h + m_c[idx];
    }
    result.push_back(interpol);
    if (up_to > 1){
        if(x < m_x[0]) {
            // extrapolation to the left
            interpol = 2*m_b0;
        } else if(x > m_x[n-1]) {
            // extrapolation to the right
            interpol = 2*m_b[n-1];
        } else {
            // interpolation
            interpol = 6*m_a[idx]*h + 2*m_b[idx];
        }
        result.push_back(interpol);
    }
    if( up_to > 2){
	if(x < m_x[0]) {
            // extrapolation to the left
            interpol = 6*m_a0;
        } else if(x > m_x[n-1]) {
            // extrapolation to the right
            interpol = 6*m_a[n-1];
        } else {
            // interpolation
            interpol = 6*m_a[idx];
        }
        result.push_back(interpol);

    }
    return result;
}

Spline::Spline(std::vector<double> sx,
	std::vector<double> sy,
	std::vector<double> ss,
	std::vector<double> d_sx,
	std::vector<double> d_sy
)
:max_s_(300.0),
lane_width_(3.0),
lanes_(2)
{
	for (int i = 0; i < sx.size(); i++)
	{
		double x = sx[i];
		double y = sy[i];
		double s = ss[i];
		double d_x = d_sx[i];
		double d_y = d_sy[i];
		maps_x_.push_back(x);
		maps_y_.push_back(y);
		maps_s_.push_back(s);
		maps_d_x_.push_back(d_x);
		maps_d_y_.push_back(d_y);
	}
	int map_size = maps_s_.size();
	if (ss.size() != 0)
		total_length_ = ss.back();
	else
		total_length_ = 0.0;

	std::vector<double> ss_vals(maps_s_), xs_vals(maps_x_), ys_vals(maps_y_);
	ss_vals.push_back(max_s_);
	xs_vals.push_back(maps_x_[0]);
	ys_vals.push_back(maps_y_[0]);
	x_s_.set_points(ss_vals, xs_vals);
	y_s_.set_points(ss_vals, ys_vals);
}

Spline::Spline(Spline & map) {
	max_s_ = map.max_s_;
	lane_width_ = map.lane_width_;
	lanes_ = map.lanes_;
	total_length_ = map.total_length_;

	x_s_ = map.x_s_;
	y_s_ = map.y_s_;

	maps_x_ = map.maps_x_;
	maps_y_ = map.maps_y_;
	maps_s_ = map.maps_s_;
	maps_d_x_ = map.maps_d_x_;
	maps_d_y_ = map.maps_d_y_;
}

Spline& Spline::operator=(const Spline& sp){
    max_s_ = sp.max_s_;
	lane_width_ = sp.lane_width_;
	lanes_ = sp.lanes_;
	total_length_ = sp.total_length_;

	x_s_ = sp.x_s_;
	y_s_ = sp.y_s_;

	maps_x_ = sp.maps_x_;
	maps_y_ = sp.maps_y_;
	maps_s_ = sp.maps_s_;
	maps_d_x_ = sp.maps_d_x_;
	maps_d_y_ = sp.maps_d_y_;
}

double Spline::distance(double x1, double y1, double x2, double y2){
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));	
}

int Spline::ClosestWaypoint(double x, double y){
	double closest_len = 100000; //large number
	int closest_waypoint = 0;

	for (int i = 0; i < maps_x_.size(); i++)
	{
		double map_x = maps_x_[i];
		double map_y = maps_y_[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closest_len)
		{
			closest_len = dist;
			closest_waypoint = i;
		}
	}
	return closest_waypoint;
}

double Spline::FrenetSCycle(double s) {
	return s - int(s / max_s_) * max_s_;
}

std::vector<double> Spline::ToFrenet(double x, double y) {
	int wp1 = ClosestWaypoint(x, y);

	double frenet_s = maps_s_[wp1];

	double ds = 0.001;

	double norm_distance = 1000;

	int i = 0;
	while (fabs(norm_distance) > 0.01) {
		double x0 = x_s_(frenet_s);
		double y0 = y_s_(frenet_s);
		double s1 = FrenetSCycle(frenet_s + ds);
		double dx = x_s_(s1) - x0;
		double dy = y_s_(s1) - y0;

		double slope = atan2(dy, dx);

		norm_distance = (y - y0) * sin(slope) + (x - x0) * cos(slope);

		frenet_s = FrenetSCycle(frenet_s + norm_distance);

		i++;
		if (i > 1000) return{ 0.0, 0.0 };
	}

	double x0 = x_s_(frenet_s);
	double y0 = y_s_(frenet_s);

	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x0;
	double dy = y_s_(s1) - y0;

	double slope = atan2(dy, dx);

	double frenet_d = (y - y0) * cos(slope) - (x - x0) * sin(slope);

	return { frenet_s, -frenet_d };
}

std::vector<double> Spline::ToCartesian(double s, double d) {
	double frenet_s = FrenetSCycle(s);
	double x = x_s_(frenet_s);
	double y = y_s_(frenet_s);

	double ds = 0.01;
	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x;
	double dy = y_s_(s1) - y;

	double norm = atan2(dx, dy);

	x += d*cos(norm);
	y -= d*sin(norm);

	return { x, y };
}

double Spline::GetSlope(double s) {
	double frenet_s = FrenetSCycle(s);
	std::vector<double> dx = x_s_.derivative(frenet_s, 2);
	std::vector<double> dy = y_s_.derivative(frenet_s, 2);

	assert(dx[0] != 0.0);

	double slop = atan2(dy[0], dx[0]);

	return slop;
}

double Spline::GetCurvature(double s) {
	double frenet_s = FrenetSCycle(s);
	std::vector<double> dx = x_s_.derivative(frenet_s, 2);
	std::vector<double> dy = y_s_.derivative(frenet_s, 2);

	double square_sum = dx[0] * dx[0] + dy[0] * dy[0];
	double zero_thre = 1e-5;
	if (square_sum < zero_thre) square_sum = zero_thre;

	double curvature = fabs(dx[0] * dy[1] - dy[0] * dx[1]) / sqrt(square_sum*square_sum*square_sum);

	return curvature;
}

std::vector<double> Spline::ToFrenetVelocity(double vx, double vy, double frenet_s) {

	double x0 = x_s_(frenet_s);
	double y0 = y_s_(frenet_s);
	double ds = 0.001;
	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x0;
	double dy = y_s_(s1) - y0;

	double slope = atan2(dy, dx);

	double cos_slope = cos(slope);
	double sin_slope = sin(slope);

	double frenet_vs = vy * sin_slope + vx * cos_slope;
	double frenet_vd = vy * cos_slope - vx * sin_slope;
	return { frenet_vs, frenet_vd };
}

double Spline::SubstractS(double minuend, double subtrahend) {
	double s_1 = FrenetSCycle(minuend);
	double s_2 = FrenetSCycle(subtrahend);
	double dist = s_1 - s_2;
	double abs_dist = fabs(dist);
	if (abs_dist > max_s_ * 0.5) {
		double sign = 1;
		if (dist > 0) sign = -1;
		dist = sign * (max_s_ - abs_dist);
	}
	return dist;
}

Map::Map(std::vector<double> x,
	std::vector<double> y )
:max_s_(300.0){


    std::vector<double> sx, sy, ss, dsx, dsy;
    double s = 0.0;
    for( int i=0; i<x.size(); i++){
	double dx = 0.0;
	double dy = 0.0;
	if( i > 0 ){
	    dx = x[i] - x[i-1];
	    dy = y[i] - y[i-1];
	}
	s += sqrt(dx*dx + dy*dy );
	sx.push_back( x[i] );
	sy.push_back( y[i] );
	ss.push_back( s );
	dsx.push_back( dx );
	dsy.push_back( dy );
    }
    for (int i = 0; i < sx.size(); i++)
    {
    	double x = sx[i];
	double y = sy[i];
	double s = ss[i];
	double d_x = dsx[i];
	double d_y = dsy[i];
	maps_x_.push_back(x);
	maps_y_.push_back(y);
	maps_s_.push_back(s);
	maps_d_x_.push_back(d_x);
	maps_d_y_.push_back(d_y);
    }
    int map_size = maps_s_.size();
    if (ss.size() != 0)
    	total_length_ = ss.back();
    else
    	total_length_ = 0.0;

    max_s_ = s + 200.0;

    std::vector<double> ss_vals(maps_s_), xs_vals(maps_x_), ys_vals(maps_y_);
    //ss_vals.push_back(max_s_);
    //xs_vals.push_back(maps_x_[0]);
    //ys_vals.push_back(maps_y_[0]);
    x_s_.set_points(ss_vals, xs_vals);
    y_s_.set_points(ss_vals, ys_vals);
}

Map::Map(std::vector<double> sx,
	std::vector<double> sy,
	std::vector<double> ss,
	std::vector<double> d_sx,
	std::vector<double> d_sy
)
:max_s_(300.0)
{
	for (int i = 0; i < sx.size(); i++)
	{
		double x = sx[i];
		double y = sy[i];
		double s = ss[i];
		double d_x = d_sx[i];
		double d_y = d_sy[i];
		maps_x_.push_back(x);
		maps_y_.push_back(y);
		maps_s_.push_back(s);
		maps_d_x_.push_back(d_x);
		maps_d_y_.push_back(d_y);
	}
	int map_size = maps_s_.size();
	if (ss.size() != 0){
		total_length_ = ss.back();
		max_s_ = ss.back() - ss.front() + 200;
	}
	else{
		total_length_ = 0.0;
		max_s_ = 0.0;
	}

	std::vector<double> ss_vals(maps_s_), xs_vals(maps_x_), ys_vals(maps_y_);
	//ss_vals.push_back(max_s_);
	//xs_vals.push_back(maps_x_[0]);
	//ys_vals.push_back(maps_y_[0]);
	x_s_.set_points(ss_vals, xs_vals);
	y_s_.set_points(ss_vals, ys_vals);
}

Map::Map(const Map & map) {
	max_s_ = map.max_s_;
	lane_width_ = map.lane_width_;
	lanes_ = map.lanes_;
	total_length_ = map.total_length_;
	
	lane_indices_ = map.lane_indices_;

	x_s_ = map.x_s_;
	y_s_ = map.y_s_;

	maps_x_ = map.maps_x_;
	maps_y_ = map.maps_y_;
	maps_s_ = map.maps_s_;
	maps_d_x_ = map.maps_d_x_;
	maps_d_y_ = map.maps_d_y_;
}

double Map::distance(double x1, double y1, double x2, double y2){
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));	
}

int Map::ClosestWaypoint(double x, double y){
	double closest_len = 100000; //large number
	int closest_waypoint = 0;

	for (int i = 0; i < maps_x_.size(); i++)
	{
		double map_x = maps_x_[i];
		double map_y = maps_y_[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closest_len)
		{
			closest_len = dist;
			closest_waypoint = i;
		}
	}
	return closest_waypoint;
}

double Map::ClosestDistance(double x, double y){
	double closest_len = 100000; //large number

	for (int i = 0; i < maps_x_.size(); i++)
	{
		double map_x = maps_x_[i];
		double map_y = maps_y_[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closest_len)
		{
			closest_len = dist;
		}
	}
	return closest_len;
}

double Map::FrenetSCycle(double s) {
	return s - int(s / max_s_) * max_s_;
}

std::vector<double> Map::ToFrenet(double x, double y) {
	int wp1 = ClosestWaypoint(x, y);

	double frenet_s = maps_s_[wp1];

	double ds = 0.001;

	double norm_distance = 1000;

    int b_reverse_direction = false;

	int i = 0;
	while (fabs(norm_distance) > 0.01) {
		double x0 = x_s_(frenet_s);
		double y0 = y_s_(frenet_s);
		double s1 = FrenetSCycle(frenet_s + ds);
		double dx = x_s_(s1) - x0;
		double dy = y_s_(s1) - y0;

		double slope = atan2(dy, dx);
		norm_distance = (y - y0) * sin(slope) + (x - x0) * cos(slope);
		frenet_s = FrenetSCycle(frenet_s + norm_distance * 0.8);

        if(i == 0 && norm_distance < 0){
            b_reverse_direction = true;
        }

		i++;
		if (i > 1000){

            // L2 Search
            if(wp1 == 0 || wp1 == maps_x_.size() - 1){
                frenet_s = maps_s_[wp1];
                break;
            }

            if(b_reverse_direction == true){
                double start_frenet_s = maps_s_[wp1];
                double end_frenet_s = maps_s_[wp1 - 1];
                double closest_len = 10000;
                double closest_frenet_s = 0;
                for(double iter_frenet_s = start_frenet_s; iter_frenet_s > end_frenet_s; iter_frenet_s -= 0.01){
                    double dist = distance(x, y, x_s_(iter_frenet_s), y_s_(iter_frenet_s));
                    if(dist < closest_len){
                        closest_len = dist;
                        closest_frenet_s = iter_frenet_s;
                    }
                }
                frenet_s = closest_frenet_s;
            }
            else{
                double start_frenet_s = maps_s_[wp1];
                double end_frenet_s = maps_s_[wp1 + 1];
                double closest_len = 10000;
                double closest_frenet_s = 0;
                for(double iter_frenet_s = start_frenet_s; iter_frenet_s < end_frenet_s; iter_frenet_s += 0.01){
                    double dist = distance(x, y, x_s_(iter_frenet_s), y_s_(iter_frenet_s));
                    if(dist < closest_len){
                        closest_len = dist;
                        closest_frenet_s = iter_frenet_s;
                    }
                }
                frenet_s = closest_frenet_s;
            }

            break;

        } 
	}

	double x0 = x_s_(frenet_s);
	double y0 = y_s_(frenet_s);

	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x0;
	double dy = y_s_(s1) - y0;

	double slope = atan2(dy, dx);

	double frenet_d = (y - y0) * cos(slope) - (x - x0) * sin(slope);

	return { frenet_s, frenet_d };
}

std::vector<double> Map::ToCartesian(double s, double d) {
	double frenet_s = FrenetSCycle(s);
	double x = x_s_(frenet_s);
	double y = y_s_(frenet_s);

	double ds = 0.01;
	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x;
	double dy = y_s_(s1) - y;

	double norm = atan2(dy, dx);

	x -= d*sin(norm);
	y += d*cos(norm);

	return { x, y };
}

// ToFrenetAll
// convert cartesian state into frenet state
// 
std::vector<double> Map::ToFrenetAllS( std::vector<double> state_cartesian ){
    double x = state_cartesian[0];
    double y = state_cartesian[1];
    double yaw = state_cartesian[2];
    double k = state_cartesian[3];
    double v = state_cartesian[4];
    double a = state_cartesian[5];

    std::vector<double> sn = ToFrenet( x, y );
    double s = sn[0];
    double n = sn[1];

    double yaw_r = GetSlope( s );
    double k_r = GetCurvature( s );

    // derivative of k_r
    std::vector<double> dx = x_s_.derivative( s, 3 );
    std::vector<double> dy = y_s_.derivative( s, 3 );

    double f_k = dx[0]*dy[1] - dy[0]*dx[1];
    double g_k = pow( dx[0]*dx[0] + dy[0]*dy[0], 1.5 );
    double df_k = dx[0]*dx[2] - dx[2]*dy[0];
    double dg_k = 3 * pow( dx[0]*dx[0]+dy[0]*dy[0], 0.5 ) * ( dx[0]*dx[1] + dy[0]*dy[1] );
    double dkr_s = ( df_k*g_k - f_k*dg_k ) / ( g_k*g_k );   

    // yaw_delta
    double yaw_delta = yaw - yaw_r;

    // ds
    double ds = v*cos(yaw_delta) / (1-k_r*n);

    // dn_s
    double dn_s = (1-k_r*n) * tan( yaw_delta );

    // dds
    double param = (1-k_r*n) / cos( yaw_delta );
    double dds = (1/param) * ( a - ds*ds/cos(yaw_delta)*( ((1-k_r*n) * tan(yaw_delta) * (k*param - k_r)) - (dkr_s*n+k_r*dn_s)  ) );

    // ddn_s
    double ddn_s = -( dkr_s * n + k_r * dn_s) * tan(yaw_delta)
	+ (param/cos(yaw_delta)) * ( k*param - k_r );

    return {s, ds, dds, n, dn_s, ddn_s};
}

// ToCartesianAll
// convert frenet state into cartesian
// find x, y, k, k'
// 
std::vector<double> Map::ToCartesianAllS( std::vector<double> state_frenet ){
    double s = FrenetSCycle( state_frenet[0] );
    double ds = state_frenet[1];
    double dds = state_frenet[2];
    double n = state_frenet[3];
    double dn_s = state_frenet[4];
    double ddn_s = state_frenet[5];

    std::vector<double> xy = ToCartesian( s, n );
    double yaw_r = GetSlope( s );
    double k_r = GetCurvature( s );
 
    // derivative of k_r
    std::vector<double> dx = x_s_.derivative( s, 3 );
    std::vector<double> dy = y_s_.derivative( s, 3 );

    double f_k = dx[0]*dy[1] - dy[0]*dx[1];
    double g_k = pow( dx[0]*dx[0] + dy[0]*dy[0], 1.5 );
    // double df_k = dx[0]*dx[2] - dx[2]*dy[0];
    double df_k = dx[0]*dy[2] - dx[2]*dy[0];
    // double df_k = dx[1]*dy[2] - dy[1]*dx[2];
    double dg_k = 3 * pow( dx[0]*dx[0]+dy[0]*dy[0], 0.5 ) * ( dx[0]*dx[1] + dy[0]*dy[1] );
    double dkr_s = ( df_k*g_k - f_k*dg_k ) / ( g_k*g_k );   

    // yaw
    double yaw_delta = atan2( dn_s, 1 - k_r * n );
    double yaw = yaw_r + yaw_delta;

    // spd
    double spd = ds * sqrt( (1-k_r*n)*(1-k_r*n) + dn_s*dn_s );

    // curvature
    double param_k = cos( yaw_delta ) / ( 1 - k_r * n );
    double k = param_k *
	( k_r + param_k * cos(yaw_delta)*(ddn_s + ( dkr_s*n+k_r*dn_s)*tan(yaw_delta) ) );
    double a = dds / param_k
	+ (ds*ds/cos(yaw_delta)) * ( ((1-k_r*n)*tan(yaw_delta)*(k/param_k - k_r)) - (dkr_s*n+k_r*dn_s) );
    
    return {xy[0], xy[1], yaw, k, spd, a};
}

std::vector<double> Map::ToFrenetAllT( std::vector<double> state_cartesian ){
    std::vector<double> state_s = ToFrenetAllS( state_cartesian );
    double ds = state_s[1];
    double dds = state_s[2];
    double dn_s = state_s[4];
    double ddn_s = state_s[5];
    
    double dn = ds * dn_s;
    double ddn = dds*dn_s + ds*ds*ddn_s;
    state_s[4] = dn;
    state_s[5] = ddn;
    return state_s;
}

std::vector<double> Map::ToCartesianAllT( std::vector<double> state_frenet ){
    double ds = state_frenet[1];
    double dds = state_frenet[2];
    double dn = state_frenet[4];
    double ddn = state_frenet[5];

    double dn_s = dn / ds;
    double ddn_s = (ddn-dds*dn_s) / (ds*ds);
    
    std::vector<double> state_s = {state_frenet[0], state_frenet[1], state_frenet[2],
				state_frenet[3], dn_s, ddn_s};
    return ToCartesianAllS( state_s );
}

double Map::GetSlope(double s) {
	double frenet_s = FrenetSCycle(s);
	std::vector<double> dx = x_s_.derivative(frenet_s, 2);
	std::vector<double> dy = y_s_.derivative(frenet_s, 2);

	assert(dx[0] != 0.0);

	double slop = atan2(dy[0], dx[0]);

	return slop;
}

double Map::GetCurvature(double s) {
	double frenet_s = FrenetSCycle(s);
	std::vector<double> dx = x_s_.derivative(frenet_s, 2);
	std::vector<double> dy = y_s_.derivative(frenet_s, 2);

	double square_sum = dx[0] * dx[0] + dy[0] * dy[0];
	double zero_thre = 1e-5;
	if (square_sum < zero_thre) square_sum = zero_thre;

	double curvature = (dx[0] * dy[1] - dy[0] * dx[1]) / sqrt(square_sum*square_sum*square_sum);

	return curvature;
}

std::vector<double> Map::ToFrenetVelocity(double vx, double vy, double frenet_s) {

	double x0 = x_s_(frenet_s);
	double y0 = y_s_(frenet_s);
	double ds = 0.001;
	double s1 = FrenetSCycle(frenet_s + ds);
	double dx = x_s_(s1) - x0;
	double dy = y_s_(s1) - y0;

	double slope = atan2(dy, dx);

	double cos_slope = cos(slope);
	double sin_slope = sin(slope);

	double frenet_vs = vy * sin_slope + vx * cos_slope;
	double frenet_vd = vy * cos_slope - vx * sin_slope;
	return { frenet_vs, frenet_vd };
}

double Map::SubstractS(double minuend, double subtrahend) {
	double s_1 = FrenetSCycle(minuend);
	double s_2 = FrenetSCycle(subtrahend);
	double dist = s_1 - s_2;
	double abs_dist = fabs(dist);
	if (abs_dist > max_s_ * 0.5) {
		double sign = 1;
		if (dist > 0) sign = -1;
		dist = sign * (max_s_ - abs_dist);
	}
	return dist;
}

int Map::GetLaneIndexN( double n ){
    return (int) ( (-n+lane_width_/2)  / lane_width_ );
}

int Map::GetLaneIndexXY( double x, double y){
    std::vector<double> sn = ToFrenet( x,y );
    return GetLaneIndexN( sn[1] );
}

double Map::GetOffset( int lane_idx ){
    return ( -lane_idx*lane_width_ );
}

int Map::GetMaxLaneID(){
	int max_id = 0;
	for( int i=0; i<lane_indices_.size(); i++){
		if( max_id < lane_indices_[i] ) max_id = lane_indices_[i];
	}
	return max_id;
}

int Map::GetMinLaneID(){
	int min_id = 100000;
	for( int i=0; i<lane_indices_.size(); i++){
		if( min_id > lane_indices_[i] ) min_id = lane_indices_[i];
	} 
	return min_id;
}