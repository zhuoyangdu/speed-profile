#ifndef OFFBOARD_CONTROL_TEST_FAKE_PLANNING_TEST_SRC_SPLINE_H_
#define OFFBOARD_CONTROL_TEST_FAKE_PLANNING_TEST_SRC_SPLINE_H_

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>

namespace planning {

// band matrix solver
class band_matrix {
private:
    std::vector< std::vector<double> > m_upper;  // upper band
    std::vector< std::vector<double> > m_lower;  // lower band

public:
    band_matrix() {}                             // constructor
    band_matrix(int dim, int n_u, int n_l);       // constructor
    ~band_matrix() {}                            // destructor
    void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
    int dim() const;                             // matrix dimension
    int num_upper() const {
        return m_upper.size()-1;
    }
    int num_lower() const {
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
                                 bool is_lu_decomposed = false);
};


// spline interpolation
class Spline {
public:
    enum bd_type {
        first_deriv = 1,
        second_deriv = 2
    };

    std::vector<double> m_x, m_y;            // x,y coordinates of points
    // interpolation parameters
    // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
    std::vector<double> m_a, m_b, m_c;        // spline coefficients
    double  m_b0, m_c0;                     // for left extrapol
    bd_type m_left, m_right;
    double  m_left_value, m_right_value;
    bool    m_force_linear_extrapolation;

    // set default boundary condition to be zero curvature at both ends
    Spline(): m_left(second_deriv), m_right(second_deriv),
        m_left_value(0.0), m_right_value(0.0), m_b0(0.0), m_c0(0.0),
        m_force_linear_extrapolation(false) {}

    // optional, but if called it has to come be before setPoints()
    void setBoundary(bd_type left, double left_value,
                      bd_type right, double right_value,
                      bool force_linear_extrapolation = false);

    void setPoints(const std::vector<double>& x,
                    const std::vector<double>& y, bool cubic_spline = true);

    double operator() (double x) const;
    double deriv1(double x) const;
    double deriv2(double x) const;

    void interpolateAscendingPoints(
        const std::vector<double>& xs,
        std::vector<double>* ys) const;

    void getRange(double* min_x, double* max_x) const;

    static void getClosestPointOnCurve(
        const Spline& curve_x,
        const Spline& curve_y,
        double x0, double y0,
        double* s, double* dist,
        int max_iteration = 20,
        double converge_threshold = 1e-3,
        bool use_bisection = true,
        double bisection_threshold = 1);

    static void getClosestPointOnCurveWithExtension(
        const Spline& curve_x, const Spline& curve_y,
        double x0, double y0,
        double* s, double* dist,
        int max_iteration = 20,
        double converge_threshold = 1e-3,
        bool use_bisection = true,
        double bisection_threshold = 1);

    static void fitCurve(
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        Spline* curve_x, Spline* curve_y,
        double* length);
};

}  // namespace av_planning

#endif  //  OFFBOARD_CONTROL_TEST_FAKE_PLANNING_TEST_SRC_SPLINE_H_
