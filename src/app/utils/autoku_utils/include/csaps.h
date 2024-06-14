#pragma once
#ifndef CSAPS_H
#define CSAPS_H

#include <limits>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>


namespace csaps
{

using Index = Eigen::DenseIndex;
using Size = Index;
using DoubleArray = Eigen::ArrayXd;
using DoubleArray2D = Eigen::ArrayXXd;
using IndexArray = Eigen::Array<Index, Eigen::Dynamic, 1>;
using DoubleSparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, Index>;

using DoubleLimits = std::numeric_limits<double>;


//! Calculates the 1-th discrete difference
DoubleArray Diff(const DoubleArray &vec);


//! Returns the indices of the bins to which each value in input array belongs
IndexArray Digitize(const DoubleArray &arr, const DoubleArray &bins);


//! Makes rows x cols sparse matrix from diagonals and offsets
DoubleSparseMatrix MakeSparseDiagMatrix(const DoubleArray2D& diags, const IndexArray& offsets, Size rows, Size cols);


//! Solves sparse linear system Ab = x via supernodal LU factorization
DoubleArray SolveLinearSystem(const DoubleSparseMatrix &A, const DoubleArray &b);




class UnivariateCubicSmoothingSpline
{
public:
  UnivariateCubicSmoothingSpline(const DoubleArray &xdata, const DoubleArray &ydata);
  UnivariateCubicSmoothingSpline(const DoubleArray &xdata, const DoubleArray &ydata, const DoubleArray &weights);
  UnivariateCubicSmoothingSpline(const DoubleArray &xdata, const DoubleArray &ydata, double smooth);
  UnivariateCubicSmoothingSpline(const DoubleArray &xdata, const DoubleArray &ydata, const DoubleArray &weights, double smooth);

  DoubleArray operator()(const DoubleArray &xidata);
  DoubleArray operator()(const Size pcount, DoubleArray &xidata);

  DoubleArray deriv(int order) const;
  
  double GetSmooth() const { return m_smooth; }
  const DoubleArray& GetBreaks() const { return m_xdata; }
  const DoubleArray2D& GetCoeffs() const { return m_coeffs; }
  Index GetPieces() const { return m_coeffs.rows(); }

protected:
  void MakeSpline();
  DoubleArray Evaluate(const DoubleArray &xidata);

protected:
  DoubleArray m_xdata;
  DoubleArray m_ydata;
  DoubleArray m_weights;

  DoubleArray m_xidata;
  DoubleArray m_yodata;

  double m_smooth;

  DoubleArray2D m_coeffs;
};

} // namespace csaps

#endif // CSAPS_H
