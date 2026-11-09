#ifndef WLA_FOLLOW_SRC_NUMPY_BLAS_H
#define WLA_FOLLOW_SRC_NUMPY_BLAS_H

// The rounding of the numpy calls the Python reference makes, measured on the
// Orin (numpy 1.26.4 on the system reference BLAS/LAPACK 3.12, 2026-11-09):
// - matmul / dot / 1-D linalg.norm go to reference BLAS, built with fused
//   multiply-add: each product is accumulated in order as acc = fma(a, b, acc);
// - linalg.norm(axis=...) and every elementwise operation are unfused;
// - linalg.solve is LAPACK dgesv: dgetrf2 (pivot, scale by the reciprocal,
//   fused Schur update) and dgetrs (fused unit-lower and upper solves).
// The rest of the core is built with -ffp-contract=off, so fusing happens
// only where std::fma is written below.
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace wla::follow::detail {

// np.dot / 1-D matmul of two 2-vectors (ddot).
[[nodiscard]] inline double BlasDot2(double a0, double a1, double b0, double b1) noexcept
{
    return std::fma(a1, b1, a0 * b0);
}

// np.linalg.norm of a 2-vector (sqrt of ddot).
[[nodiscard]] inline double BlasNorm2(double x, double y) noexcept
{
    return std::sqrt(BlasDot2(x, y, x, y));
}

// np.linalg.norm(..., axis=-1) of a 2-vector row (unfused reduce).
[[nodiscard]] inline double RowNorm2(double x, double y) noexcept
{
    return std::sqrt(x * x + y * y);
}

// C = A @ B, row-major, A is M x K and B is K x N (dgemm / dgemv).
template <std::size_t M, std::size_t K, std::size_t N>
[[nodiscard]] std::array<double, M * N> BlasMatmul(const std::array<double, M * K>& a,
                                                   const std::array<double, K * N>& b) noexcept
{
    std::array<double, M * N> c{};
    for (std::size_t i = 0; i < M; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            double acc = 0.;
            for (std::size_t l = 0; l < K; ++l)
            {
                acc = std::fma(a[i * K + l], b[l * N + j], acc);
            }
            c[i * N + j] = acc;
        }
    }
    return c;
}

// LU factors of a 2x2 matrix as dgetrf2 computes them.
struct Lu2
{
    bool m_swap{false};
    double m_u00{0.}, m_u01{0.}, m_l10{0.}, m_u11{0.};
};

// Throws std::domain_error("singular_matrix") where numpy raises LinAlgError.
[[nodiscard]] inline Lu2 Factor2(double a00, double a01, double a10, double a11)
{
    Lu2 lu;
    if (std::fabs(a10) > std::fabs(a00))  // idamax keeps the first of equal magnitudes
    {
        lu.m_swap = true;
        std::swap(a00, a10);
        std::swap(a01, a11);
    }
    if (a00 == 0.)
    {
        throw std::domain_error("singular_matrix");
    }
    // dgetrf2 scales by the reciprocal unless the pivot is below sfmin.
    lu.m_l10 = std::fabs(a00) >= DBL_MIN ? a10 * (1. / a00) : a10 / a00;
    lu.m_u00 = a00;
    lu.m_u01 = a01;
    lu.m_u11 = std::fma(-a01, lu.m_l10, a11);
    if (lu.m_u11 == 0.)
    {
        throw std::domain_error("singular_matrix");
    }
    return lu;
}

// One right-hand side through dgetrs (dlaswp, unit-lower dtrsm, upper dtrsm).
[[nodiscard]] inline std::array<double, 2> Solve2(const Lu2& lu, double b0, double b1) noexcept
{
    if (lu.m_swap)
    {
        std::swap(b0, b1);
    }
    if (b0 != 0.)
    {
        b1 = std::fma(-b0, lu.m_l10, b1);
    }
    if (b1 != 0.)
    {
        b1 = b1 / lu.m_u11;
        b0 = std::fma(-b1, lu.m_u01, b0);
    }
    if (b0 != 0.)
    {
        b0 = b0 / lu.m_u00;
    }
    return {b0, b1};
}

} // namespace wla::follow::detail

#endif // WLA_FOLLOW_SRC_NUMPY_BLAS_H
