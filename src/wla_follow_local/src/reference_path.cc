// Port of bezier_reference and bspline_corners (sim_core/target_reference.py).
// Every numpy call is reproduced with its rounding (see numpy_blas.h): 1-D
// norms and dots fused, row norms and elementwise arithmetic unfused, einsum
// and cumsum summed in order, linspace as i*step + start with the exact end.
#include <wla_follow/tracking.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <wla_follow/rolling_map.h>  // NumpyInterp

#include "numpy_blas.h"

namespace wla::follow {

namespace {

using detail::BlasDot2;
using detail::BlasNorm2;
using detail::RowNorm2;

// np.linspace(start, stop, num) with endpoint=True.
[[nodiscard]] std::vector<double> Linspace(double start, double stop, std::size_t num)
{
    std::vector<double> y(num);
    const double delta = stop - start;
    if (num > 1)
    {
        const double div = static_cast<double>(num - 1);
        const double step = delta / div;
        for (std::size_t i = 0; i < num; ++i)
        {
            const double index = static_cast<double>(i);
            y[i] = (step == 0. ? index / div * delta : index * step) + start;
        }
        y[num - 1] = stop;
    }
    else if (num == 1)
    {
        y[0] = 0. * delta + start;
    }
    return y;
}

} // namespace

std::vector<Point2> BezierReference(std::span<const Point2> history)
{
    for (const Point2& point : history)
    {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]))
        {
            throw std::invalid_argument("nonfinite_target_history");
        }
    }
    if (history.size() < 2)
    {
        return {history.begin(), history.end()};
    }
    // Suppress stationary jitter without dropping the actual endpoint.
    std::vector<Point2> knots{history.front()};
    for (std::size_t i = 1; i + 1 < history.size(); ++i)
    {
        const Point2& point = history[i];
        if (BlasNorm2(point[0] - knots.back()[0], point[1] - knots.back()[1]) >= .03)
        {
            knots.push_back(point);
        }
    }
    const Point2& last = history.back();
    if (BlasNorm2(last[0] - knots.back()[0], last[1] - knots.back()[1]) > 1e-6)
    {
        knots.push_back(last);
    }
    else
    {
        knots.back() = last;
    }
    const std::size_t m = knots.size();
    if (m < 2)
    {
        return knots;
    }
    // shift = .25*(knots[:-2] + knots[2:] - 2.*knots[1:-1]), at most 5 cm.
    std::vector<Point2> anchor = knots;
    for (std::size_t i = 1; i + 1 < m; ++i)
    {
        Point2 shift;
        for (std::size_t d = 0; d < 2; ++d)
        {
            shift[d] = .25 * ((knots[i - 1][d] + knots[i + 1][d]) - 2. * knots[i][d]);
        }
        const double scale = std::min(1., .05 / std::max(RowNorm2(shift[0], shift[1]), 1e-12));
        anchor[i][0] += shift[0] * scale;
        anchor[i][1] += shift[1] * scale;
    }
    std::vector<double> length(m - 1);
    std::vector<Point2> direction(m - 1);
    for (std::size_t i = 0; i + 1 < m; ++i)
    {
        const double cx = anchor[i + 1][0] - anchor[i][0], cy = anchor[i + 1][1] - anchor[i][1];
        length[i] = RowNorm2(cx, cy);
        const double denominator = std::max(length[i], 1e-12);
        direction[i] = {cx / denominator, cy / denominator};
    }
    std::vector<Point2> handle(m);
    for (std::size_t i = 0; i < m; ++i)
    {
        Point2 tangent;
        if (i == 0)
        {
            tangent = direction.front();
        }
        else if (i == m - 1)
        {
            tangent = direction.back();
        }
        else
        {
            tangent = {direction[i - 1][0] + direction[i][0], direction[i - 1][1] + direction[i][1]};
        }
        const double before = i == 0 ? length[0] : length[i - 1];
        const double after = i == m - 1 ? length[m - 2] : length[i];
        const double handle_length = std::min(.05, std::min(before, after) / 3.);
        const double norm = std::max(RowNorm2(tangent[0], tangent[1]), 1e-12);
        handle[i] = {tangent[0] / norm * handle_length, tangent[1] / norm * handle_length};
    }
    // Eight samples per cubic; einsum('uk,skd->sud') sums the four terms in
    // order: unfused, except for a single cubic, where numpy drops the size-1
    // axis and its multiply-add loop fuses (measured, numpy 1.26.4 on Orin).
    const bool fused = m == 2;
    std::array<std::array<double, 4>, 8> basis{};
    for (std::size_t k = 0; k < 8; ++k)
    {
        const double u = static_cast<double>(k) / 8.;
        const double v = 1 - u;
        basis[k] = {std::pow(v, 3.), 3 * (v * v) * u, 3 * v * (u * u), std::pow(u, 3.)};
    }
    std::vector<Point2> curve;
    curve.reserve(8 * (m - 1) + 1);
    for (std::size_t s = 0; s + 1 < m; ++s)
    {
        const std::array<Point2, 4> controls{
            anchor[s],
            Point2{anchor[s][0] + handle[s][0], anchor[s][1] + handle[s][1]},
            Point2{anchor[s + 1][0] - handle[s + 1][0], anchor[s + 1][1] - handle[s + 1][1]},
            anchor[s + 1]};
        for (const auto& b : basis)
        {
            Point2 point;
            for (std::size_t d = 0; d < 2; ++d)
            {
                double acc = 0.;
                for (std::size_t k = 0; k < 4; ++k)
                {
                    acc = fused ? std::fma(b[k], controls[k][d], acc) : acc + b[k] * controls[k][d];
                }
                point[d] = acc;
            }
            curve.push_back(point);
        }
    }
    curve.push_back(anchor.back());
    std::vector<Point2> result{curve.front()};
    for (std::size_t i = 1; i < curve.size(); ++i)
    {
        if (RowNorm2(curve[i][0] - curve[i - 1][0], curve[i][1] - curve[i - 1][1]) > 1e-9)
        {
            result.push_back(curve[i]);
        }
    }
    return result;
}

std::vector<Point2> BsplineCorners(std::span<const Point2> path, double corner, double step)
{
    if (path.empty())
    {
        throw std::invalid_argument("empty_reference_path");  // the reference raises IndexError
    }
    std::vector<Point2> raw;
    raw.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (i == 0 || RowNorm2(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]) > 1e-6)
        {
            raw.push_back(path[i]);
        }
    }
    if (raw.size() < 3 || corner <= 0)
    {
        return raw;
    }
    const double straight = std::cos(3. * (std::numbers::pi / 180.));  // math.cos(math.radians(3.))
    std::vector<Point2> controls{raw.front()};
    for (std::size_t i = 1; i + 1 < raw.size(); ++i)
    {
        const double bx = raw[i][0] - raw[i - 1][0], by = raw[i][1] - raw[i - 1][1];
        const double ax = raw[i + 1][0] - raw[i][0], ay = raw[i + 1][1] - raw[i][1];
        const double lb = BlasNorm2(bx, by), la = BlasNorm2(ax, ay);
        const Point2 ub{bx / lb, by / lb}, ua{ax / la, ay / la};
        if (BlasDot2(ub[0], ub[1], ua[0], ua[1]) > straight)
        {
            controls.push_back(raw[i]);
            continue;
        }
        const double d = std::min({corner, .45 * lb, .45 * la});
        controls.push_back({raw[i][0] - d * ub[0], raw[i][1] - d * ub[1]});
        controls.push_back(raw[i]);
        controls.push_back({raw[i][0] + d * ua[0], raw[i][1] + d * ua[1]});
    }
    controls.push_back(raw.back());
    const std::size_t n = controls.size();
    constexpr std::size_t K = 3;
    if (n < 4)
    {
        return raw;
    }
    std::vector<double> knots(K, 0.);
    const auto inner = Linspace(0., 1., n - K + 1);
    knots.insert(knots.end(), inner.begin(), inner.end());
    knots.insert(knots.end(), K, 1.);
    const auto u = Linspace(0., 1., std::max<std::size_t>(32, 12 * n));
    std::vector<Point2> curve(u.size());
    for (std::size_t q = 0; q < u.size(); ++q)
    {
        // span = clip(searchsorted(knots, u, side='right') - 1, k, n - 1)
        const auto right = std::upper_bound(knots.begin(), knots.end(), u[q]);
        const std::ptrdiff_t found = (right - knots.begin()) - 1;
        const std::size_t span = static_cast<std::size_t>(
            std::clamp<std::ptrdiff_t>(found, K, static_cast<std::ptrdiff_t>(n - 1)));
        std::array<Point2, K + 1> d;
        for (std::size_t j = 0; j <= K; ++j)
        {
            d[j] = controls[span - K + j];
        }
        for (std::size_t r = 1; r <= K; ++r)
        {
            for (std::size_t j = K; j >= r; --j)
            {
                const std::size_t i = span - K + j;
                const double left = knots[i], right_knot = knots[i + K + 1 - r];
                const double a = (u[q] - left) / (right_knot > left ? right_knot - left : 1.);
                for (std::size_t c = 0; c < 2; ++c)
                {
                    d[j][c] = (1. - a) * d[j - 1][c] + a * d[j][c];
                }
            }
        }
        curve[q] = d[K];
    }
    curve.back() = raw.back();
    std::vector<double> arcs(curve.size(), 0.);
    for (std::size_t i = 1; i < curve.size(); ++i)
    {
        arcs[i] = arcs[i - 1] + RowNorm2(curve[i][0] - curve[i - 1][0], curve[i][1] - curve[i - 1][1]);
    }
    const double steps = arcs.back() / step;
    if (!std::isfinite(steps))
    {
        throw std::invalid_argument("invalid_bspline_step");
    }
    const std::size_t count = std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(steps)) + 1);
    const auto s = Linspace(0., arcs.back(), count);
    std::vector<double> xs(curve.size()), ys(curve.size());
    for (std::size_t i = 0; i < curve.size(); ++i)
    {
        xs[i] = curve[i][0];
        ys[i] = curve[i][1];
    }
    std::vector<Point2> result(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        result[i] = {NumpyInterp(s[i], arcs.data(), xs.data(), arcs.size(), 1),
                     NumpyInterp(s[i], arcs.data(), ys.data(), arcs.size(), 1)};
    }
    return result;
}

} // namespace wla::follow
