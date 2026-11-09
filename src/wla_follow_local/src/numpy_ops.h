#ifndef WLA_FOLLOW_SRC_NUMPY_OPS_H
#define WLA_FOLLOW_SRC_NUMPY_OPS_H

// numpy / scipy operations of the Python reference with their exact results
// (numpy 1.26.4, scipy 1.11.4 on the Orin, measured or ported from source):
// linspace, sinc, clip, the default argsort (introsort, ties included) and
// scipy.sparse.csgraph.dijkstra (Fibonacci heap: same predecessors on ties).
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace wla::follow::detail {

// np.linspace(start, stop, num), endpoint=True, scalar ends.
[[nodiscard]] inline std::vector<double> Linspace(double start, double stop, std::size_t num)
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

// np.linspace(a, b, num) with 2-vector ends: if any step component is zero,
// numpy divides by div and multiplies by delta for every component.
[[nodiscard]] inline std::vector<std::array<double, 2>> Linspace2(const std::array<double, 2>& a,
                                                                 const std::array<double, 2>& b, std::size_t num)
{
    std::vector<std::array<double, 2>> y(num);
    const std::array<double, 2> delta{b[0] - a[0], b[1] - a[1]};
    if (num > 1)
    {
        const double div = static_cast<double>(num - 1);
        const std::array<double, 2> step{delta[0] / div, delta[1] / div};
        const bool any_zero = step[0] == 0. || step[1] == 0.;
        for (std::size_t i = 0; i < num; ++i)
        {
            const double index = static_cast<double>(i);
            for (std::size_t d = 0; d < 2; ++d)
            {
                y[i][d] = (any_zero ? index / div * delta[d] : index * step[d]) + a[d];
            }
        }
        y[num - 1] = b;
    }
    else if (num == 1)
    {
        y[0] = {0. * delta[0] + a[0], 0. * delta[1] + a[1]};
    }
    return y;
}

// np.sinc
[[nodiscard]] inline double Sinc(double x) noexcept
{
    const double y = std::numbers::pi * (x == 0. ? 1.0e-20 : x);
    return std::sin(y) / y;
}

// common.wrap: np.arctan2(np.sin(a), np.cos(a))
[[nodiscard]] inline double Wrap(double a) noexcept
{
    return std::atan2(std::sin(a), std::cos(a));
}

// np.clip for finite bounds
[[nodiscard]] inline double Clip(double x, double lo, double hi) noexcept
{
    const double low = (x >= lo || std::isnan(x)) ? x : lo;
    return (low <= hi || std::isnan(low)) ? low : hi;
}

// np.sign
[[nodiscard]] inline double Sign(double x) noexcept
{
    return x > 0. ? 1. : (x < 0. ? -1. : (x == 0. ? 0. : x));
}

namespace sort {

[[nodiscard]] inline bool Less(double a, double b) noexcept
{
    return a < b || (b != b && a == a);  // numpy: NaN sorts last
}

inline void AHeapSort(const double* v, std::int64_t* tosort, std::int64_t n)
{
    std::int64_t* a = tosort - 1;  // one-based heap indexing
    std::int64_t i, j, l, tmp;
    for (l = n >> 1; l > 0; --l)
    {
        tmp = a[l];
        for (i = l, j = l << 1; j <= n;)
        {
            if (j < n && Less(v[a[j]], v[a[j + 1]]))
            {
                j += 1;
            }
            if (Less(v[tmp], v[a[j]]))
            {
                a[i] = a[j];
                i = j;
                j += j;
            }
            else
            {
                break;
            }
        }
        a[i] = tmp;
    }
    for (; n > 1;)
    {
        tmp = a[n];
        a[n] = a[1];
        n -= 1;
        for (i = 1, j = 2; j <= n;)
        {
            if (j < n && Less(v[a[j]], v[a[j + 1]]))
            {
                j++;
            }
            if (Less(v[tmp], v[a[j]]))
            {
                a[i] = a[j];
                i = j;
                j += j;
            }
            else
            {
                break;
            }
        }
        a[i] = tmp;
    }
}

} // namespace sort

// np.argsort(values) (kind='quicksort': numpy's aquicksort_, bit for bit,
// so equal values come out in the same order).
[[nodiscard]] inline std::vector<std::int64_t> ArgSort(const std::vector<double>& values)
{
    constexpr std::int64_t SMALL_QUICKSORT = 15;
    const std::int64_t num = static_cast<std::int64_t>(values.size());
    std::vector<std::int64_t> order(values.size());
    for (std::int64_t k = 0; k < num; ++k)
    {
        order[static_cast<std::size_t>(k)] = k;
    }
    if (num < 2)
    {
        return order;
    }
    const double* v = values.data();
    std::int64_t* tosort = order.data();
    std::int64_t* pl = tosort;
    std::int64_t* pr = tosort + num - 1;
    std::int64_t* stack[128];
    std::int64_t** sptr = stack;
    std::int64_t *pm, *pi, *pj, *pk, vi;
    double vp;
    int depth[128];
    int* psdepth = depth;
    int cdepth = 0;
    for (std::uint64_t unum = static_cast<std::uint64_t>(num); unum >>= 1;)
    {
        ++cdepth;
    }
    cdepth *= 2;
    for (;;)
    {
        if (cdepth < 0)
        {
            sort::AHeapSort(v, pl, pr - pl + 1);
            goto stack_pop;
        }
        while ((pr - pl) > SMALL_QUICKSORT)
        {
            pm = pl + ((pr - pl) >> 1);
            if (sort::Less(v[*pm], v[*pl]))
            {
                std::swap(*pm, *pl);
            }
            if (sort::Less(v[*pr], v[*pm]))
            {
                std::swap(*pr, *pm);
            }
            if (sort::Less(v[*pm], v[*pl]))
            {
                std::swap(*pm, *pl);
            }
            vp = v[*pm];
            pi = pl;
            pj = pr - 1;
            std::swap(*pm, *pj);
            for (;;)
            {
                do
                {
                    ++pi;
                } while (sort::Less(v[*pi], vp));
                do
                {
                    --pj;
                } while (sort::Less(vp, v[*pj]));
                if (pi >= pj)
                {
                    break;
                }
                std::swap(*pi, *pj);
            }
            pk = pr - 1;
            std::swap(*pi, *pk);
            if (pi - pl < pr - pi)
            {
                *sptr++ = pi + 1;
                *sptr++ = pr;
                pr = pi - 1;
            }
            else
            {
                *sptr++ = pl;
                *sptr++ = pi - 1;
                pl = pi + 1;
            }
            *psdepth++ = --cdepth;
        }
        for (pi = pl + 1; pi <= pr; ++pi)
        {
            vi = *pi;
            vp = v[vi];
            pj = pi;
            pk = pi - 1;
            while (pj > pl && sort::Less(vp, v[*pk]))
            {
                *pj-- = *pk--;
            }
            *pj = vi;
        }
    stack_pop:
        if (sptr == stack)
        {
            break;
        }
        pr = *(--sptr);
        pl = *(--sptr);
        cdepth = *(--psdepth);
    }
    return order;
}

// Directed graph in scipy CSR form: rows by source, columns ascending.
struct Csr
{
    std::vector<int> m_indptr;
    std::vector<int> m_indices;
    std::vector<double> m_weights;
};

// coo_matrix((w, (src, dst)), shape=(n, n)).tocsr(): duplicates summed.
[[nodiscard]] Csr BuildCsr(int n, const std::vector<int>& src, const std::vector<int>& dst,
                           const std::vector<double>& weights);

// scipy.sparse.csgraph.dijkstra(graph, indices=source, return_predecessors=True)
// (directed): distances (inf unreached) and predecessors (-9999 for none).
void Dijkstra(const Csr& graph, int source, std::vector<double>& distance, std::vector<int>& predecessor);

} // namespace wla::follow::detail

#endif // WLA_FOLLOW_SRC_NUMPY_OPS_H
