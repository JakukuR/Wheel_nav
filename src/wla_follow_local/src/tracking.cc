// Statement-by-statement port of sim_core/tracking.py and the TargetHistory of
// sim_core/target_reference.py. Comments name the reference expression where
// the order of operations matters for bit equality.
#include <wla_follow/tracking.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "numpy_blas.h"

namespace wla::follow {

namespace {

using detail::BlasDot2;
using detail::BlasMatmul;

[[nodiscard]] bool Finite(const Point2& p) noexcept
{
    return std::isfinite(p[0]) && std::isfinite(p[1]);
}

// One VisualTrack.update step on the filter state; the caller feeds the trail
// when the measurement was accepted.
TrackUpdate Advance(KalmanState& s, const VisualTrack::Options& o, const std::optional<Point2>& z, double t,
                    bool& accepted)
{
    accepted = false;
    if (!s.m_initialised)
    {
        if (!z)
        {
            return {std::nullopt, {0., 0.}, TrackState::Lost};
        }
        s.m_x = {(*z)[0], (*z)[1], 0., 0.};
        s.m_covariance = {.01, 0., 0., 0., 0., .01, 0., 0., 0., 0., .2, 0., 0., 0., 0., .2};
        s.m_seen = s.m_time = t;
        s.m_initialised = true;
    }
    const double prediction_time = z ? t : std::min(t, s.m_seen + o.m_timeout);
    const double dt = std::max(0., prediction_time - s.m_time);
    const std::array<double, 4> x0 = s.m_x;
    const double free = (z || !o.m_coastDecay)
        ? dt : std::max(0., std::min(dt, s.m_seen + o.m_coastDecay->first - s.m_time));
    s.m_time = prediction_time;
    // F = eye(4); F[:2, 2:] = eye(2)*dt; G = vstack((eye(2)*dt*dt/2, eye(2)*dt)).
    std::array<double, 16> f{1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.};
    f[2] = f[7] = dt;
    std::array<double, 16> f_t{};
    for (std::size_t i = 0; i < 4; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            f_t[i * 4 + j] = f[j * 4 + i];
        }
    }
    const double half = dt * dt / 2;
    const std::array<double, 8> g{half, 0., 0., half, dt, 0., 0., dt};
    const std::array<double, 8> g_t{half, 0., dt, 0., 0., half, 0., dt};
    // x = F@x; P = F@P@F.T + accel_std**2*G@G.T, i.e. (F@P)@F.T + (a2*G)@G.T.
    s.m_x = BlasMatmul<4, 4, 1>(f, s.m_x);
    const double a2 = o.m_accelStd * o.m_accelStd;
    std::array<double, 8> a2g{};
    for (std::size_t k = 0; k < 8; ++k)
    {
        a2g[k] = a2 * g[k];
    }
    const auto propagated = BlasMatmul<4, 4, 4>(BlasMatmul<4, 4, 4>(f, s.m_covariance), f_t);
    const auto noise = BlasMatmul<4, 2, 4>(a2g, g_t);
    for (std::size_t k = 0; k < 16; ++k)
    {
        s.m_covariance[k] = propagated[k] + noise[k];
    }
    if (dt > free)
    {
        const double tau = o.m_coastDecay->second;
        const double k = std::exp(-(dt - free) / tau);
        const double reach = free + tau * (1. - k);
        s.m_x = {x0[0] + x0[2] * reach, x0[1] + x0[3] * reach, x0[2] * k, x0[3] * k};
    }
    if (z)
    {
        const double r = o.m_measStd * o.m_measStd;
        const std::array<double, 16>& p = s.m_covariance;
        const auto lu = detail::Factor2(p[0] + r, p[1] + 0., p[4] + 0., p[5] + r);
        const double y0 = (*z)[0] - s.m_x[0], y1 = (*z)[1] - s.m_x[1];
        bool skip = false;
        if (o.m_gate && s.m_skips < o.m_gateMaxSkips)
        {
            const auto w = detail::Solve2(lu, y0, y1);
            skip = BlasDot2(y0, y1, w[0], w[1]) > *o.m_gate;
        }
        if (skip)
        {
            ++s.m_skips;
            ++s.m_rejected;
        }
        else
        {
            s.m_skips = 0;
            // K = solve(S, P[:, :2].T).T, one right-hand side per state row.
            std::array<double, 8> gain{};
            for (std::size_t i = 0; i < 4; ++i)
            {
                const auto col = detail::Solve2(lu, p[i * 4], p[i * 4 + 1]);
                gain[i * 2] = col[0];
                gain[i * 2 + 1] = col[1];
            }
            const auto dx = BlasMatmul<4, 2, 1>(gain, {y0, y1});
            for (std::size_t k = 0; k < 4; ++k)
            {
                s.m_x[k] += dx[k];
            }
            std::array<double, 8> top{};
            std::copy(p.begin(), p.begin() + 8, top.begin());
            const auto dp = BlasMatmul<4, 2, 4>(gain, top);
            for (std::size_t k = 0; k < 16; ++k)
            {
                s.m_covariance[k] -= dp[k];
            }
            s.m_seen = t;
            accepted = true;
        }
    }
    if (t - s.m_seen > o.m_timeout)
    {
        s.m_x[2] = s.m_x[3] = 0.;
        return {Point2{s.m_x[0], s.m_x[1]}, {0., 0.}, TrackState::Lost};
    }
    return {Point2{s.m_x[0], s.m_x[1]}, {s.m_x[2], s.m_x[3]}, z ? TrackState::Visible : TrackState::Coast};
}

} // namespace

// ---------------------------------------------------------------- TargetHistory

TargetHistory::TargetHistory(const Options& options)
    : m_options(options)
{
    const double w = options.m_windowS, p = options.m_periodS, g = options.m_maxGapS;
    if (!(2. <= w && w <= 20.) || !(.2 <= p && p <= 1.) || w / p > 64 || !(.1 <= g && g <= 2.))
    {
        throw std::invalid_argument("invalid_target_history_configuration");
    }
    m_capacity = static_cast<std::size_t>(std::ceil(w / p)) + 1;
}

void TargetHistory::update(const Point2& point, double stamp)
{
    if (!Finite(point) || !std::isfinite(stamp))
    {
        throw std::invalid_argument("invalid_target_history_observation");
    }
    if (m_latest && stamp <= m_latest->m_stamp)
    {
        return;
    }
    ++m_revision;
    const auto append = [this](const Sample& sample) {
        if (m_samples.size() == m_capacity)
        {
            m_samples.pop_front();  // deque(maxlen=...)
        }
        m_samples.push_back(sample);
    };
    if (!m_latest || stamp - m_latest->m_stamp > m_options.m_maxGapS)
    {
        m_samples.clear();
        m_origin = stamp;
        m_nextIndex = 1;
        append({stamp, point});
    }
    else
    {
        const double previous_stamp = m_latest->m_stamp;
        const Point2 previous_point = m_latest->m_point;
        double sample_stamp = m_origin + static_cast<double>(m_nextIndex) * m_options.m_periodS;
        while (sample_stamp <= stamp)
        {
            const double alpha = (sample_stamp - previous_stamp) / (stamp - previous_stamp);
            append({sample_stamp, {previous_point[0] + alpha * (point[0] - previous_point[0]),
                                   previous_point[1] + alpha * (point[1] - previous_point[1])}});
            ++m_nextIndex;
            sample_stamp = m_origin + static_cast<double>(m_nextIndex) * m_options.m_periodS;
        }
    }
    m_latest = Sample{stamp, point};
    while (!m_samples.empty() && m_samples.front().m_stamp < stamp - m_options.m_windowS)
    {
        m_samples.pop_front();
    }
}

std::vector<Point2> TargetHistory::points() const
{
    std::vector<Point2> result;
    result.reserve(m_samples.size() + 1);
    for (const Sample& sample : m_samples)
    {
        result.push_back(sample.m_point);
    }
    // The live endpoint is separate from the evenly spaced history samples.
    if (m_latest && (m_samples.empty() || m_latest->m_stamp > m_samples.back().m_stamp + 1e-6))
    {
        result.push_back(m_latest->m_point);
    }
    return result;
}

std::optional<double> TargetHistory::oldestStamp() const noexcept
{
    return m_samples.empty() ? std::nullopt : std::optional<double>(m_samples.front().m_stamp);
}

std::optional<double> TargetHistory::lastObservationStamp() const noexcept
{
    return m_latest ? std::optional<double>(m_latest->m_stamp) : std::nullopt;
}

// ---------------------------------------------------------------- VisualTrack

std::string_view TrackStateName(TrackState state) noexcept
{
    switch (state)
    {
    case TrackState::Visible:
        return "visible";
    case TrackState::Coast:
        return "coast";
    case TrackState::Lost:
        break;
    }
    return "lost";
}

VisualTrack::VisualTrack(const Options& options)
    : m_options(options), m_trail(options.m_history)
{
    if (options.m_coastDecay && !(options.m_coastDecay->second > 0.))
    {
        throw std::invalid_argument("invalid_coast_decay");
    }
}

TrackUpdate VisualTrack::update(const std::optional<Point2>& z, double t)
{
    bool accepted = false;
    TrackUpdate result = Advance(m_state, m_options, z, t, accepted);
    if (accepted)
    {
        m_trail.update({m_state.m_x[0], m_state.m_x[1]}, t);
    }
    return result;
}

TrackUpdate VisualTrack::predict(double t) const
{
    KalmanState copy = m_state;
    bool accepted = false;
    return Advance(copy, m_options, std::nullopt, t, accepted);
}

std::optional<Point2> VisualTrack::predictedPosition(double t) const
{
    const KalmanState& s = m_state;
    if (!s.m_initialised)
    {
        return std::nullopt;
    }
    const double dt = std::max(0., std::min(t, s.m_seen + m_options.m_timeout) - s.m_time);
    const double free = !m_options.m_coastDecay
        ? dt : std::max(0., std::min(dt, s.m_seen + m_options.m_coastDecay->first - s.m_time));
    if (dt > free)
    {
        const double tau = m_options.m_coastDecay->second;
        const double reach = free + tau * (1. - std::exp(-(dt - free) / tau));
        return Point2{s.m_x[0] + s.m_x[2] * reach, s.m_x[1] + s.m_x[3] * reach};
    }
    return Point2{s.m_x[0] + s.m_x[2] * dt, s.m_x[1] + s.m_x[3] * dt};
}

// ---------------------------------------------------------------- PeopleTracks

namespace {

[[nodiscard]] VisualTrack::Options PeopleTrackOptions()
{
    VisualTrack::Options options;
    options.m_timeout = 1.5;
    options.m_gate = 13.8;
    return options;
}

[[nodiscard]] PersonEstimate Estimate(std::int64_t id, const TrackUpdate& update, double age)
{
    // r = .28 + .035 + .15*age
    return {id, *update.m_position, update.m_velocity, .28 + .035 + .15 * age, age};
}

} // namespace

PeopleTracks::PeopleTracks() = default;
PeopleTracks::~PeopleTracks() = default;

std::vector<PersonEstimate> PeopleTracks::update(std::span<const Detection> detections, double t)
{
    const auto find = [this](std::int64_t id) {
        return std::find_if(m_tracks.begin(), m_tracks.end(), [id](const auto& item) { return item.first == id; });
    };
    for (std::size_t a = 0; a < detections.size(); ++a)
    {
        for (std::size_t b = a + 1; b < detections.size(); ++b)
        {
            if (detections[a].m_id == detections[b].m_id)
            {
                throw std::invalid_argument("duplicate_people_detection");
            }
        }
        if (!Finite(detections[a].m_point))
        {
            throw std::invalid_argument("nonfinite_people_detection");
        }
    }
    for (const Detection& detection : detections)
    {
        if (find(detection.m_id) == m_tracks.end())
        {
            m_tracks.emplace_back(detection.m_id, std::make_unique<VisualTrack>(PeopleTrackOptions()));
        }
    }
    std::vector<PersonEstimate> result;
    for (auto it = m_tracks.begin(); it != m_tracks.end();)
    {
        std::optional<Point2> z;
        for (const Detection& detection : detections)
        {
            if (detection.m_id == it->first)
            {
                z = detection.m_point;
                break;
            }
        }
        const TrackUpdate update = it->second->update(z, t);
        const double age = t - it->second->state().m_seen;
        if (age > MAX_AGE_S)
        {
            it = m_tracks.erase(it);
            continue;
        }
        result.push_back(Estimate(it->first, update, age));
        ++it;
    }
    return result;
}

std::vector<PersonEstimate> PeopleTracks::predict(double t) const
{
    std::vector<PersonEstimate> result;
    for (const auto& [id, track] : m_tracks)
    {
        const TrackUpdate update = track->predict(t);
        const double age = t - track->state().m_seen;
        if (age > MAX_AGE_S)
        {
            continue;
        }
        result.push_back(Estimate(id, update, age));
    }
    return result;
}

bool PeopleTracks::drop(std::int64_t id)
{
    const auto it = std::find_if(m_tracks.begin(), m_tracks.end(), [id](const auto& item) { return item.first == id; });
    if (it == m_tracks.end())
    {
        return false;
    }
    m_tracks.erase(it);
    return true;
}

std::vector<std::int64_t> PeopleTracks::ids() const
{
    std::vector<std::int64_t> result;
    result.reserve(m_tracks.size());
    for (const auto& item : m_tracks)
    {
        result.push_back(item.first);
    }
    return result;
}

} // namespace wla::follow
