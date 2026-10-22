# Machine-constant and contention types for the parallel planners.
#
# `MachineConstants` holds the rates `calibrate_machine` measures once per
# machine; the predictive campaign planner reads its USL terms.
# `ContentionInputs` holds what one warm RHS pass shows about how a workload
# scales; the native-lock width cap reads its lock occupancy.

"""
    ContentionInputs

What one warm RHS pass reveals about how this workload will behave under a
split, as opposed to how much work it contains.

It answers "how badly does this work scale", which needs different evidence
from how much work there is. It measures two of the mechanisms found to cap
thread scaling in this codebase:

- `alloc_bytes_per_pass` / `pass_ns`  allocation pressure. Shared-heap
                    allocation and the GC it drives are process-wide, which is
                    why separate processes escape a ceiling that more threads
                    only worsen. The measured contrast is stark and mechanical:
                    a vacuum constellation iterates a homogeneous effector
                    1-tuple and allocates 0 bytes per pass, while the same
                    physics plus atmosphere reaches tens of GiB over a solve and
                    inverts past four threads.
- `lock_*`          native-lock occupancy, read straight off the shared
                    GRAM/SPICE critical section. Unlike allocation this is
                    not a proxy: `lock_hold_ns` over the window is an Amdahl
                    serial fraction with a hard `1/rho` speedup ceiling, and
                    `lock_wait_ns / lock_hold_ns` reports whether the width in
                    use is already past what the lock admits.

`valid` is false when the warm pass could not be completed, in which case every
field is meaningless rather than zero -- a workload that allocates nothing and
one that could not be measured must not read the same.
"""
Base.@kwdef struct ContentionInputs
    alloc_bytes_per_pass::Float64 = 0.0
    pass_ns::Float64 = 0.0
    n_active_sats::Int = 0
    lock_acquisitions::Int64 = 0
    lock_hold_ns::Float64 = 0.0
    lock_wait_ns::Float64 = 0.0
    valid::Bool = false
end

"""
    alloc_bytes_per_second(ci) -> Float64

Allocation rate of one worker running this workload. Zero for an invalid or
zero-length measurement, never `Inf`: a caller sizing a width from this must get
"no pressure observed" rather than a poisoned number.
"""
@inline function alloc_bytes_per_second(ci::ContentionInputs)::Float64
    (ci.valid && ci.pass_ns > 0.0) || return 0.0
    return ci.alloc_bytes_per_pass * 1.0e9 / ci.pass_ns
end

"""
    lock_duty_cycle(ci, workers = 1) -> Float64

Fraction rho of available worker-time spent inside the native lock. `1/rho`
bounds achievable speedup at any width.
"""
@inline function lock_duty_cycle(ci::ContentionInputs, workers::Integer = 1)::Float64
    window = ci.pass_ns * max(1.0, Float64(workers))
    (ci.valid && window > 0.0) || return 0.0
    return clamp(ci.lock_hold_ns / window, 0.0, 1.0)
end

"""
    lock_wait_hold_ratio(ci) -> Float64

Time blocked outside the native lock against time spent inside it. Much greater
than one means the split is already wider than the lock admits, and the finding
needs no history to reach: it is visible in the window that measured it.
"""
@inline function lock_wait_hold_ratio(ci::ContentionInputs)::Float64
    (ci.valid && ci.lock_hold_ns > 0.0) || return 0.0
    return ci.lock_wait_ns / ci.lock_hold_ns
end

"""
    RateCurve

A machine rate that depends on the size of the thing it operates on, stored as
measured knots and interpolated in log2 of that size.

Scalars were the original design and the measurements rejected them. Both of
the model's discriminating rates move by more than a factor of two across the
sizes real workloads produce, on this 12-core box (L1d 48K, L2 1M, L3 32M):

    coefficient touch, by table footprint
        3.4 KiB  0.103 ns      315 KiB  0.240 ns
       20.3 KiB  0.125 ns     1256 KiB  0.299 ns
       79.7 KiB  0.174 ns    20025 KiB  0.346 ns          -> 3.4x spread

    SIMD lane-term, by batch width
        B=16     0.228 ns      B=4096   0.0227 ns
        B=256    0.0259 ns     B=16384  0.0435 ns
        B=1024   0.0227 ns     B=262144 0.0551 ns

A single number for either would be wrong by 2-3x at the extremes, and a 2x
error in the coefficient-touch rate alone shifts the predicted `satellite_batch`
cost by 49% -- enough to invert the routing decision it exists to make.

The shape is physical, not noise: flat while the working set sits in L1, rising
through the L1-to-L2 and L2-to-L3 transitions, flattening again in L3. The small
batch-width end is a different effect -- per-pass loop overhead that cannot
amortize over a short vector -- and is absorbed into the same curve rather than
modelled as a separate term, because what is measured at each knot is the
*effective* per-lane cost with that overhead already amortized.

Knots are interpolated linearly in log2 of the size and clamped outside the
measured range, so a workload larger or smaller than anything calibrated gets
the nearest measured rate rather than an extrapolation off the end of a curve
whose shape is only known where it was sampled.
"""
struct RateCurve
    log2_size::Vector{Float64}
    ns::Vector{Float64}

    function RateCurve(log2_size::Vector{Float64}, ns::Vector{Float64})
        length(log2_size) == length(ns) ||
            throw(ArgumentError("RateCurve knots and values must have equal length."))
        isempty(log2_size) && throw(ArgumentError("RateCurve needs at least one knot."))
        issorted(log2_size) ||
            throw(ArgumentError("RateCurve knots must be sorted ascending in log2 size."))
        return new(log2_size, ns)
    end
end

"""
    rate_at(curve, size) -> Float64

The rate at `size`, linearly interpolated in log2 and clamped at both ends.
"""
function rate_at(curve::RateCurve, size::Real)::Float64
    n = length(curve.log2_size)
    n == 1 && return curve.ns[1]
    x = log2(max(1.0, Float64(size)))
    x <= curve.log2_size[1] && return curve.ns[1]
    x >= curve.log2_size[n] && return curve.ns[n]
    @inbounds for i in 2:n
        if x <= curve.log2_size[i]
            x0, x1 = curve.log2_size[i - 1], curve.log2_size[i]
            y0, y1 = curve.ns[i - 1], curve.ns[i]
            w = (x - x0) / max(1e-12, x1 - x0)
            return y0 + w * (y1 - y0)
        end
    end
    return curve.ns[n]
end

"""
    MachineConstants

Per-machine rates measured by [`calibrate_machine`](@ref), plus the
reference-kernel reading that validates them. The predictive campaign planner
reads `usl_alpha_base` and `usl_beta_alloc`. The other rates were measured for
the RHS-plan predictor, removed in 0.6.0, and nothing reads them now.

Produced by `scripts/calibration/calibrate_machine.jl` and cached in a fingerprinted TOML.
Never fitted from the benchmark case catalog: the catalog is held out so that a
predicted-versus-actual comparison over it is an out-of-sample test rather than
a restatement of the fit.

- `simd_lane`          [`RateCurve`] cost of one vectorized inner-loop iteration
                       per satellite, as a function of the satellite batch width
                       the loop runs over.
- `coeff_touch`        [`RateCurve`] cost of one coefficient-table read, as a
                       function of the table footprint being strided over.
                       Calibrated rather than derived from element size: the
                       coefficient matrices are column-major and the kernel walks
                       a row, so each read strides by `(L+2)*8` bytes onto its own
                       cache line, and what it costs is set by which cache level
                       holds the table -- not by `sizeof(Float64)`.
- `parallel_speedup`   [`RateCurve`] ACHIEVED speedup against worker count, not
                       assumed linear scaling. Measured here: 0.99 efficiency at
                       one worker falling to 0.26 at twelve, saturating near 3x,
                       and the real harmonics kernel shows the same ~2.3x
                       ceiling. Dividing work by the worker count over-credits
                       every wide plan, and no correction to the other constants
                       can compensate for a term that is simply absent.
- `ns_per_scalar_item` cost of one per-satellite scalar unit.
- `ns_per_queue_node`  per-node flat-queue bookkeeping.
- `dispatch_pool_ns_base` / `dispatch_pool_ns_per_worker`
- `dispatch_batch_ns_base` / `dispatch_batch_ns_per_worker`
                       affine models of one parallel dispatch and join, ONE PER
                       MECHANISM. The routes do not share a dispatcher: the flat
                       routes go through the persistent channel pool
                       (`threaded_foreach_worker_persistent`) while
                       `satellite_batch` goes through `Polyester.@batch`, and
                       they differ by enough to invert a decision. A single
                       constant measured from a third mechanism (`Threads.@spawn`)
                       was the first version of this, and it drove the predictor
                       to 25% decision accuracy against the real kernel: it
                       systematically preferred narrow allotments because it
                       could not see what a wide one really cost on the route it
                       was actually taken on.
- `ns_per_atomic`      one contended atomic read-modify-write, which is what the
                       dynamic scheduler pays per chunk.
- `reference_fma_ns` / `reference_mem_ns`
                       the two reference kernels' costs at calibration time, in
                       ns per lane and ns per cache-line touch. Two rather than
                       one because normalisation only cancels the part of a
                       contention slowdown that the reference and the measured
                       kernel share; an arithmetic reference tracks a
                       stride-bound term poorly (measured: 14.6% raw drift under
                       memory load, still 6.5% after FMA-only normalisation).
                       Arithmetic terms normalise against the first,
                       stride-bound terms against the second.

                       `reference_fma_ns` doubles as the staleness canary:
                       re-measured at the start of every run and compared, and
                       if it has moved beyond tolerance these constants no
                       longer describe this machine (different cgroup quota,
                       changed governor, a co-tenant, thermal state) and the
                       predictor abstains. It is the cheap one, at tens of
                       nanoseconds per call, which is why it and not the memory
                       kernel is on the per-run path. It is the only freshness
                       check the model needs, because everything else volatile
                       is re-derived per run rather than cached.
- `fingerprint`        machine identity these constants were measured on.
- `schema_version`     bumped when term semantics change, so stale files are
                       rejected rather than reinterpreted.
"""
Base.@kwdef struct MachineConstants
    simd_lane::RateCurve
    coeff_touch::RateCurve
    parallel_speedup::RateCurve
    ns_per_scalar_item::Float64
    ns_per_queue_node::Float64
    dispatch_pool_ns_base::Float64
    dispatch_pool_ns_per_worker::Float64
    dispatch_batch_ns_base::Float64
    dispatch_batch_ns_per_worker::Float64
    ns_per_atomic::Float64
    reference_fma_ns::Float64
    reference_mem_ns::Float64
    usl_alpha_base::Float64 = 0.0
    usl_beta_arith::Float64 = 0.0
    usl_beta_alloc::Float64 = 0.0
    usl_beta_bw::Float64 = 0.0
    llc_bytes::Float64 = 0.0
    fingerprint::String = ""
    schema_version::Int = 1
    # `Threads.nthreads()` of the session that measured the speedup ladders
    # (`parallel_speedup`, the USL fit, the dispatch widths); 0 when unknown
    # (a file written before this was recorded).
    measured_threads::Int = 0
end

"""
    usl_speedup(α, β, k) -> Float64

Universal Scalability Law: `S(k) = k / (1 + α(k−1) + βk(k−1))`.

Two parameters rather than one because the measured curves do two different
things. `α` alone (Amdahl) can only saturate, and the workloads that matter here
do not saturate -- they **invert**: the aero RHS at N=1024 runs 4957 µs serial,
1637 at four threads and 4755 at twelve. Only a term that grows faster than the
worker count can bend a speedup curve back down, which is what `βk(k−1)` does,
and it puts the turning point at `k* = √((1−α)/β)` in closed form rather than
requiring that the inverting width have been measured.

Returns at least `1.0`: a plan can always decline to split, so a predicted
speedup below one means "do not widen", not "widening makes the work itself
slower".
"""
@inline function usl_speedup(alpha::Real, beta::Real, k::Integer)::Float64
    kf = Float64(max(1, k))
    kf <= 1.0 && return 1.0
    a = max(0.0, Float64(alpha))
    b = max(0.0, Float64(beta))
    denom = 1.0 + a * (kf - 1.0) + b * kf * (kf - 1.0)
    denom > 0.0 || return 1.0
    return max(1.0, kf / denom)
end

"""
    usl_peak_workers(α, β) -> Float64

The width at which [`usl_speedup`](@ref) turns over, `k* = √((1−α)/β)`. `Inf`
when `β` is zero, which is the saturating case: more workers never help, but
they never hurt either.
"""
@inline function usl_peak_workers(alpha::Real, beta::Real)::Float64
    b = max(0.0, Float64(beta))
    b > 0.0 || return Inf
    a = clamp(Float64(alpha), 0.0, 1.0)
    return sqrt(max(0.0, 1.0 - a) / b)
end
