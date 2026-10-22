#!/usr/bin/env julia
#
# One-shot per-machine calibration of the parallel cost constants.
#
#   julia --project=. --threads=auto scripts/calibration/calibrate_machine.jl [--verbose] [--repeats=N]
#
# Writes output/parallel_policy_state/cost_constants_<fingerprint>.toml, which
# every later run on this machine reads instead of measuring anything itself.
#
# Run it with the thread count you intend to simulate at: the speedup ladders and
# the USL fit only reach the thread count of the session that measured them.
#
# --repeats runs the whole pass N times and reports each constant's spread.
#
# Read repeat spread as a diagnostic, not as an acceptance criterion: dispatch
# timing includes OS wake scheduling. The predictive campaign planner reads the
# USL terms (usl_alpha_base and usl_beta_alloc). The other constants were
# measured for the RHS-plan predictor, which 0.6.0 removed, and nothing reads
# them now.

using SpaceAGORA

const PC = SpaceAGORA.SimulationModel.ParallelCost

verbose = any(a -> a in ("--verbose", "-v"), ARGS)
function _parse_repeats(args)::Int
    for a in args
        startswith(a, "--repeats=") && return max(1, parse(Int, split(a, "=")[2]))
    end
    return 1
end
repeats = _parse_repeats(ARGS)

println("SpaceAGORA cost-model machine calibration")
println("  fingerprint : ", PC.machine_fingerprint())
println("  threads     : ", Threads.nthreads())
println("  repeats     : ", repeats)
println()

results = PC.MachineConstants[]
for r in 1:repeats
    repeats > 1 && println("--- pass $(r)/$(repeats) ---")
    push!(results, PC.calibrate_machine(; verbose = verbose))
end
mc = results[end]

if repeats > 1
    println()
    println("Reproducibility across $(repeats) passes (max spread, % of min).")
    println("Diagnostics, not an acceptance gate -- see the header comment:")
    spread(v) = isempty(v) || minimum(v) <= 0 ? NaN :
        (maximum(v) - minimum(v)) / minimum(v) * 100
    report(name, f) = println("  ", rpad(name, 30),
        isnan(spread(map(f, results))) ? "n/a" : string(round(spread(map(f, results)), digits=2), "%"))
    report("ns_per_scalar_item", m -> m.ns_per_scalar_item)
    report("ns_per_queue_node", m -> m.ns_per_queue_node)
    report("dispatch_pool_ns_base", m -> m.dispatch_pool_ns_base)
    report("dispatch_pool_ns_per_worker", m -> m.dispatch_pool_ns_per_worker)
    report("dispatch_batch_ns_base", m -> m.dispatch_batch_ns_base)
    report("dispatch_batch_ns_per_worker", m -> m.dispatch_batch_ns_per_worker)
    report("ns_per_atomic", m -> m.ns_per_atomic)
    report("reference_fma_ns", m -> m.reference_fma_ns)
    report("reference_mem_ns", m -> m.reference_mem_ns)
    for i in eachindex(mc.coeff_touch.ns)
        report("coeff_touch[$(i)]", m -> m.coeff_touch.ns[i])
    end
    for i in eachindex(mc.simd_lane.ns)
        report("simd_lane[$(i)]", m -> m.simd_lane.ns[i])
    end
end

path = PC.save_machine_constants(mc)
println()
println("Wrote ", path)
