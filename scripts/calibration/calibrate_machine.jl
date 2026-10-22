#!/usr/bin/env julia
#
# One-shot per-machine calibration of the parallel cost constants.
#
#   julia --project=. --threads=auto scripts/calibration/calibrate_machine.jl [--verbose] [--repeats=N]
#
# Writes output/parallel_policy_state/cost_constants_<fingerprint>.toml, which
# every later run on this machine reads instead of measuring anything itself.
#
# Run it with the thread count you intend to simulate at: the USL ladders only
# reach the thread count of the session that measured them.
#
# It measures the two USL terms the predictive campaign planner reads,
# usl_alpha_base and usl_beta_alloc. --repeats runs the whole pass N times and
# reports each term's spread. Read the spread as a diagnostic, not as an
# acceptance criterion: dispatch timing includes OS wake scheduling.

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

println("SpaceAGORA machine calibration")
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
    report("usl_alpha_base", m -> m.usl_alpha_base)
    report("usl_beta_alloc", m -> m.usl_beta_alloc)
end

path = PC.save_machine_constants(mc)
println()
println("Wrote ", path)
