# Every entry in Project.toml [deps] must be loaded by the package itself.
# Aqua's stale-dependency check catches the class of leftover that #92 and this
# gate's own PR removed by hand (Rotations, ControlSystemsBase, ..., Quaternions):
# packages that stay in the dependency list after their last use is deleted.
#
# The ignore list is deliberate and short. Plots serves the examples and a
# test; Roots serves the examples' apsis searches and the typed EDG repair
# test; SatelliteToolboxGravityModels is the independent reference the
# harmonics parity test in suite 04 checks the in-house spherical-harmonics
# model against. None of them is used by the package; they stay in [deps]
# until the examples have an environment of their own. Do not add package code
# to this list; either use the dependency in src/ or remove it.
#
# Aqua comes from the test environment (test/Project.toml): run this gate with
# `julia --project=test`, after `julia --startup-file=no test/setup.jl`.
Base.find_package("Aqua") === nothing &&
    error("Run this gate from the test environment: `julia --startup-file=no test/setup.jl`, then `julia --project=test test/gates/ci_stale_deps_gate.jl`.")
using Aqua
using SpaceAGORA

Aqua.test_stale_deps(SpaceAGORA; ignore=[:Plots, :Roots, :SatelliteToolboxGravityModels])
println("stale_deps_gate_ok")
