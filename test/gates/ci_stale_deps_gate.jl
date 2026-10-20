# Every entry in Project.toml [deps] must be loaded by the package itself.
# Aqua's stale-dependency check catches the class of leftover that #92 and this
# gate's own PR removed by hand (Rotations, ControlSystemsBase, ..., Quaternions):
# packages that stay in the dependency list after their last use is deleted.
#
# The ignore list is deliberate and short. Plots and Roots are example
# dependencies: the quickstart writes its plots with Plots, and the aerobraking
# examples find apsis times with Roots. Neither is used by the package. The
# documented examples run from the root project, so both stay in [deps] until
# 0.6.0 gives the examples an environment of their own. Do not add package code
# to this list; either use the dependency in src/ or remove it.
#
# Aqua comes from the test environment (test/Project.toml), with the test-only
# references such as SatelliteToolboxGravityModels: run this gate with
# `julia --project=test`, after `julia --startup-file=no test/setup.jl`.
Base.find_package("Aqua") === nothing &&
    error("Run this gate from the test environment: `julia --startup-file=no test/setup.jl`, then `julia --project=test test/gates/ci_stale_deps_gate.jl`.")
using Aqua
using SpaceAGORA

Aqua.test_stale_deps(SpaceAGORA; ignore=[:Plots, :Roots])
println("stale_deps_gate_ok")
