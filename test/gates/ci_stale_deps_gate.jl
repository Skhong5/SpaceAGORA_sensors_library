# Every entry in Project.toml [deps] must be loaded by the package itself.
# Aqua's stale-dependency check catches the class of leftover that #92 and this
# gate's own PR removed by hand (Rotations, ControlSystemsBase, ..., Quaternions):
# packages that stay in the dependency list after their last use is deleted.
#
# The ignore list is deliberate and short. Aqua and Plots serve the test
# harness and the examples; Roots serves the examples' apsis searches and the
# typed EDG repair test; SatelliteToolboxGravityModels is the independent
# reference the harmonics parity test in suite 04 checks the in-house
# spherical-harmonics model against. None of them is used by the package, and
# they live in [deps] only because the repository has no separate test project
# yet. Do not add package code to this list; either use the dependency in src/
# or remove it.
using Aqua
using SpaceAGORA

Aqua.test_stale_deps(SpaceAGORA; ignore=[:Aqua, :Plots, :Roots, :SatelliteToolboxGravityModels])
println("stale_deps_gate_ok")
