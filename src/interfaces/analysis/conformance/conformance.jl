"""
    SpaceAGORA.Conformance

Checks that code built around SpaceAGORA (a force/torque model, a service facade)
behaves as the engine's contracts require. Each `check_*` function runs the engine's
own machinery (its pre-run hook check and `run_simulation`) on the object and returns
a [`ConformanceReport`](@ref); it needs no test framework. With the `Test` standard
library loaded, each `test_*` function records the same checks as tests:

```julia
using Test, SpaceAGORA
using SpaceAGORA.Conformance

@testset "MyModels" begin
    test_force_torque_model(MyDisturbanceTorque(gain=1e-6))
end
```

A conformance pass certifies interface behaviour only. Scientific validation of a
model, its parameters and the claims made from it stay with its author.
"""
module Conformance

import Logging
import Random
import SciMLBase
import TOML
import ..SimulationModel
import ..SimulationEngine

export ConformanceCheck, ConformanceReport, ConformanceFixture, passed, conformance_fixture
export check_force_torque_model, check_service_facade, check_sensor
export test_report, test_force_torque_model, test_service_facade, test_sensor

include(joinpath(@__DIR__, "reports.jl"))
include(joinpath(@__DIR__, "fixtures.jl"))
include(joinpath(@__DIR__, "walker.jl"))
include(joinpath(@__DIR__, "effectors.jl"))
include(joinpath(@__DIR__, "facades.jl"))
include(joinpath(@__DIR__, "sensors.jl"))

# The check ids of each report kind, in report order. Core's negative controls
# fail when an id here has no control.
const CHECK_IDS = (
    force_torque_model = (:hooks_present, :supported_sample_signature, :repeatable_run, :deepcopy_equivalent,
        :global_rng_independent, :concurrent_equals_sequential, :output_hook_pure, :environment_declared),
    service_facade = (:facade_shape,),
    sensor = (:run_state_fields, :requirements_supported, :measurement_shape, :repeatable_run, :deepcopy_equivalent,
        :global_rng_independent, :concurrent_equals_sequential, :seed_repeatable, :custom_state_reset),
)

# Defined by ext/SpaceAGORATestExt.jl when Test is loaded. Today Test is always
# loaded with SpaceAGORA (MathOptInterface, a dependency of OSQP, loads it), so the
# hint below matters only if that changes.
const _TEST_FUNCTIONS = (test_report, test_force_torque_model, test_service_facade, test_sensor)

function _test_extension_hint(io::IO, exc::MethodError, extension_loaded::Bool)
    extension_loaded && return nothing
    any(f -> exc.f === f, _TEST_FUNCTIONS) || return nothing
    print(io, "\nThe test_* conformance functions come from SpaceAGORA's Test extension: ",
        "load the Test standard library first (`using Test`). Without it, the matching ",
        "check_* function returns the same checks as a ConformanceReport.")
    return nothing
end

function __init__()
    Base.Experimental.register_error_hint(MethodError) do io, exc, argtypes, kwargs
        loaded = Base.get_extension(parentmodule(@__MODULE__), :SpaceAGORATestExt) !== nothing
        _test_extension_hint(io, exc, loaded)
    end
    return nothing
end

end # module Conformance
