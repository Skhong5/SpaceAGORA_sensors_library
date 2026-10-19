using Test
using SpaceAGORA
using SpaceAGORA.SimulationModel
using StaticArrays

# `probe_effector_costs!` measures each dynamic effector in isolation against
# live state. A `wrench` effector needs a StateSample, and the probe used to
# read it from the flat route's per-satellite buffers. Only the flat route
# fills those, so on every other route each `wrench` effector was left
# unmeasured (NaN). A user-defined effector is not on the flat route's
# thread-safe list, so it never takes that route and was never measured.
#
# The probe now builds the sample from the state, as the serial,
# satellite-batch and per-satellite routes do. On the flat route that is the
# sample the flat route itself builds, so the effector is timed with the same
# arguments as before.

const SE = SpaceAGORA.SimulationEngine
const PC = SpaceAGORA.SimulationModel.ParallelCost

# The sample of the last `wrench` call, so a test can check which satellite's
# state the probe handed over.
const _ROUTE_PROBE_LAST_SAMPLE = Ref{Any}(nothing)

# A user-defined effector that implements only `wrench`: real work proportional
# to `iterations`, zero force.
struct _RouteProbeWrenchEffector <: SimulationModel.AbstractForceTorqueModel
    iterations::Int
end

function SimulationModel.wrench(
    model::_RouteProbeWrenchEffector,
    x::SimulationModel.StateSample,
    ::SimulationModel.EnvironmentSample,
    ::Float64,
)::Tuple{SVector{3, Float64}, SVector{3, Float64}}
    _ROUTE_PROBE_LAST_SAMPLE[] = x
    acc = 0.0
    @inbounds for _ in 1:model.iterations
        acc = muladd(x.pos_ii[1], 1.0000001, acc)
    end
    return (SVector{3, Float64}(acc * 0.0, 0.0, 0.0), SVector{3, Float64}(0.0, 0.0, 0.0))
end

function _route_probe_spacecraft(planet, id::Int)
    root = Link(root=true, m=500.0, ref_area=12.0)
    ic = InitialCondition(
        ra = planet.Rp_e + 540e3 + 2e3 * (id - 1),
        rp = planet.Rp_e + 500e3 + 1e3 * (id - 1),
        i = 35.0, ω = 40.0, Ω = 10.0, ν = 120.0 + 12.0 * (id - 1),
    )
    return SpacecraftModel(Joint[], Link[root], root, true, 500.0, 0.0, root.inertia, 0, 0, ic, id)
end

function _route_probe_config(n_sats::Int, dynamic_effectors::Tuple; orientation_sim::Bool=false)
    planet = Earth()
    return SimulationConfiguration(
        simulation_settings=SimulationSettings(
            results=false, verbose=false, generate_plots=false, normalize=false, save_csv=false
        ),
        mission_configuration=MissionConfiguration(
            mission_type=MissionTime, keplerian=true, number_of_orbits=1,
            mission_time=600.0, orientation_sim=orientation_sim, num_steps_to_save=10, data_rate=10.0
        ),
        environment_model=EnvironmentModel(
            planet=planet, EI=120.0,
            density_model=ExponentialAtmosphereModel(planet),
            # No SPICE kernels: a fresh checkout has none, and nothing here
            # needs an ephemeris.
            ephemerides_model=SimpleEphemeridesModel(),
            thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
            topography=false, wind=false
        ),
        dynamics_model=DynamicsModel(
            [_route_probe_spacecraft(planet, i) for i in 1:n_sats],
            dynamic_effectors,
        ),
        guidance_model=GuidanceModel(guidance_effectors=(), guidance_rates=Float64[]),
        navigation_model=NavigationModel(navigation_effectors=(), navigation_rates=Float64[]),
        control_model=ControlModel(control_effectors=(), control_rates=Float64[]),
        initial_time=InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        integration_tolerances=IntegrationTolerances(
            reltol_orbit=1e-9, abstol_orbit=1e-9, dt_max_orbit=20.0
        ),
    )
end

function _route_probe_params(args, n_sats::Int)
    p = SE.ODEParams(n_sats=n_sats, args=args)
    SE._initialize_heat_rate_buffers!(p)
    SE._initialize_harmonics_workspace_buffers!(p)
    SE._initialize_density_model_instances!(p)
    SE._initialize_density_cache_buffers!(p)
    SE._initialize_gram_isolated_pool_buffers!(p)
    SE._initialize_aero_workspace_buffers!(p)
    SE._initialize_nbody_workspace_buffers!(p)
    return p
end

# Pins the route the warm-up evaluation takes, as a run's setup snapshot does.
function _route_probe_pin!(p, mode::String)
    p.shared_buffers.rhs_env_config[] = withenv(
        "SPACEAGORA_RHS_EXECUTION_MODE" => mode,
        "SPACEAGORA_PARALLEL_PROFILE" => nothing,
    ) do
        SE._snapshot_rhs_plan_env_config()
    end
    return p
end

@testset "Effector cost probe measures wrench effectors on every route" begin
    floor_ns = PC.timing_noise_floor()
    routes = (
        ("serial", :serial),
        ("satellite", :satellite_batch),
        ("per_satellite", :per_satellite_effector_reduce),
    )
    for (mode, expected_route) in routes
        effectors = (InverseSquaredJ2GravityModel(), _RouteProbeWrenchEffector(4096))
        args = _route_probe_config(3, effectors)
        p = _route_probe_pin!(_route_probe_params(args, 3), mode)
        @test SE._rhs_execution_plan(args, p, args.dynamics_model.dynamic_effectors, 3).mode === expected_route
        u = SE.build_initial_conditions(args)
        # Probe satellite 2, so a sample taken from another satellite shows.
        p.is_active[1] = false
        _ROUTE_PROBE_LAST_SAMPLE[] = nothing

        probes = SE.probe_effector_costs!(p, u, args)

        # The warm-up took a route other than the flat one, so the flat
        # route's per-satellite buffers were never filled.
        @test isempty(p.shared_buffers.rhs_flat_state_pos_ii[])
        @test length(probes) == 2
        for idx in 1:2
            @test isfinite(probes[idx].ns_per_sat)
            @test isfinite(probes[idx].reference_ratio)
        end
        # The user-defined effector declares no cost terms, so the
        # measurement is its only cost, and it is real work.
        @test !probes[2].declared
        @test probes[2].ns_per_sat > floor_ns
        @test haskey(PC.probe_ns_map(probes), 2)

        # The probe handed over the probed satellite's own state.
        sample = _ROUTE_PROBE_LAST_SAMPLE[]
        @test sample isa SimulationModel.StateSample
        @test sample.spacecraft_index == 2
        @test sample.spacecraft === args.dynamics_model.spacecraft[2]
        sc2 = u.sc[2]
        @test sample.pos_ii == SVector{3, Float64}(sc2[1], sc2[2], sc2[3])
        @test sample.vel_ii == SVector{3, Float64}(sc2[4], sc2[5], sc2[6])
        @test sample.mass_kg == Float64(sc2[7])
    end
end

@testset "Effector cost probe on the flat route times the flat route's sample" begin
    for orientation_sim in (false, true)
        effectors = (InverseSquaredJ2GravityModel(), _RouteProbeWrenchEffector(256))
        args = _route_probe_config(3, effectors; orientation_sim=orientation_sim)
        p = _route_probe_params(args, 3)
        u = SE.build_initial_conditions(args)
        # Run the flat route itself at width 1, so it runs on any thread
        # count; it fills its per-satellite sample buffers from `u`.
        plan = (
            mode=:flat_constellation_effector_queue, allotment=1, scheduler=:static,
            dominant_axis=:effector, policy_applied=false,
            effector_decision=(use_threads=false, allotment=1, mode=:off, policy_applied=false),
        )
        SE._spacecraft_dynamics_flat_constellation_effector_queue!(zero(u), u, p, 0.0, plan; rhs_kind=:full)
        @test length(p.shared_buffers.rhs_flat_state_pos_ii[]) >= 3
        flat_sample(sat_idx) = SE._rhs_flat_state_sample_from_buffers(
            p.shared_buffers, args.dynamics_model.spacecraft, sat_idx, orientation_sim
        )

        p.is_active[1] = false
        _ROUTE_PROBE_LAST_SAMPLE[] = nothing
        probes = SE.probe_effector_costs!(p, u, args)
        @test all(isfinite(probes[idx].ns_per_sat) for idx in 1:2)

        # Field for field the sample the flat route evaluates satellite 2 with.
        sample = _ROUTE_PROBE_LAST_SAMPLE[]
        expected = flat_sample(2)
        @test typeof(sample) === typeof(expected)
        for field in fieldnames(SimulationModel.StateSample)
            @test getfield(sample, field) === getfield(expected, field)
        end
        @test (sample.q_ib === nothing) == !orientation_sim
    end
end
