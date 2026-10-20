module GravityBackboneAtmosphereTests
# solver_mode=:gravity_backbone_split integrates position and velocity under
# gravity alone, and no density or thermal callback can read that state, so a
# run with a density model set is refused before it starts, however the mode is
# chosen. Without the refusal, SolverConfig(solver_mode=:gravity_backbone_split)
# failed at the first step (the density callback read `integrator.u.sc` from the
# backbone's ArrayPartition state), and SPACEAGORA_SOLVER_MODE=gravity_backbone_split
# ran with the atmosphere ignored and reported zero heating.
using Test
using Logging
using SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine

const PLANET = SM.make_no_gram_planet(:earth)
const BACKBONE = SolverConfig(solver_mode=:gravity_backbone_split)

exponential_atmosphere() = SM.ExponentialAtmosphereModel(1.0e-11, 4.0e5, 6.0e4; temperature_k=1000.0,
    valid_min_altitude_m=0.0, valid_max_altitude_m=2.0e6)

function backbone_config(density_model; solver_config=BACKBONE)
    root = SM.Link(root=true, m=500.0, ref_area=4.0)
    spacecraft = SM.SpacecraftModel(SM.Joint[], [root], root, true, 500.0, 0.0, root.inertia, 0, 0,
        SM.InitialCondition(a=PLANET.Rp_e + 4.0e5, e=0.0, i=53.0, Ω=0.0, ω=0.0, ν=0.0), 1)
    return SM.make_example_config(planet=PLANET, spacecraft=spacecraft, mission_time=60.0,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(SM.InverseSquaredJ2GravityModel(),), density_model=density_model,
        ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=false, keplerian=true,
        verbose=false, results=false, results_directory=mktempdir(), solver_config=solver_config)
end

# The atmosphere-without-drag warning is expected for these configurations.
function run_error(args; solver_mode_env)
    withenv("SPACEAGORA_SOLVER_MODE" => solver_mode_env) do
        with_logger(NullLogger()) do
            try
                run_simulation(args)
            catch err
                return err
            end
            return nothing
        end
    end
end

refuses_density(msg) = occursin("gravity_backbone_split requires density_model=NoAtmosphereModel()", msg) &&
    occursin("ExponentialAtmosphereModel", msg)

@testset "gravity backbone and a density model" begin
    @testset "the admission check names the density model" begin
        args = backbone_config(exponential_atmosphere())
        reason = SE._gravity_backbone_reject_reason(args)
        @test reason !== nothing && refuses_density(reason)
        @test !SE._gravity_backbone_eligible(args)
    end

    @testset "chosen by SolverConfig under another process default" begin
        # The callbacks are then assembled for the process default, density included.
        err = run_error(backbone_config(exponential_atmosphere()); solver_mode_env="tsit5")
        @test err isa ArgumentError && refuses_density(err.msg)
    end

    @testset "chosen by SPACEAGORA_SOLVER_MODE" begin
        args = backbone_config(exponential_atmosphere(); solver_config=nothing)
        err = run_error(args; solver_mode_env="gravity_backbone_split")
        @test err isa ArgumentError && refuses_density(err.msg)
    end

    @testset "a vacuum backbone run is still admitted" begin
        args = backbone_config(SM.NoAtmosphereModel())
        @test SE._gravity_backbone_reject_reason(args) === nothing
        sol = withenv("SPACEAGORA_SOLVER_MODE" => "tsit5") do
            run_simulation(args; return_solution=true)
        end
        @test sol.t[end] == 60.0
    end
end

end
