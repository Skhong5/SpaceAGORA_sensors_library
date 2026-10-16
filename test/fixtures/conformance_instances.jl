# One instance of every concrete built-in force/torque model for the conformance
# tests, with the fixture it runs on, and the types exempted with their reason.
# test/unit/conformance/builtin_conformance_tests.jl fails when a concrete built-in
# type is neither here nor exempted, so a new model is covered when it is added.
# The models that need SPICE ephemerides run on a SPICE fixture when the kernels of
# data/GRAMSuite.jl are present (as in CI) and are exempt otherwise.
module ConformanceInstances

using LinearAlgebra: I
using StaticArrays: SVector
using SpaceAGORA
using SpaceAGORA.Conformance: ConformanceFixture, conformance_fixture

const SM = SpaceAGORA.SimulationModel
const C = SpaceAGORA.Conformance

export force_torque_instances, force_torque_exemptions, spice_available

const SPICE_PATH = joinpath(pkgdir(SpaceAGORA), "data", "GRAMSuite.jl", "GRAM Suite 2.0", "SPICE")
spice_available() = isdir(SPICE_PATH)

# The built-in fixture with an exponential atmosphere, for the drag models.
function atmosphere_fixture(model)
    planet = SpaceAGORA.make_no_gram_planet(:earth)
    density = SpaceAGORA.make_no_gram_density_model(planet, :exponential)
    return ConformanceFixture(C._fixture_configuration(effectors=(model,), density_model=density))
end

# The spherical-harmonics model reads its own planet, so it gets the fixture's.
function harmonics_instance()
    base = C._fixture_configuration()
    planet = base.environment_model.planet
    file = joinpath(pkgdir(SpaceAGORA), "data", "planets", "gravity", "EarthGGM05C.csv")
    model = SpaceAGORA.GravitationalHarmonicsModel(4, 4, file, planet)
    dynamics = SM.DynamicsModel(base.dynamics_model.spacecraft, (base.dynamics_model.dynamic_effectors..., model))
    return (model=model, fixture=ConformanceFixture(SM.SimConfig._with_configuration(base; dynamics_model=dynamics)))
end

function mesh_surrogate_model()
    coefficients = reshape(sqrt(4pi) .* [-1.0, 0.2, 0.1, 0.05, 0.1, -0.07], 6, 1)
    surrogate = SM.MeshAeroSurrogate(0, 0, coefficients, zeros(6, 1), 1.0, 1.0, SVector(0.0, 0.0, 0.0), 1.0, 1.0, 3.0, 20.0)
    return SM.AerodynamicCoefficientMeshSurrogate(Dict(1 => surrogate), true)
end

function wheel_model()
    t = collect(0.0:1.0:60.0)
    speeds = hcat(400.0 .+ 2.0 .* t, -250.0 .+ 0.5 .* t, fill(120.0, length(t)))
    return SM.DynamicEffectors.ReactionWheelMomentum.ReactionWheelMomentumModel(t, speeds, Matrix{Float64}(I, 3, 3), 2.0e-5)
end

# The built-in fixture on SPICE ephemerides; `build` makes the model from the
# fixture's planet, which loads the kernels.
function on_spice(build)
    planet = SM.Earth("", SPICE_PATH)
    model = build(planet)
    fixture = ConformanceFixture(C._fixture_configuration(effectors=(model,), planet=planet,
        ephemerides_model=SM.SpiceEphemeridesModel()))
    return (model=model, fixture=fixture)
end

on_fixture(model) = (model=model, fixture=conformance_fixture(effectors=(model,)))
on_atmosphere(model) = (model=model, fixture=atmosphere_fixture(model))

"""
Fresh instances of the built-in force/torque models, each with its fixture.
"""
function force_torque_instances()
    aero = SM.DynamicEffectors.AerodynamicEffectors
    gravity = SM.DynamicEffectors.GravityEffectors
    perturbations = SM.DynamicEffectors.PerturbationEffectors
    return [
        on_atmosphere(aero.AerodynamicCoefficientConstant()),
        on_atmosphere(mesh_surrogate_model()),
        on_atmosphere(aero.AerodynamicCoefficientNoBallisticFlight()),
        on_atmosphere(SpaceAGORA.AerodynamicCoefficientfM()),
        on_atmosphere(SpaceAGORA.TelemetryVerification.ScaledAerodynamicCoefficientfM(SpaceAGORA.AerodynamicCoefficientfM(), 1.1)),
        on_fixture(gravity.ConstantGravityModel()),
        on_fixture(gravity.GravityGradientTorqueModel()),
        on_fixture(SpaceAGORA.InverseSquaredGravityModel()),
        on_fixture(SpaceAGORA.InverseSquaredJ2GravityModel()),
        harmonics_instance(),
        on_fixture(perturbations.EddyCurrentDampingModel(k_e=1.0e4)),
        on_fixture(perturbations.LVLHCascadeAttitudeControlModel(k_out=[0.1, 0.1, 0.1], w_max=0.01,
            k_rate=[1.0, 1.0, 1.0], tau_max=0.1)),
        on_fixture(perturbations.MagneticTorqueRodModel()),
        on_fixture(SpaceAGORA.PlumeSurfaceInteractionModel((; actuators=(; thrust_n=[1000.0])))),
        on_fixture(wheel_model()),
        on_fixture(SpaceAGORA.RobotArmReactionEffector()),
        (spice_available() ? spice_instances() : [])...,
    ]
end

# The N-body model declares its bodies as third bodies, so its environment check
# covers a declared third-body sample.
spice_instances() = [
    on_spice(planet -> SpaceAGORA.NBodyGravityModel(body_names=("Moon", "Sun"), primary_body_name="Earth",
        planet=planet, body_mus=(4.9028e12, 1.32712440018e20))),
    on_spice(planet -> SM.DynamicEffectors.PerturbationEffectors.SolarRadiationPressureModel(1.2, 12.0)),
    on_spice(planet -> SM.DynamicEffectors.PerturbationEffectors.FacetSolarRadiationPressureModel()),
]

"""
Concrete built-in force/torque types without an instance, each with the reason.
"""
function force_torque_exemptions()
    spice_available() && return Dict{Any, String}()
    reason = "needs SPICE ephemerides (the engine refuses it under SimpleEphemeridesModel before the run), " *
        "and the SPICE kernels of data/GRAMSuite.jl are absent here; it runs on a SPICE fixture where they are present"
    return Dict{Any, String}(T => reason for T in (SpaceAGORA.NBodyGravityModel,
        SM.DynamicEffectors.PerturbationEffectors.SolarRadiationPressureModel,
        SM.DynamicEffectors.PerturbationEffectors.FacetSolarRadiationPressureModel))
end

end # module ConformanceInstances
