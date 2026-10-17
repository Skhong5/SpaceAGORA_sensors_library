"""A research project on SpaceAGORA's public API, as the contributor workflow tests it.

Core CI runs it through `.github/workflows/contributor-ci.yml` with this checkout as the
core. Every SpaceAGORA name here is a root export.
"""
module ContributorProject

using SpaceAGORA

"""A short no-GRAM Earth orbit: no native GRAM, SPICE kernels or private data."""
function case_configuration(; mission_time_s=30.0)
    planet = make_no_gram_planet(:earth)
    root = Link(root=true, m=100.0, ref_area=2.0)
    initial = InitialCondition(ra=planet.Rp_e + 700e3, rp=planet.Rp_e + 650e3, i=45.0, ω=0.0, Ω=0.0, ν=10.0)
    spacecraft = SpacecraftModel(joints=Joint[], links=[root], root=root, prop_mass=0.0,
        inertia_tensor=root.inertia, initial_condition=initial, id=1)
    return SimulationConfiguration(
        simulation_settings=SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false),
        mission_configuration=MissionConfiguration(mission_type=MissionTime, keplerian=true,
            number_of_orbits=1, mission_time=mission_time_s, orientation_sim=false, num_steps_to_save=20),
        environment_model=make_no_gram_environment(planet=planet),
        dynamics_model=DynamicsModel(SpacecraftModel[spacecraft], (InverseSquaredJ2GravityModel(),)),
        guidance_model=GuidanceModel((), Float64[]),
        navigation_model=NavigationModel((), Float64[]),
        control_model=ControlModel((), Float64[]),
        initial_time=InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
end

run_case(; kwargs...) = run_simulation(case_configuration(; kwargs...); return_solution=true)

end
