# Your First Script

Use this page when you have run the [Quickstart](quickstart.md) and want to
write a scenario of your own. It starts from the same no-GRAM Earth scenario
and changes what you need.

What to read next:

- [Configuring a Simulation](simulation_configuration.md)
- [Gravity and Forces](gravity_forces.md)
- [Simulation Outputs](outputs.md)
- [Stopping on a Condition](stop_conditions.md)

## Your own script

Every type needed to set up a run is exported from the root module, so
`using SpaceAGORA` is the only import. `make_example_config` and
`make_three_body_spacecraft` build a complete configuration from a planet, a
spacecraft, and an initial condition. `make_example_config` defaults to SPICE
ephemerides, so pass `SimpleEphemeridesModel()` for a run without SPICE kernels:

```julia
using SpaceAGORA

planet = make_no_gram_planet(:earth)
spacecraft = make_three_body_spacecraft(
    bus_dims=(2.0, 2.0, 2.0), panel_dims=(0.01, 2.0, 1.0),
    bus_mass=500.0, panel_mass_each=10.0, panel_offset_y=2.0,
    ic=InitialCondition(ra=planet.Rp_e + 500e3, rp=planet.Rp_e + 500e3,
                        i=45.0, ω=0.0, Ω=0.0, ν=0.0),
)
config = make_example_config(
    planet=planet, spacecraft=spacecraft, mission_time=300.0,
    initial_time=InitialTime(year=2024, month=1, day=1),
    dynamic_effectors=(InverseSquaredGravityModel(),),
    ephemerides_model=SimpleEphemeridesModel(), results=false,
)
run_simulation(config)
```

## When to pick a different path

Choose [Verification Study](verification_study.md) instead when your goal is a
known study workflow with enforcement and report outputs.

Choose [Simulation Outputs](outputs.md) when the run completed and you need to
interpret the CSV, Feather, or manifest files.

Choose [Assets & Modes](../assets.md) instead when you need to decide whether a
machine is ready for GRAM/SPICE-backed runs.
