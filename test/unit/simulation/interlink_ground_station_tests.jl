module InterlinkGroundStationTests

# Ground stations: the geodetic position, a station's state sample, the
# elevation mask, the station and its refusals, registration, pairs, the
# policies at a station end, a pass against the closed-form elevation, the
# planet's frame inside a step, an uplink, the setup checks, the viewer block,
# checkpoints, and a negative control.

using Test
using LinearAlgebra
using StaticArrays
using SpaceAGORA

const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const IM = SM.InterLinkModels
const PLANET = make_no_gram_planet(:earth)
const RADIUS = PLANET.Rp_e + 700e3
const BUDGET = ShannonBudget(bandwidth_hz=1e6, frequency_hz=2.2e9)
const LINK = DataLink(transmit_power_w=1.0, budget=BUDGET, range_m=3000e3)

radio(; kwargs...) = RadioTerminal(; boresight=[0.0, 0.0, 1.0], max_power_w=5.0, gain_dbi=0.0, efficiency=0.5, kwargs...)
station(; name="north", lat_deg=0.0, lon_deg=0.0, alt_m=0.0, min_elevation_deg=10.0, antennas=[radio()]) =
    GroundStation(; name, lat_deg, lon_deg, alt_m, min_elevation_deg, antennas)

# A 100 kg bus at `pos` with velocity `vel`, with a radio and, when asked, a battery.
function radio_sat(index; pos=SVector(RADIUS, 0.0, 0.0), vel=SVector(0.0, sqrt(PLANET.μ / RADIUS), 0.0),
                   terminal=radio(), n_terminal=0, battery=false)
    bus = Link(root=true, m=100.0)
    ic = CartesianInitialCondition(SVector{3, Float64}(pos), SVector{3, Float64}(vel); q=SVector(0.0, 0.0, 0.0, 1.0))
    sc = SpacecraftModel(root=bus, links=[bus], inertia_tensor=bus.inertia, initial_condition=ic, id=index,
        n_terminal=n_terminal)
    terminal === nothing || add_actuator!(sc, bus, terminal)
    battery && add_power!(sc, bus, Battery(capacity_wh=10.0, initial_soc=0.5))
    return sc
end

# A planet whose frame (J2000 to planet-fixed) is a turn of `θ` about the pole.
function turned_planet(θ)
    planet = make_no_gram_planet(:earth)
    planet.L_PI .= [cos(θ) sin(θ) 0.0; -sin(θ) cos(θ) 0.0; 0.0 0.0 1.0]
    return planet
end

# The inertial position `range_m` from station `st` at elevation `el` and azimuth
# `az` (rad, from north toward east), under the planet's frame.
function seen_from(st, planet, el, az, range_m)
    lat, lon = deg2rad(st.lat_deg), deg2rad(st.lon_deg)
    up = SVector(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
    east = SVector(-sin(lon), cos(lon), 0.0)
    north = cross(up, east)
    position = SM.Geodesy.geodetic_to_pp(lat, lon, st.alt_m, planet)
    direction = cos(el) * (cos(az) * north + sin(az) * east) + sin(el) * up
    return transpose(SMatrix{3, 3, Float64}(planet.L_PI)) * (position + range_m * direction)
end

state_at(positions...) = (sc=[(pos=p, vel=SVector(0.0, 7.5e3, 0.0), mass=100.0) for p in positions],)

# The closed-form Shannon rate of `BUDGET` at `d` between two 0 dBi radios at 500 K.
function shannon(d; power_w=1.0)
    λ = 299_792_458.0 / BUDGET.frequency_hz
    received = power_w * (λ / (4π * d))^2
    noise = 1.380649e-23 * 500.0 * BUDGET.bandwidth_hz
    return BUDGET.implementation_efficiency * BUDGET.bandwidth_hz * log2(1 + received / noise)
end

function simpson(f, a, b, n)
    h = (b - a) / n
    total = f(a) + f(b)
    for k in 1:n-1
        total += (isodd(k) ? 4 : 2) * f(a + k * h)
    end
    return total * h / 3
end

function run_args(spacecraft, model; duration=60.0, dt_max=10.0, policy=DataThroughputPolicy(),
                  settings=SimulationSettings(results=false, generate_plots=false))
    return SimulationConfiguration(
        simulation_settings=settings,
        mission_configuration=MissionConfiguration(mission_type=MissionTime, mission_time=duration,
            orientation_sim=false, data_rate=1.0),
        environment_model=make_no_gram_environment(planet=PLANET),
        dynamics_model=DynamicsModel(spacecraft, (InverseSquaredGravityModel(),)),
        guidance_model=GuidanceModel((), Float64[]), navigation_model=NavigationModel((), Float64[]),
        control_model=ControlModel((), Float64[]), initial_time=InitialTime(),
        integration_tolerances=IntegrationTolerances(reltol_orbit=1e-10, abstol_orbit=1e-8, dt_max_orbit=dt_max),
        solver_config=SM.SolverConfig(solver_mode=:tsit5), interlink_model=model,
        scheduling_policy_model=policy)
end

run(spacecraft, model; kwargs...) = run_simulation(run_args(spacecraft, model; kwargs...);
    return_results=true, return_solution=true)
refusal(f) = try f(); nothing catch err err end

# The planet's frame at `t` s into a run, as the ephemerides give it.
const EPHEMERIDES = make_no_gram_environment(planet=PLANET).ephemerides_model
const ET0 = SM.EphemeridesModels.ephemerides_epoch_seconds(EPHEMERIDES, InitialTime())
frame(t) = SMatrix{3, 3, Float64}(SM.EphemeridesModels.ephemerides_frame_lpi(EPHEMERIDES, PLANET, ET0 + t))

# An equatorial station at longitude 0 and one spacecraft on a circular
# equatorial orbit, starting `lead_deg` west of it, with the link `link` (a
# downlink by default; `uplink=true` reverses it).
function pass_fixture(; lead_deg=25.0, st=station(), link=LINK, uplink=false, battery=false)
    L0 = frame(0.0)
    u = transpose(L0) * normalize(SM.Geodesy.geodetic_to_pp(0.0, 0.0, 0.0, PLANET))
    pole = transpose(L0) * SVector(0.0, 0.0, 1.0)
    φ = -deg2rad(lead_deg)
    r̂ = cos(φ) * u + sin(φ) * cross(pole, u)
    spacecraft = [radio_sat(1; pos=RADIUS * r̂, vel=sqrt(PLANET.μ / RADIUS) * cross(pole, r̂), battery)]
    model = InterlinkModel()
    add_ground_station!(model, st)
    uplink ? register_link!(model, spacecraft, (-1, 1), (1, 1), link) : register_link!(model, spacecraft, (1, 1), (-1, 1), link)
    return spacecraft, model
end

# The elevation of spacecraft 1 above the station of `pass_fixture` at `t`, from the solution.
const SITE = SM.Geodesy.geodetic_to_pp(0.0, 0.0, 0.0, PLANET)
pass_elevation(solution, t) = (d = frame(t) * SVector{3, Float64}(solution(t).sc[1].pos) - SITE; asin(d[1] / norm(d)))

# The data the link `key` of a `pass_fixture` run carried, by quadrature: from each
# history sample that selects it to the next one, at the closed-form rate between
# the spacecraft of the solution and the station under the planet's frame at each
# time, or, with `held`, under the frame at each step's start.
function carried(solution, key; held::Bool=false)
    history = solution.prob.p.args.interlink_model.history
    rate(t, L) = shannon(norm(SVector{3, Float64}(solution(t).sc[1].pos) - transpose(L) * SITE))
    total = 0.0
    for k in 1:length(history) - 1
        key in history[k].active || continue
        t0, t1 = history[k].time, history[k + 1].time
        total += simpson(t -> rate(t, held ? frame(t0) : frame(t)), t0, t1, 64)
    end
    return total
end

@testset "Ground stations" begin
    @testset "geodetic position" begin
        for planet in (PLANET, make_no_gram_planet(:mars))
            @test SM.Geodesy.geodetic_to_pp(0.0, 0.0, 100.0, planet) ≈ [planet.Rp_e + 100.0, 0.0, 0.0] rtol=1e-15
            @test SM.Geodesy.geodetic_to_pp(0.0, π / 2, 0.0, planet) ≈ [0.0, planet.Rp_e, 0.0] atol=1e-6
            @test SM.Geodesy.geodetic_to_pp(π / 2, 0.0, 50.0, planet) ≈ [0.0, 0.0, planet.Rp_p + 50.0] atol=1e-6
            # The round trip through `rtolatlong`.
            for (lat, lon, alt) in ((42.3, -83.7, 270.0), (-60.0, 150.0, 3000.0), (89.0, 10.0, 0.0), (0.5, 179.0, -50.0))
                back = SM.FrameTransforms.rtolatlong(SM.Geodesy.geodetic_to_pp(deg2rad(lat), deg2rad(lon), alt, planet), planet)
                @test back[1] ≈ alt atol=1e-5
                @test back[2] ≈ deg2rad(lat) atol=1e-11
                @test back[3] ≈ deg2rad(lon) atol=1e-11
            end
        end
    end

    @testset "a station's state sample" begin
        planet = turned_planet(0.6)
        L = SMatrix{3, 3, Float64}(planet.L_PI)
        st = station(lon_deg=30.0)
        sample = IM._station_sample(st, 2, planet, nothing)
        site = SM.Geodesy.geodetic_to_pp(0.0, deg2rad(30.0), 0.0, planet)
        # Its inertial position is `L_PIᵀ r`, and it moves at `ω R` toward the planet-fixed east.
        @test L * sample.pos_ii ≈ site rtol=1e-14
        @test L * sample.vel_ii ≈ cross(planet.ω, site) rtol=1e-14
        @test norm(sample.vel_ii) ≈ planet.ω[3] * planet.Rp_e rtol=1e-12
        @test dot(L * sample.vel_ii, [-sin(deg2rad(30.0)), cos(deg2rad(30.0)), 0.0]) ≈ norm(sample.vel_ii) rtol=1e-12
        @test isnan(sample.mass_kg) && sample.q_ib === nothing && sample.ω_body === nothing
        @test sample.spacecraft === st && sample.spacecraft_index == -2
        # A frame given in place of the planet's current one.
        other = SMatrix{3, 3, Float64}(turned_planet(1.1).L_PI)
        @test other * IM._station_sample(st, 2, planet, other).pos_ii ≈ site rtol=1e-14
        err = refusal(() -> IM._station_sample(st, 1, nothing, nothing))
        @test err isa ArgumentError && occursin("needs the run's planet", err.msg)
    end

    @testset "the elevation mask" begin
        planet = turned_planet(0.3)
        for st in (station(lat_deg=60.0, lon_deg=-20.0, alt_m=500.0), station(lat_deg=-35.0, lon_deg=148.0))
            for (el, az) in ((0.2, 0.0), (deg2rad(10.05), 1.0), (deg2rad(9.95), 4.0), (1.4, 2.5), (-0.1, 3.0))
                # The geodetic up, not the geocentric radial: they differ by a few mrad at these latitudes.
                @test IM._station_elevation_rad(st, planet, nothing, seen_from(st, planet, el, az, 1500e3)) ≈ el atol=1e-12
            end
        end
        st = station(lat_deg=60.0, lon_deg=-20.0)
        spacecraft = [radio_sat(1)]
        model = InterlinkModel()
        add_ground_station!(model, st)
        register_link!(model, spacecraft, (1, 1), (-1, 1), LINK)
        available(el, az; range_m=1500e3, kwargs...) = only(update_availability!(model, spacecraft,
            state_at(seen_from(st, planet, el, az, range_m)); context=LinkContext(planet=planet), kwargs...))
        # Overhead and just above the mask: available, also through the planet's sphere of equatorial
        # radius, which a station at 60° latitude sits inside.
        @test available(π / 2, 0.0)
        @test available(deg2rad(10.05), 0.0) && available(deg2rad(10.05), π)
        # Just below the mask, north and south: not.
        @test !available(deg2rad(9.95), 0.0) && !available(deg2rad(9.95), π)
        # The link's own range applies too.
        @test !available(π / 3, 0.0; range_m=3100e3)
        # A station is always active; the spacecraft's own flag applies.
        @test available(π / 2, 0.0; is_active=[true]) && !available(π / 2, 0.0; is_active=[false])
        # A forbidden spacecraft-station pair.
        forbidden = InterlinkModel(forbidden_pairs=[(1, -1)])
        add_ground_station!(forbidden, st)
        register_link!(forbidden, spacecraft, (1, 1), (-1, 1), LINK)
        @test (-1, 1) in forbidden.forbidden_pairs
        @test !only(update_availability!(forbidden, spacecraft, state_at(seen_from(st, planet, π / 2, 0.0, 1500e3));
            context=LinkContext(planet=planet)))
    end

    @testset "the station" begin
        st = station(name="x", lat_deg=10, lon_deg=-200.0, alt_m=-30.0, min_elevation_deg=0.0)
        @test st.name == "x" && st.lat_deg === 10.0 && st.lon_deg == -200.0 && st.alt_m == -30.0 && st.min_elevation_deg == 0.0
        defaults = GroundStation(name="d", lat_deg=1.0, lon_deg=2.0, antennas=[radio()])
        @test defaults.alt_m == 0.0 && defaults.min_elevation_deg == 10.0
        for bad in ((name="",), (lat_deg=90.5,), (lat_deg=NaN,), (lon_deg=Inf,), (alt_m=NaN,), (min_elevation_deg=-1.0,),
                    (min_elevation_deg=90.0,), (antennas=RadioTerminal[],),
                    (antennas=[LaserTerminal(boresight=[1.0, 0.0, 0.0], max_power_w=1.0)],))
            @test_throws ArgumentError station(; bad...)
        end
        # The antennas are the station's own copy.
        list = [radio()]
        copied = station(antennas=list)
        push!(list, radio())
        @test length(copied.antennas) == 1
    end

    @testset "registration" begin
        spacecraft = [radio_sat(1), radio_sat(2), radio_sat(3; terminal=nothing, n_terminal=1)]
        model = InterlinkModel()
        # Two antennas with equal fields cannot be told apart, so these differ.
        dish = radio()
        @test add_ground_station!(model, station(name="a", antennas=[dish, radio(gain_dbi=3.0)])) == 1
        @test add_ground_station!(model, station(name="b")) == 2
        err = refusal(() -> add_ground_station!(model, station(name="a")))
        @test err isa ArgumentError && occursin("already has a ground station named \"a\"", err.msg)
        # A downlink keeps its order, an uplink takes the antenna itself.
        @test register_link!(model, spacecraft, (1, 1), (-1, 1), LINK) == ((1, 1), (-1, 1))
        @test register_link!(model, spacecraft, (-1, dish), (2, 1), LINK) == ((-1, 1), (2, 1))
        @test register_link!(model, spacecraft, (2, 1), (-2, 1), LINK) == ((2, 1), (-2, 1))
        @test model.graph.satellite_edges == [[1], [2, 3], Int[]]
        @test model.graph.vertices == [(1, 1), (-1, 1), (2, 1), (-2, 1)]
        for (first, second, link, needle) in (
                ((-1, 1), (-2, 1), LINK, "cannot join two ground stations"),
                ((1, 1), (-3, 1), LINK, "Invalid ground station 3"),
                ((1, 1), (-2, 2), LINK, "Invalid antenna 2 on ground station 2 (b)"),
                ((1, 1), (-1, radio(gain_dbi=7.0)), LINK, "not an antenna of ground station a"),
                ((3, 1), (-1, 2), LaserMomentumLink(), "Antenna 2 of ground station 1 (a), a RadioTerminal, cannot carry"),
                ((1, 1), (-1, 1), LINK, "already registered"))
            err = refusal(() -> register_link!(model, spacecraft, first, second, link))
            @test err isa ArgumentError && occursin(needle, err.msg)
        end
        # The station's transmitter at a level beyond its antenna's limits is refused at setup.
        loud = InterlinkModel()
        add_ground_station!(loud, station())
        register_link!(loud, spacecraft, (-1, 1), (1, 1), DataLink(transmit_power_w=10.0, budget=BUDGET, range_m=3e6))
        err = refusal(() -> SM.InterlinkTerminals.prepare_terminal_endpoints!(loud, spacecraft))
        @test err isa ArgumentError && occursin("antenna 1 of ground station 1 sets it to level 10.0", err.msg)
        # A station's links take no spacecraft's terminal endpoints, and its antennas are never shed.
        @test SM.InterlinkTerminals.prepare_terminal_endpoints!(model, spacecraft)
        @test model.terminal_endpoints[1] == [(1, 1, 1)] && model.terminal_endpoints[2] == [(2, 2, 1), (3, 1, 1)]
        @test SM.InterlinkTerminals.terminal_availability(model, spacecraft) == fill(true, 4)
    end

    @testset "pairs" begin
        spacecraft = [radio_sat(1), radio_sat(2)]
        model = InterlinkModel()
        add_ground_station!(model, station(name="a"))
        add_ground_station!(model, station(name="b", antennas=[radio(), radio()]))
        @test station_pairs(spacecraft, model) == [(1, -1), (1, -2), (2, -1), (2, -2)]
        @test station_pairs(spacecraft, model.ground_stations) == station_pairs(spacecraft, model)
        # Each station's links take its antennas in turn.
        @test register_links!(model, spacecraft, station_pairs(spacecraft, model), LINK) ==
            [((1, 1), (-1, 1)), ((1, 1), (-2, 1)), ((2, 1), (-1, 1)), ((2, 1), (-2, 2))]
        @test register_links!(model, spacecraft, reverse.(station_pairs(spacecraft, model)), LINK) ==
            [((-1, 1), (1, 1)), ((-2, 1), (1, 1)), ((-1, 1), (2, 1)), ((-2, 2), (2, 1))]
        err = refusal(() -> register_links!(model, spacecraft, [(1, -3)], LINK))
        @test err isa ArgumentError && occursin("Invalid ground station 3", err.msg)
        err = refusal(() -> register_links!(model, spacecraft, [(-1, -2)], LINK))
        @test err isa ArgumentError && occursin("cannot join two ground stations", err.msg)
    end

    @testset "the policies at a station end" begin
        planet = turned_planet(0.2)
        st = station(lat_deg=20.0, lon_deg=40.0)
        spacecraft = [radio_sat(1), radio_sat(2)]
        model = InterlinkModel()
        add_ground_station!(model, st)
        first_down = register_link!(model, spacecraft, (1, 1), (-1, 1), LINK)
        second_down = register_link!(model, spacecraft, (2, 1), (-1, 1), LINK)
        state = state_at(seen_from(st, planet, 1.2, 0.0, 800e3), seen_from(st, planet, 0.5, 2.0, 1200e3))
        context = LinkContext(time_s=0.0, planet=planet)
        scheduler = SchedulingPolicyModel(DataThroughputPolicy())
        # One antenna carries one link: the nearer spacecraft, with the higher rate, wins it.
        @test schedule_interlinks!(model, scheduler, spacecraft, state, planet.μ; context) == [first_down]
        @test scheduler.matching.score ≈ [shannon(800e3), shannon(1200e3)] rtol=1e-9
        # The candidate's station end: the station's sample and its antenna, with or without the spacecraft models.
        for models in (spacecraft, nothing)
            candidate = SM.SchedulingPolicies._link_candidate(model, 1,
                SchedulingContext(planet.μ, context, models, state, nothing))
            @test candidate.key == first_down && candidate.b.spacecraft === st && candidate.b.spacecraft_index == -1
            @test candidate.terminal_b === st.antennas[1]
            @test (candidate.terminal_a === nothing) == (models === nothing)
        end
        # The data the selected link carries counts at its spacecraft end (the run's hardware endpoints
        # give the spacecraft's radio).
        SM.InterlinkTerminals.prepare_terminal_endpoints!(model, spacecraft)
        sent, received = SM.data_link_rates_bps(model, spacecraft, state; context)
        @test sent ≈ [shannon(800e3), 0.0] rtol=1e-9
        @test received == [0.0, 0.0]
        # A station is never an orbital target and has no battery: these score a data link 0, without reading it.
        ctx = SchedulingContext(planet.μ, context, spacecraft, state, nothing)
        @test isnan(state_of_charge(ctx, -1))
        for policy in (SemimajorAxisRatePolicy(targets=:both), SemimajorAxisRatePolicy(targets=:lower_index),
                       SemimajorAxisRatePolicy(targets=[1, 2]), EccentricityRatePolicy(targets=:both),
                       EnergyBalancingPolicy(), PowerDeliveryPolicy())
            scored = SchedulingPolicyModel(policy)
            score_candidates!(model, scored, state, planet.μ; spacecraft, context)
            @test scored.matching.score == [0.0, 0.0]
        end
        mix = SchedulingPolicyModel(WeightedPolicy(PolicyTerm(DataThroughputPolicy(); scale=1e3),
            PolicyTerm(SemimajorAxisRatePolicy(targets=:both); scale=1.0)))
        score_candidates!(model, mix, state, planet.μ; spacecraft, context)
        @test mix.matching.score ≈ [shannon(800e3), shannon(1200e3)] ./ 1e3 rtol=1e-9
        # A caller's candidate force receives a station partner as its state sample.
        partners = Any[]
        force(target, partner, link) = (push!(partners, partner); SVector(0.0, 0.0, 0.0))
        score_candidates!(model, SchedulingPolicyModel(SemimajorAxisRatePolicy(targets=:lower_index)), state, planet.μ;
            spacecraft, context, candidate_force=force)
        @test length(partners) == 2 && all(partner -> partner.spacecraft === st, partners)
    end

    @testset "a pass" begin
        spacecraft, model = pass_fixture()
        solution = run(spacecraft, model; duration=900.0, dt_max=30.0).solution
        history = solution.prob.p.args.interlink_model.history
        key = ((1, 1), (-1, 1))
        mask = deg2rad(10.0)
        # Available at exactly the accepted steps where the closed-form elevation clears the mask.
        elevations = [pass_elevation(solution, sample.time) for sample in history]
        flags = [key in sample.available for sample in history]
        clear = abs.(elevations .- mask) .> 1e-7
        @test flags[clear] == (elevations .>= mask)[clear]
        @test !flags[1] && any(flags) && !flags[end]
        window = only(contact_windows(history))
        rise, set = findfirst(flags), findlast(flags)
        @test window.key == key && window.start_s == history[rise].time && window.end_s == history[set + 1].time
        @test 100.0 < window.start_s < 170.0 && 720.0 < window.end_s < 790.0
        # The throughput policy selects it for the whole pass.
        @test only(contact_windows(history; active=true)).start_s == window.start_s
        # The link schedule shows the station end as -1.
        table = link_schedule(solution.prob.p.args.interlink_model)
        @test all(==(-1), table.b_spacecraft) && all(==(1), table.b_terminal) && all(==(1), table.a_spacecraft)
        # The downlink counts what it sent at the spacecraft end.
        @test solution.u[end].sc[1].data_sent_bits > 0 && solution.u[end].sc[1].data_received_bits == 0
    end

    @testset "the planet's frame inside a step" begin
        # The rising half of the pass, before culmination: the station moves away from the spacecraft.
        spacecraft, model = pass_fixture()
        solution = run(spacecraft, model; duration=430.0, dt_max=30.0).solution
        exact, held = carried(solution, ((1, 1), (-1, 1))), carried(solution, ((1, 1), (-1, 1)); held=true)
        @test solution.u[end].sc[1].data_sent_bits ≈ exact rtol=1e-7
        # A frame held at each step's start, the negative control, misses by far more than that.
        @test abs(held - exact) > 1e-4 * exact
        # The engine reads the frame at the evaluation's time.
        p = solution.prob.p
        @test SM.SimulationCallbacks._station_planet_frame(p, 123.4) ≈ frame(123.4) rtol=1e-9
        @test SM.SimulationCallbacks._station_planet_frame(p, NaN) === nothing
    end

    @testset "an uplink" begin
        spacecraft, model = pass_fixture(uplink=true, battery=true)
        solution = run(spacecraft, model; duration=430.0, dt_max=30.0).solution
        final = solution.u[end].sc[1]
        # The spacecraft receives what the station sends, at the closed-form rate.
        @test final.data_received_bits > 0 && final.data_sent_bits == 0
        @test final.data_received_bits ≈ carried(solution, ((-1, 1), (1, 1))) rtol=1e-7
        # A receiving radio draws nothing, and a data link delivers no power.
        @test final.battery_wh == 5.0
    end

    @testset "setup checks" begin
        spacecraft, model = pass_fixture()
        empty!(model.ground_stations)
        err = refusal(() -> run_simulation(run_args(spacecraft, model)))
        @test err isa ArgumentError && occursin("references ground station 1", err.msg)
    end

    @testset "the viewer block" begin
        spacecraft = [radio_sat(1), radio_sat(2)]
        model = InterlinkModel()
        add_ground_station!(model, station())
        between = register_link!(model, spacecraft, (1, 1), (2, 1), LINK)
        down = register_link!(model, spacecraft, (2, 1), (-1, 1), LINK)
        push!(model.history, InterLinkHistory(0.0, [between, down], [between, down]))
        push!(model.history, InterLinkHistory(10.0, [between], [between]))
        table = link_schedule(model)
        @test sort(unique(table.b_spacecraft)) == [-1, 2]
        # The viewer does not draw stations yet: their links are left out.
        block = SM.SceneVisualization.interlink_payload(table, 2)
        @test length(block["links"]) == 1 && only(block["links"])["b"] == 2
        @test block["active"]["count"] == 1 && block["contact"]["count"] == 1
    end

    @testset "checkpoints" begin
        configuration = SE._interlink_checkpoint_configuration(run_args(pass_fixture()...))
        entry = only(configuration.ground_stations)
        @test keys(entry) == (:site, :antennas) && entry.site.min_elevation_deg.value == 10.0
        @test only(entry.antennas) == SE._interlink_terminal_identity(radio())
        # A changed site, mask or antenna is another configuration; a new name is not.
        for changed in (station(lat_deg=0.001), station(lon_deg=-0.001), station(alt_m=1.0), station(min_elevation_deg=10.5),
                        station(antennas=[radio(gain_dbi=1.0)]), station(antennas=[radio(), radio()]))
            @test !isequal(SE._interlink_checkpoint_configuration(run_args(pass_fixture(st=changed)...)), configuration)
        end
        @test isequal(SE._interlink_checkpoint_configuration(run_args(pass_fixture(st=station(name="renamed"))...)),
            configuration)
        # A resumed run continues; one with a changed station is refused. The spacecraft is in view
        # from the start to the end, so the selection does not depend on the steps.
        overhead() = pass_fixture(lead_deg=10.0)
        straight = run(overhead()...; duration=450.0, dt_max=30.0).solution.u[end].sc[1]
        mktempdir() do directory
            segments(resume) = SimulationSettings(results=resume, results_directory=directory, generate_plots=false,
                checkpoint_enabled=true, checkpoint_interval_s=100.0, checkpoint_directory=directory,
                resume_from_checkpoint=resume)
            run(overhead()...; duration=200.0, dt_max=30.0, settings=segments(false))
            err = refusal(() -> run(pass_fixture(lead_deg=10.0, st=station(min_elevation_deg=12.0))...; duration=450.0,
                dt_max=30.0, settings=segments(true)))
            @test err isa ArgumentError && occursin("ground stations", err.msg)
            resumed = run(overhead()...; duration=450.0, dt_max=30.0, settings=segments(true))
            @test all(sample -> ((1, 1), (-1, 1)) in sample.active, resumed.solution.prob.p.args.interlink_model.history)
            @test resumed.solution.u[end].sc[1].data_sent_bits ≈ straight.data_sent_bits rtol=1e-8
            @test resumed.table.sc1_data_sent_bits[end] ≈ straight.data_sent_bits rtol=1e-8
        end
    end

    @testset "negative control" begin
        # Without stations: none on the model, no planet frame read, no checkpoint entry.
        model = InterlinkModel()
        @test isempty(model.ground_stations) && !IM.has_ground_stations(model) && !IM.has_ground_stations(nothing)
        @test SM.SimulationCallbacks._station_planet_frame((args=(interlink_model=model,),), 1.0) === nothing
        @test SM.SimulationCallbacks._station_planet_frame((args=(interlink_model=nothing,),), 1.0) === nothing
        spacecraft = [radio_sat(1), radio_sat(2)]
        register_link!(model, spacecraft, (1, 1), (2, 1), LINK)
        @test !haskey(SE._interlink_checkpoint_configuration(run_args(spacecraft, model)), :ground_stations)
        # A link between spacecraft keeps its occultation test.
        a = IM.link_endpoint((pos=[RADIUS, 0.0, 0.0],), nothing, 1)
        b = IM.link_endpoint((pos=[-RADIUS, 1.0, 0.0],), nothing, 2)
        @test !link_available(DataLink(transmit_power_w=1.0, budget=BUDGET, range_m=2e7), a, b, LinkContext(planet=PLANET))
        # A station does not see a spacecraft on the planet's far side.
        planet = turned_planet(0.0)
        st = station()
        far = InterlinkModel()
        add_ground_station!(far, st)
        register_link!(far, spacecraft[1:1], (1, 1), (-1, 1), DataLink(transmit_power_w=1.0, budget=BUDGET, range_m=2e7))
        @test !only(update_availability!(far, spacecraft[1:1], state_at(SVector(-RADIUS, 0.0, 0.0));
            context=LinkContext(planet=planet)))
    end
end

end # module InterlinkGroundStationTests
