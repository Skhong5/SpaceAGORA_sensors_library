# Included by `Components` (src/models/vehicle/spacecraft/components.jl), which
# owns and exports this record. The actuator layer takes ownership in the
# contract step of the actuator plan.

"""
    ReactionWheelAssembly

A struct to hold the parameters and state of a
reaction wheel assembly. The wheel count is a runtime field, not a type
parameter; see the note on the struct for why.
"""
# Deliberately NOT parameterised on the wheel count. The parameter existed only
# to size J_rw/h_wheels statically, but it propagated outward as Link{N_RW},
# which made `Link` an abstract UnionAll and therefore made
# SpacecraftModel.links::Vector{Link} and .root::Link abstract fields. Every
# read of a link field in the RHS was then a dynamic lookup that boxed a
# Float64: measured at ~835 bytes allocated per satellite per RHS call, which
# accounted for essentially all of the 1.53 GB a 10000-satellite run allocated,
# and a 35.7x slowdown on field access in isolation. The wheel arrays are
# touched once per satellite per attitude step; the link fields are touched
# constantly, so the static sizing was being bought at far too high a price.
@kwdef mutable struct ReactionWheelAssembly
    # --- Parameters (Fixed) ---
    n_wheels::Int = 0 # Number of reaction wheels in the assembly

    "Jacobian mapping wheel angular velocity to body angular momentum"
    J_rw::Matrix{Float64} = zeros(Float64, 3, 0)

    "Precomputed pseudo-inverse of J_rw, mapping a desired body torque/momentum onto per-wheel values"
    J_rw_pinv::Matrix{Float64} = Matrix{Float64}(pinv(J_rw))

    "Maximum torque (Nm) *each* wheel can produce"
    max_wheel_torque::Float64 = 0.1
    
    "Maximum angular momentum (Nms) *each* wheel can store"
    max_wheel_h::Float64 = 0.1

    "Attitude control algorithm associated with this RW assembly"
    attitude_control_function::Function = (m, b, root_index, vel_pp_rw, h_pp_hat, aerobraking_phase, t) -> nothing

    # --- State (Mutable) ---
    "Angular momentum (h) of each wheel"
    h_wheels::Vector{Float64} = zeros(Float64, n_wheels)

    "Derivative of wheel angular momentum (h_dot)"
    h_dot_wheels::Vector{Float64} = zeros(Float64, n_wheels)
    
    "Net torque (τ) applied to the body *by* the RW assembly"
    tau_body_net::MVector{3, Float64} = MVector{3, Float64}(zeros(3))
end
