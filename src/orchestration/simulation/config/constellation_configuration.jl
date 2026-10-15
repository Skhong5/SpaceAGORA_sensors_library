# The existing spacecraft collection and selected dynamics effectors for one run.
# Included inside SpacecraftModels to preserve DynamicsModel type identity and public access.
# No additional collection wrapper or constellation-generation algorithm is introduced.

"""
    DynamicsModel(spacecraft, dynamic_effectors)

Struct containing the information needed for the dynamics propagation of all spacecraft.

- `spacecraft`: vector of spacecraft models, one per propagated vehicle, each given by its root link (main bus or core body). This is the vector over which the threads are parallelized, so each thread handles the dynamics propagation of one spacecraft and its attached links.
- `dynamic_effectors`: tuple of force and torque models (gravity, drag, etc.) applied during the dynamics propagation.

After a run, `res.configuration.dynamics_model.spacecraft[i]` is the spacecraft that ran.
"""
struct DynamicsModel{T_Effectors<:Tuple}
    spacecraft::Vector{SpacecraftModel} # Vector of root links (main bus or core bodies)
    dynamic_effectors::T_Effectors # Tuple of dynamic effector models (gravity, drag, etc.)

    function DynamicsModel(roots::Vector{SpacecraftModel}, dynamic_effectors::T_Effectors) where {T_Effectors<:Tuple}
        new{T_Effectors}(roots, dynamic_effectors)
    end
end
