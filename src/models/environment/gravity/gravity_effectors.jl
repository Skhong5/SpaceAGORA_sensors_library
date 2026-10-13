# Compatibility facade only. Physical field kernels are in this directory;
# spacecraft wrench adapters are owned by DynamicEffectors.GravityEffectors.
module GravityEffectors

using ..DynamicEffectors.GravityEffectors: ConstantGravityModel, InverseSquaredGravityModel, InverseSquaredJ2GravityModel
using ..DynamicEffectors.GravityEffectors: GravityGradientTorqueModel
using ..DynamicEffectors.GravityEffectors: aerobraking_gravity_force_ii, j2_secular_rates

export ConstantGravityModel, InverseSquaredGravityModel, InverseSquaredJ2GravityModel
export GravityGradientTorqueModel
export aerobraking_gravity_force_ii, j2_secular_rates

end # module GravityEffectors
