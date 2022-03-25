module IndianTrafficSim

include("Environment.jl")
include("Vehicles.jl")

using .Environment: plot_environment!
using .Vehicles: plot_vehicles!

export plot_environment!
export plot_vehicles!

end
