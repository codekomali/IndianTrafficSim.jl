# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Vehicles

export plot_vehicles!

using Agents
using InteractiveDynamics
using GLMakie

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

function initialize(
    n_vehicles=2,
    extent=(4000, 4000))
    space2d = ContinuousSpace(extent)
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly)
    vel = (1.0, 0.0)
    x = 0.0
    y = 0.0
   add_agent!(
            (x, y),
            model,
            vel
        )
    return model
end

# Average Car width 1.8 m
# ∴ Car width = 1.8 * 15.5 ≈ 27.9 units (rounded to 28)
# Average Car length 4.5 m
# ∴ Car length = 4.5 * 15.5 ≈ 68.75 units (rounded to 69)
# source : https://measuringstuff.com/car-length-and-width-measured-in-feet/

const vehicle_polygon = Polygon(
    Point2f[(0, 0), (69, 0), (69, 28), (0, 28)]
)

function vehicle_marker(v::VehicleAgent)
    φ = atan(v.vel[2], v.vel[1]) #+ π/2 + π
    #scale(rotate2D(vehicle_polygon, φ), 1)
    rotate2D(vehicle_polygon, φ)
end

function plot_vehicles!()
    axiskwargs = (title="Indian Traffic Simulator", titlealign=:left) #title and position
    fig, _ = abmplot(
        initialize();
        am=vehicle_marker,
        ac=:green,
        axiskwargs=axiskwargs
    )
    return fig
end

plot_vehicles!()

end

