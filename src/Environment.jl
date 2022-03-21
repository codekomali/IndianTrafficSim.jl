# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

using Agents
using InteractiveDynamics
using CairoMakie

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

function initialize(
    n_vehicles=2,
    visual_distance=5.0,
    extent=(100, 100),
    spacing=visual_distance / 1.5,
)
    space2d = ContinuousSpace(extent, spacing)
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly)
    vel = (1.0, 0.0)
    x = 0.0
    y = 50.0
    for _ in 1:n_vehicles
        add_agent!(
            (x, y += 5.0),
            model,
            vel
        )
    end
    return model
end

const vehicle_polygon = Polygon(Point2f[(0, 0), (2, 0), (2, 1), (0, 1)])

function vehicle_marker(v::VehicleAgent)
    φ = atan(v.vel[2], v.vel[1]) #+ π/2 + π
    scale(rotate2D(vehicle_polygon, φ), 1)
end

model = initialize()

axiskwargs = (title="Traffic Simulation", titlealign=:left) #title and position

fig, abmstepper = abm_plot(
    model;
    am=vehicle_marker,
    ac=:blue
)
fig