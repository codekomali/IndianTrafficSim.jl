# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# module Vehicles

# export plot_vehicles!

using Agents
using InteractiveDynamics
using GLMakie

include("Environment.jl")
include("Parameters.jl")

import .Parameters as P

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

function initialize(
    n_vehicles=2,
    extent=(4100, 4100))
    space2d = ContinuousSpace(extent)
    properties = Dict()
    properties[:tick] = 0
    intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)
    properties[:env] = intersectingRoads
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly;properties=properties)
    add_vehicle!((4000.0, 1924.0), model, (-0.1,0.0))
    return model
end


function add_vehicle!(spawn_pos, model, initial_vel=(P.VEHICLE_INITIAL_SPEED,0.0))
   add_agent!(
            spawn_pos,
            model,
            initial_vel
        )
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


function vehicle_step!(agent, model)
    move_agent!(agent, model)
end

function plot_environment!(model)
    custom_setup_environment!(model.env)
    drawRoad!(model.env)
end

function custom_setup_environment!(twir::TwoWayIntersectingRoads)
    # Make the signals on the edge inactive
    twir.leftRoadSeg.R2Lroad.signal.active = false
    twir.rightRoadSeg.L2Rroad.signal.active = false
    twir.topRoadSeg.B2Troad.signal.active = false
    twir.bottomRoadSeg.T2Broad.signal.active = false
    # Alternate L2R and T2B road signals
    setSignalState!(twir.leftRoadSeg.L2Rroad.signal, :green)
    setSignalState!(twir.rightRoadSeg.R2Lroad.signal, :green)
    setSignalState!(twir.topRoadSeg.T2Broad.signal, :red)
    setSignalState!(twir.bottomRoadSeg.B2Troad.signal, :red)
    mark_spawn_positions(twir)
end

function mark_spawn_positions(twir::TwoWayIntersectingRoads)
    for sp in spawnPos(twir)
        spawnPt = Point2f(sp.pos...)
        scatter!(spawnPt, color=:green, markersize=P.SIGNAL_MS)
    end
end

function model_step!(model)
    model.tick += 1
    if (model.tick % 1400 ==0)
        rnd_spawn_pos = rand(spawnPos(model.env))
        @show rnd_spawn_pos
        spawn_vel = rnd_spawn_pos.orient .* P.VEHICLE_INITIAL_SPEED
        @show spawn_vel
        add_vehicle!(rnd_spawn_pos.pos, model, spawn_vel)
    end
    draw_signal!(model.env)
    @show model.tick
end



function plot_vehicles!()
    axiskwargs = (title="Indian Traffic Simulator", titlealign=:left) #title and position
    model = initialize()
    fig, _ = abmplot(
        model;
        agent_step! = vehicle_step!,
        model_step! = model_step!,
        am=vehicle_marker,
        ac=:green,
        axiskwargs=axiskwargs
    )
    plot_environment!(model)
    return fig
end

#intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)
plot_vehicles!()

# end

