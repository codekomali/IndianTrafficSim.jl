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

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

function initialize(
    n_vehicles=2,
    extent=(4000, 4000))
    space2d = ContinuousSpace(extent)
    properties = Dict()
    properties[:tick] = 0
    intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)
    properties[:env] = intersectingRoads
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly;properties=properties)
    add_vehicle!((0.0,2010.0), model)
    return model
end

function add_vehicle!(spawn_pos, model, initial_vel=(50,0))
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

vehicle_step!(agent, model) = move_agent!(agent, model)

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
end

function model_step!(model)
    model.tick += 1
    (model.tick % 10 ==0) && add_vehicle!((0.0,2050.0), model)
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

