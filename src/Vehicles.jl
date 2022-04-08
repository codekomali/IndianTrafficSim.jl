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
    extent=(P.EXTENT_WIDTH, P.EXTENT_HEIGHT))
    space2d = ContinuousSpace(extent)
    properties = Dict()
    properties[:tick] = 0
    intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)
    properties[:env] = intersectingRoads
    properties[:spawn_rate] = 1400
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly; properties=properties)
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



# Generalize
function vehicle_poly()
    hw = P.VEHICLE_WIDTH/2
    pt = Point2f[(0,-hw),(P.VEHICLE_LENGTH, -hw),(P.VEHICLE_LENGTH, hw), (0, hw)]
    return Polygon(pt)
end

function vehicle_marker(v::VehicleAgent)
    φ = atan(v.vel[2], v.vel[1])
    rotate2D(vehicle_poly(), φ)
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
    foreach(draw_cross_path!,cross_paths(twir))
end

function cross_paths(twir)
    cps = []
    xpos = top_boundary(twir.leftRoadSeg)[2][1] - P.CROSS_PATH_WIDTH - P.CROSS_PATH_MARGIN
    push!(cps, HcrossPath(twir.leftRoadSeg, xpos))
    xpos = top_boundary(twir.rightRoadSeg)[1][1] + P.CROSS_PATH_WIDTH + P.CROSS_PATH_MARGIN
    push!(cps, HcrossPath(twir.rightRoadSeg, xpos))
    ypos = right_boundary(twir.topRoadSeg)[2][2] + P.CROSS_PATH_WIDTH + P.CROSS_PATH_MARGIN
    push!(cps, VcrossPath(twir.topRoadSeg, ypos))
    ypos = right_boundary(twir.bottomRoadSeg)[1][2] - P.CROSS_PATH_WIDTH - P.CROSS_PATH_MARGIN
    push!(cps, VcrossPath(twir.bottomRoadSeg, ypos))
    return cps
end

function mark_spawn_positions(twir::TwoWayIntersectingRoads)
    for sp in spawnPos(twir)
        spawnPt = Point2f(sp.pos...)
        scatter!(spawnPt, color=:green, markersize=P.SIGNAL_MS)
    end
end

function model_step!(model)
    model.tick += 1
    if (model.tick % model.spawn_rate ==0)
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
    axiskwargs = (title=P.PLOT_TITLE, titlealign=P.PLOT_TITLE_ALIGN)
    model = initialize()
    params = Dict(
        :spawn_rate => P.VSR_MIN:P.VSR_INC:P.VSR_MAX,
        )
    fig, _ = abmplot(
        model;
        agent_step! = vehicle_step!,
        model_step! = model_step!,
        am=vehicle_marker,
        params = params,
        ac=:green,
        axiskwargs=axiskwargs
    )
    plot_environment!(model)
    return fig
end



#intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)

#a macro for easing development only. will be removed later
macro pv()
    quote
        plot_vehicles!()
    end
end

# end

