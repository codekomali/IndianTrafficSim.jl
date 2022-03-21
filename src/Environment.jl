# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

using Agents
using InteractiveDynamics
using CairoMakie

struct Road
    alignment::Symbol
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int
    laneWidth::Float64
end

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

function initialize(
    n_vehicles=2,
    extent=(100, 100),
)
    space2d = ContinuousSpace(extent)
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly)
    vel = (1.0, 0.0)
    x = 0.0
    y = 42.0
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

function draw_line!(pos1, pos2; kwargs...)
    lines!([pos1[1], pos2[1]], [pos1[2], pos2[2]]; kwargs...)
end

half_width(road::Road) = road.numLanes * road.laneWidth / 2

top_boundary(road::Road) = (
    road.startPos .+ (0, half_width(road)),
    road.endPos .+ (0, half_width(road))
)
bottom_boundary(road::Road) = (
    road.startPos .- (0, half_width(road)),
    road.endPos .- (0, half_width(road))
)

function draw_road_boundaries(road::Road)
    draw_line!(
        top_boundary(road)...;
        color=:red,
        linewidth=2
    )
    draw_line!(
        bottom_boundary(road)...;
        color=:red,
        linewidth=2
    )
end

reduce_y(line, val) = (line[1] .-(0,val), line[2] .- (0, val))

function draw_lane_markers(road::Road)
    last_boundary = top_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = reduce_y(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=:blue, linestyle=:dash)
    end
end

function drawRoad!(road::Road)
    draw_road_boundaries(road)
    draw_lane_markers(road)
end

function initial_plot()
    axiskwargs = (title="Indian Traffic Simulator", titlealign=:left) #title and position
    fig, abmstepper = abm_plot(
        initialize();
        am=vehicle_marker,
        ac=:blue,
        axiskwargs=axiskwargs
    )
    road = Road(:horizontal, (0, 50), (100, 50), 6, 5)
    drawRoad!(road)
    return fig
end

fig = initial_plot()