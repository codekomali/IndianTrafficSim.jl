# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

#module Environment

## Scale
## Typical lane width = 3.7 m
## Current lane width = 50 units
## ∴ 1 m = (50/3.7) ≈ 15.5 units

export plot_environment!

using Agents
using InteractiveDynamics
using GLMakie

include("Vehicles.jl")
using .Vehicles: plot_vehicles!

abstract type Road end

struct HorizontalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int
    laneWidth::Float64
    function HorizontalRoad(Ypos, startXpos, endXpos; numLanes=2, laneWidth=50)
        startPos = (startXpos, Ypos)
        endPos = (endXpos, Ypos)
        return new(startPos, endPos, numLanes, laneWidth)
    end
end

struct VerticalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int
    laneWidth::Float64
    function VerticalRoad(Xpos, startYpos, endYpos; numLanes=2, laneWidth=50)
        startPos = (Xpos, startYpos)
        endPos = (Xpos, endYpos)
        return new(startPos, endPos, numLanes, laneWidth)
    end
end

struct TwoWayHroad <: Road
    L2Rroad::HorizontalRoad
    R2Lroad::HorizontalRoad
    medianWidth::Float64
    Ypos::Float64
    function TwoWayHroad(Ypos, startXpos, endXpos; medianWidth=2, numLanes=2, laneWidth=50)
        widthBetweenRoads = (medianWidth + (numLanes * laneWidth)) / 2
        L2RroadYpos = Ypos + widthBetweenRoads
        R2LroadYpos = Ypos - widthBetweenRoads
        L2Rroad = HorizontalRoad(L2RroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
        R2Lroad = HorizontalRoad(R2LroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
        new(L2Rroad, R2Lroad, medianWidth, Ypos)
    end
end

startXPos(road::TwoWayHroad) = road.L2Rroad.startPos[1]
endXPos(road::TwoWayHroad) = road.L2Rroad.endPos[1]

struct TwoWayVroad <: Road
    T2Broad::VerticalRoad
    B2Troad::VerticalRoad
    medianWidth::Float64
    Xpos::Float64
    function TwoWayVroad(Xpos, startYpos, endYpos; medianWidth=2, numLanes=2, laneWidth=50)
        widthBetweenRoads = (medianWidth + (numLanes * laneWidth)) / 2
        T2BroadXpos = Xpos + widthBetweenRoads
        B2TroadXpos = Xpos - widthBetweenRoads
        T2Broad = VerticalRoad(T2BroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
        B2Troad = VerticalRoad(B2TroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
        new(T2Broad, B2Troad, medianWidth, Xpos)
    end
end

startYPos(road::TwoWayVroad) = road.T2Broad.startPos[2]
endYPos(road::TwoWayVroad) = road.T2Broad.endPos[2]

function draw_line!(pos1, pos2; kwargs...)
    lines!([pos1[1], pos2[1]], [pos1[2], pos2[2]]; kwargs...)
end

half_width(road::Road) = road.numLanes * road.laneWidth / 2

top_boundary(road::HorizontalRoad) = (
    road.startPos .+ (0, half_width(road)),
    road.endPos .+ (0, half_width(road))
)
right_boundary(road::VerticalRoad) = (
    road.startPos .+ (half_width(road), 0),
    road.endPos .+ (half_width(road), 0)
)
bottom_boundary(road::HorizontalRoad) = (
    road.startPos .- (0, half_width(road)),
    road.endPos .- (0, half_width(road))
)
left_boundary(road::VerticalRoad) = (
    road.startPos .- (half_width(road), 0),
    road.endPos .- (half_width(road), 0)
)

reduce_y(line, val) = (line[1] .- (0, val), line[2] .- (0, val))

reduce_x(line, val) = (line[1] .- (val, 0), line[2] .- (val, 0))

function draw_road_boundaries!(road::HorizontalRoad)
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

function draw_road_boundaries!(road::VerticalRoad)
    draw_line!(
        right_boundary(road)...;
        color=:red,
        linewidth=2
    )
    draw_line!(
        left_boundary(road)...;
        color=:red,
        linewidth=2
    )
end

function draw_lane_markers!(road::HorizontalRoad)
    last_boundary = top_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = reduce_y(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=:blue, linestyle=:dash)
    end
end

function draw_lane_markers!(road::VerticalRoad)
    last_boundary = right_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = reduce_x(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=:blue, linestyle=:dash)
    end
end

function drawRoad!(road::Road)
    draw_road_boundaries!(road)
    draw_lane_markers!(road)
end

function drawMedian!(road::TwoWayHroad)
    startPt = (startXPos(road), road.Ypos)
    endPt = (endXPos(road), road.Ypos)
    draw_line!(startPt, endPt, color=:green, linewidth=road.medianWidth)
end

function drawMedian!(road::TwoWayVroad)
    startPt = (road.Xpos, startYPos(road))
    endPt = (road.Xpos, endYPos(road))
    draw_line!(startPt, endPt, color=:green, linewidth=road.medianWidth)
end

function drawRoad!(road::TwoWayHroad)
    drawRoad!(road.L2Rroad)
    drawMedian!(road)
    drawRoad!(road.R2Lroad)
end

function drawRoad!(road::TwoWayVroad)
    drawRoad!(road.B2Troad)
    drawMedian!(road)
    drawRoad!(road.T2Broad)
end

function initialize_empty_figure(;figure=NamedTuple(),axis=NamedTuple())
    fig = Figure(; figure...)
    ax = fig[1,1][1,1] = Axis(fig; axis...)
    return fig, ax
end

function plot_environment!()
    fig=plot_vehicles!()
    road = TwoWayHroad(2000, 0, 4000)
    drawRoad!(road)
    return fig
end

plot_environment!()
#end

