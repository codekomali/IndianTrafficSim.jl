# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Environment

export initial_plot!

using Agents
using InteractiveDynamics
using CairoMakie

abstract type Road end

struct HorizontalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int
    laneWidth::Float64
    function HorizontalRoad(Ypos, startXpos, endXpos; numLanes=2, laneWidth=5)
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
    function VerticalRoad(Xpos, startYpos, endYpos; numLanes=2, laneWidth=5)
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
    function TwoWayHroad(Ypos, startXpos, endXpos; medianWidth=2, numLanes=2, laneWidth=5 )
    widthBetweenRoads = (medianWidth + (numLanes * laneWidth))/2
    L2RroadYpos = Ypos + widthBetweenRoads
    R2LroadYpos = Ypos - widthBetweenRoads
    L2Rroad = HorizontalRoad(L2RroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
    R2Lroad = HorizontalRoad(R2LroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
    new(L2Rroad,R2Lroad,medianWidth,Ypos)
    end
end

startXPos(road::TwoWayHroad) = road.L2Rroad.startPos[2]
endXPos(road::TwoWayHroad) = road.L2Rroad.endPos[2]

struct TwoWayVroad <: Road
    T2Broad::VerticalRoad
    B2Troad::VerticalRoad
    medianWidth::Float64
    Xpos::Float64
    function TwoWayVroad(Xpos, startYpos, endYpos; medianWidth=2, numLanes=2, laneWidth=5 )
    widthBetweenRoads = (medianWidth + (numLanes * laneWidth))/2
    T2BroadXpos = Xpos + widthBetweenRoads
    B2TroadXpos = Xpos - widthBetweenRoads
    T2Broad = VerticalRoad(T2BroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
    B2Troad = VerticalRoad(B2TroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
    new(T2Broad,B2Troad,medianWidth, Xpos)
    end
end

startYPos(road::TwoWayVroad) = road.T2Broad.startPos[1]
endYPos(road::TwoWayVroad) = road.T2Broad.endPos[1]

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

reduce_y(line, val) = (line[1] .-(0,val), line[2] .- (0, val))

reduce_x(line, val) = (line[1] .-(val,0), line[2] .- (val,0))

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

function drawRoad!(road::TwoWayHroad)
    drawRoad!(road.L2Rroad)
    drawRoad!(road.R2Lroad)
end

function drawRoad!(road::TwoWayVroad)
    drawRoad!(road.B2Troad)
    drawRoad!(road.T2Broad)
end

function initial_plot!()
    axiskwargs = (title="Indian Traffic Simulator", titlealign=:left) #title and position
    fig, _ = abmplot(
        initialize();
        am=vehicle_marker,
        ac=:blue,
        axiskwargs=axiskwargs
    )
    road = TwoWayVroad(50,0,100)
    drawRoad!(road)
    return fig
end


end