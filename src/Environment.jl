# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

#module Environment

# Scale
# Typical lane width = 3.7 m
# Current lane width = 50 units
# ∴ 1 m = (50/3.7) ≈ 15.5 units

# export plot_environment!

using Agents
using InteractiveDynamics
using GLMakie

# include("Vehicles.jl")
# using .Vehicles: plot_vehicles!

include("Utils.jl")
include("Parameters.jl")

import .Parameters as P
import .Utils as U

abstract type Road end

mutable struct Signal
    pos::NTuple{2,Float64}
    state::Symbol
    countDown::Int8
    plot::Union{Scatter,Missing}
    active::Bool
    function Signal(pos, state)
        if state === :red
            return new(pos, state, P.SIGNAL_RED_TIME, missing, true)
        elseif state === :green
            return new(pos, state, P.SIGNAL_GREEN_TIME, missing, true)
        else
            return new(pos, state, P.SIGNAL_YELLOW_TIME, missing, true)
        end
    end
end

function transitionState!(s::Signal)
    if s.state === :red
        s.state = :green
        s.countDown = P.SIGNAL_GREEN_TIME
    elseif s.state === :green
        s.state = :yellow
        s.countDown = P.SIGNAL_YELLOW_TIME
    else
        s.state = :red
        s.countDown = P.SIGNAL_RED_TIME
    end
    return nothing
end

function processSignalState!(s::Signal)
    if s.countDown == 0
        transitionState!(s)
    end
    s.countDown -= 1
    return nothing
end

function setSignalState!(s::Signal, state::Symbol)
    if state === :green
        s.state = :green
        s.countDown = P.SIGNAL_GREEN_TIME
    elseif state === :yellow
        s.state = :yellow
        s.countDown = P.SIGNAL_YELLOW_TIME
    elseif state === :red
        s.state = :red
        s.countDown = P.SIGNAL_RED_TIME
    else
        error("Unrecognized signal state", s)
    end
end

half_width(road::Road) = road.numLanes * road.laneWidth / 2
half_width(numLanes, laneWidth) = numLanes * laneWidth / 2


function hsignalPos(Ypos, startXpos, endXpos, half_width)
    yoffset = half_width + P.SIGNAL_POS_Y_MARGIN
    if startXpos < endXpos
        return (endXpos - P.SIGNAL_POS_X_MARGIN, Ypos + yoffset)
    else
        return (endXpos + P.SIGNAL_POS_X_MARGIN, Ypos - yoffset)
    end
end

function vsignalPos(Xpos, startYpos, endYpos, half_width)
    xoffset = half_width + P.SIGNAL_POS_X_MARGIN
    if startYpos < endYpos
        return (Xpos - xoffset, endYpos - P.SIGNAL_POS_Y_MARGIN)
    else
        return (Xpos + xoffset, endYpos + P.SIGNAL_POS_Y_MARGIN)
    end
end

struct HorizontalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int64
    laneWidth::Float64
    spawnPos::Vector{NTuple{2,Float64}}
    signal::Signal
    function HorizontalRoad(Ypos, startXpos, endXpos; numLanes=2, laneWidth=50)
        startPos = (startXpos, Ypos)
        endPos = (endXpos, Ypos)
        lanemid = (Ypos - half_width(numLanes, laneWidth)) + laneWidth / 2
        spawnPos = []
        for _ in 1:numLanes
            spawnPos = push!(spawnPos, (startXpos, lanemid))
            lanemid += laneWidth
        end
        signalPos = hsignalPos(Ypos, startXpos, endXpos, half_width(numLanes, laneWidth))
        signal = Signal(signalPos, :red)
        return new(startPos, endPos, numLanes, laneWidth, spawnPos, signal)
    end
end

top_boundary(road::HorizontalRoad) = (
    road.startPos .+ (0, half_width(road)),
    road.endPos .+ (0, half_width(road))
)
bottom_boundary(road::HorizontalRoad) = (
    road.startPos .- (0, half_width(road)),
    road.endPos .- (0, half_width(road))
)
struct VerticalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int64
    laneWidth::Float64
    spawnPos::Vector{NTuple{2,Float64}}
    signal::Signal
    function VerticalRoad(Xpos, startYpos, endYpos; numLanes=2, laneWidth=50)
        startPos = (Xpos, startYpos)
        endPos = (Xpos, endYpos)
        lanemid = (Xpos - half_width(numLanes, laneWidth)) + laneWidth / 2
        spawnPos = []
        for _ in 1:numLanes
            spawnPos = push!(spawnPos, (lanemid, startYpos))
            lanemid += laneWidth
        end
        signalPos = vsignalPos(Xpos, startYpos, endYpos, half_width(numLanes, laneWidth))
        signal = Signal(signalPos, :red)
        return new(startPos, endPos, numLanes, laneWidth, spawnPos, signal)
    end
end

right_boundary(road::VerticalRoad) = (
    road.startPos .+ (half_width(road), 0),
    road.endPos .+ (half_width(road), 0)
)
left_boundary(road::VerticalRoad) = (
    road.startPos .- (half_width(road), 0),
    road.endPos .- (half_width(road), 0)
)
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
        R2Lroad = HorizontalRoad(R2LroadYpos, endXpos, startXpos; numLanes=numLanes, laneWidth=laneWidth)
        return new(L2Rroad, R2Lroad, medianWidth, Ypos)
    end
end

signals(road::TwoWayHroad) = [road.L2Rroad.signal, road.R2Lroad.signal]
numlanes(road::TwoWayHroad) = road.L2Rroad.numLanes
lanewidth(road::TwoWayHroad) = road.L2Rroad.laneWidth
startXPos(road::TwoWayHroad) = road.L2Rroad.startPos[1]
endXPos(road::TwoWayHroad) = road.L2Rroad.endPos[1]
leftSpawnPos(road::TwoWayHroad) = road.L2Rroad.spawnPos
rightSpawnPos(road::TwoWayHroad) = road.R2Lroad.spawnPos

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
        B2Troad = VerticalRoad(B2TroadXpos, endYpos, startYpos; numLanes=numLanes, laneWidth=laneWidth)
        return new(T2Broad, B2Troad, medianWidth, Xpos)
    end
end

signals(road::TwoWayVroad) = [road.B2Troad.signal, road.T2Broad.signal]
numlanes(road::TwoWayVroad) = road.B2Troad.numLanes
lanewidth(road::TwoWayVroad) = road.B2Troad.laneWidth
totalWidth(road::Union{TwoWayHroad,TwoWayVroad}) = (2 * numlanes(road) * lanewidth(road)) + road.medianWidth
startYPos(road::TwoWayVroad) = road.T2Broad.startPos[2]
endYPos(road::TwoWayVroad) = road.T2Broad.endPos[2]
topSpawnPos(road::TwoWayVroad) = road.T2Broad.spawnPos
bottomSpawnPos(road::TwoWayVroad) = road.B2Troad.spawnPos

struct TwoWayIntersectingRoads <: Road
    leftRoadSeg::TwoWayHroad
    rightRoadSeg::TwoWayHroad
    topRoadSeg::TwoWayVroad
    bottomRoadSeg::TwoWayVroad
    function TwoWayIntersectingRoads(Ypos, startXpos, endXpos, Xpos, startYpos, endYpos;
        vnumlanes=2,
        vlanewidth=50,
        vmedianwidth=2,
        hnumlanes=2,
        hlanewidth=50,
        hmedianwidth=2
    )
        horizontalWidth = (2 * vnumlanes * vlanewidth) + vmedianwidth
        verticalWidth = (2 * hnumlanes * hlanewidth) + hmedianwidth
        leftRoadSeg = TwoWayHroad(Ypos, startXpos, Xpos - horizontalWidth / 2)
        rightRoadSeg = TwoWayHroad(Ypos, Xpos + horizontalWidth / 2, endXpos)
        topRoadSeg = TwoWayVroad(Xpos, startYpos, Ypos + verticalWidth / 2)
        bottomRoadSeg = TwoWayVroad(Xpos, Ypos - verticalWidth / 2, endYpos)
        return new(leftRoadSeg, rightRoadSeg, topRoadSeg, bottomRoadSeg)
    end
end

roadsegs(road::TwoWayIntersectingRoads) = [road.leftRoadSeg, road.topRoadSeg, road.rightRoadSeg, road.bottomRoadSeg]
signals(road::TwoWayIntersectingRoads) = U.flatten(map(x->signals(x),roadsegs(road)))
leftSpawnPos(road::TwoWayIntersectingRoads) = leftSpawnPos(road.leftRoadSeg)
rightSpawnPos(road::TwoWayIntersectingRoads) = rightSpawnPos(road.rightRoadSeg)
topSpawnPos(road::TwoWayIntersectingRoads) = topSpawnPos(road.topRoadSeg)
bottomSpawnPos(road::TwoWayIntersectingRoads) = bottomSpawnPos(road.bottomRoadSeg)

# Move to Utils
draw_line!(pos1, pos2; kwargs...) = lines!([pos1[1], pos2[1]], [pos1[2], pos2[2]]; kwargs...)
reduce_y(line, val) = (line[1] .- (0, val), line[2] .- (0, val))
increase_y(line,val)= (line[1] .+ (0, val), line[2] .+ (0, val))
reduce_x(line, val) = (line[1] .- (val, 0), line[2] .- (val, 0))
increase_x(line, val) = (line[1] .+ (val, 0), line[2] .+ (val, 0))

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
    return nothing
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
    return nothing
end

function draw_signal!(road::Road)
    draw_signal!(road.signal)
    return nothing
end

function draw_signal!(road::Union{TwoWayHroad, TwoWayVroad, TwoWayIntersectingRoads})
    foreach(draw_signal!, signals(road))
    return nothing
end

function draw_signal!(s::Signal)
    s.active || return nothing
    if s.plot === missing
        signalPt = Point2f(s.pos...)
        plot = scatter!(signalPt, color=s.state, markersize=P.SIGNAL_MS)
        s.plot = plot
    else
        processSignalState!(s)
        s.plot.color = s.state
    end
    return nothing
end


function draw_lane_markers!(road::HorizontalRoad)
    last_boundary = top_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = reduce_y(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=:blue, linestyle=:dash)
    end
    return nothing
end

function draw_lane_markers!(road::VerticalRoad)
    last_boundary = right_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = reduce_x(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=:blue, linestyle=:dash)
    end
    return nothing
end

function draw_pedestrian_walkway!(road::HorizontalRoad)
    pedWay = nothing
    if road.startPos[1] < road.endPos[1]
        pedWay = increase_y(top_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    else
        pedWay = reduce_y(bottom_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    end
    draw_line!(pedWay..., color=:orange)
end

function draw_pedestrian_walkway!(road::VerticalRoad)
    pedWay = nothing
    if road.startPos[2] > road.endPos[2]
        pedWay = increase_x(right_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    else
        pedWay = reduce_x(left_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    end
    draw_line!(pedWay..., color=:orange)
end

function drawRoad!(road::Road)
    draw_road_boundaries!(road)
    draw_lane_markers!(road)
    draw_signal!(road)
    draw_pedestrian_walkway!(road)
    return nothing
end

function drawMedian!(road::TwoWayHroad)
    startPt = (startXPos(road), road.Ypos)
    endPt = (endXPos(road), road.Ypos)
    draw_line!(startPt, endPt, color=:green, linewidth=road.medianWidth)
    return nothing
end

function drawMedian!(road::TwoWayVroad)
    startPt = (road.Xpos, startYPos(road))
    endPt = (road.Xpos, endYPos(road))
    draw_line!(startPt, endPt, color=:green, linewidth=road.medianWidth)
    return nothing
end

function drawRoad!(road::TwoWayHroad)
    drawRoad!(road.L2Rroad)
    drawMedian!(road)
    drawRoad!(road.R2Lroad)
    return nothing
end

function drawRoad!(road::TwoWayVroad)
    drawRoad!(road.B2Troad)
    drawMedian!(road)
    drawRoad!(road.T2Broad)
    return nothing
end

function drawRoad!(interRoad::TwoWayIntersectingRoads)
    drawRoad!(interRoad.leftRoadSeg)
    drawRoad!(interRoad.topRoadSeg)
    drawRoad!(interRoad.rightRoadSeg)
    drawRoad!(interRoad.bottomRoadSeg)
    return nothing
end






