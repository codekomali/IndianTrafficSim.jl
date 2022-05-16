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
using LinearAlgebra

include("Utils.jl")
include("Parameters.jl")

import .Parameters as P
import .Utils as U

abstract type Road end

mutable struct Signal
    pos::NTuple{2,Float64}
    state::Symbol
    countDown::Int64
    plot::Union{Scatter,Missing}
    active::Bool
end

function Signal(pos, state)
    if state === :red
        return Signal(pos, state, P.SIGNAL_RED_TIME, missing, true)
    elseif state === :green
        return Signal(pos, state, P.SIGNAL_GREEN_TIME, missing, true)
    else
        return Signal(pos, state, P.SIGNAL_YELLOW_TIME, missing, true)
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

mutable struct SpawnPosition
    pos::NTuple{2,Float64}
    orient::NTuple{2,Float64}
end

mutable struct HorizontalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int64
    laneWidth::Float64
    spawnPos::Vector{SpawnPosition}
    signal::Signal
end

function HorizontalRoad(Ypos, startXpos, endXpos; numLanes=P.H_NUM_LANES, laneWidth=P.H_LANE_WIDTH)
    startPos = (startXpos, Ypos)
    endPos = (endXpos, Ypos)
    lanemid = (Ypos - half_width(numLanes, laneWidth)) + laneWidth / 2
    spawnPos = []
    for _ in 1:numLanes
        laneSpawnPos = SpawnPosition((startXpos, lanemid),U.orientation(startPos, endPos))
        spawnPos = push!(spawnPos, laneSpawnPos)
        lanemid += laneWidth
    end
    signalPos = hsignalPos(Ypos, startXpos, endXpos, half_width(numLanes, laneWidth))
    signal = Signal(signalPos, :red)
    return HorizontalRoad(startPos, endPos, numLanes, laneWidth, spawnPos, signal)
end

top_boundary(road::HorizontalRoad) = (
    road.startPos .+ (0, half_width(road)),
    road.endPos .+ (0, half_width(road))
)
bottom_boundary(road::HorizontalRoad) = (
    road.startPos .- (0, half_width(road)),
    road.endPos .- (0, half_width(road))
)

mutable struct VerticalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int64
    laneWidth::Float64
    spawnPos::Vector{SpawnPosition}
    signal::Signal
end

function VerticalRoad(Xpos, startYpos, endYpos; numLanes=P.V_NUM_LANES, laneWidth=P.V_LANE_WIDTH)
    startPos = (Xpos, startYpos)
    endPos = (Xpos, endYpos)
    lanemid = (Xpos - half_width(numLanes, laneWidth)) + laneWidth / 2
    spawnPos = []
    for _ in 1:numLanes
        laneSpawnPos = SpawnPosition((lanemid, startYpos),U.orientation(startPos, endPos))
        spawnPos = push!(spawnPos, laneSpawnPos)
        lanemid += laneWidth
    end
    signalPos = vsignalPos(Xpos, startYpos, endYpos, half_width(numLanes, laneWidth))
    signal = Signal(signalPos, :red)
    return VerticalRoad(startPos, endPos, numLanes, laneWidth, spawnPos, signal)
end

right_boundary(road::VerticalRoad) = (
    road.startPos .+ (half_width(road), 0),
    road.endPos .+ (half_width(road), 0)
)
left_boundary(road::VerticalRoad) = (
    road.startPos .- (half_width(road), 0),
    road.endPos .- (half_width(road), 0)
)
mutable struct TwoWayHroad <: Road
    L2Rroad::HorizontalRoad
    R2Lroad::HorizontalRoad
    medianWidth::Float64
    Ypos::Float64
end

function TwoWayHroad(Ypos, startXpos, endXpos; medianWidth=P.H_MEDIAN_WIDTH, numLanes=P.H_NUM_LANES, laneWidth=P.H_LANE_WIDTH)
    widthBetweenRoads = (medianWidth + (numLanes * laneWidth)) / 2
    L2RroadYpos = Ypos + widthBetweenRoads
    R2LroadYpos = Ypos - widthBetweenRoads
    L2Rroad = HorizontalRoad(L2RroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
    R2Lroad = HorizontalRoad(R2LroadYpos, endXpos, startXpos; numLanes=numLanes, laneWidth=laneWidth)
    return TwoWayHroad(L2Rroad, R2Lroad, medianWidth, Ypos)
end

signals(road::TwoWayHroad) = [road.L2Rroad.signal, road.R2Lroad.signal]
numlanes(road::TwoWayHroad) = road.L2Rroad.numLanes
lanewidth(road::TwoWayHroad) = road.L2Rroad.laneWidth
startXPos(road::TwoWayHroad) = road.L2Rroad.startPos[1]
endXPos(road::TwoWayHroad) = road.L2Rroad.endPos[1]
leftSpawnPos(road::TwoWayHroad) = road.L2Rroad.spawnPos
rightSpawnPos(road::TwoWayHroad) = road.R2Lroad.spawnPos
top_boundary(road::TwoWayHroad) = top_boundary(road.L2Rroad)
bottom_boundary(road::TwoWayHroad) = bottom_boundary(road.R2Lroad)

mutable struct TwoWayVroad <: Road
    T2Broad::VerticalRoad
    B2Troad::VerticalRoad
    medianWidth::Float64
    Xpos::Float64
end

function TwoWayVroad(Xpos, startYpos, endYpos; medianWidth=P.V_MEDIAN_WIDTH, numLanes=P.V_NUM_LANES, laneWidth=P.V_LANE_WIDTH)
    widthBetweenRoads = (medianWidth + (numLanes * laneWidth)) / 2
    T2BroadXpos = Xpos + widthBetweenRoads
    B2TroadXpos = Xpos - widthBetweenRoads
    T2Broad = VerticalRoad(T2BroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
    B2Troad = VerticalRoad(B2TroadXpos, endYpos, startYpos; numLanes=numLanes, laneWidth=laneWidth)
    return TwoWayVroad(T2Broad, B2Troad, medianWidth, Xpos)
end

signals(road::TwoWayVroad) = [road.B2Troad.signal, road.T2Broad.signal]
numlanes(road::TwoWayVroad) = road.B2Troad.numLanes
lanewidth(road::TwoWayVroad) = road.B2Troad.laneWidth
totalWidth(road::Union{TwoWayHroad,TwoWayVroad}) = (2 * numlanes(road) * lanewidth(road)) + road.medianWidth
startYPos(road::TwoWayVroad) = road.T2Broad.startPos[2]
endYPos(road::TwoWayVroad) = road.T2Broad.endPos[2]
topSpawnPos(road::TwoWayVroad) = road.T2Broad.spawnPos
bottomSpawnPos(road::TwoWayVroad) = road.B2Troad.spawnPos
left_boundary(road::TwoWayVroad) = left_boundary(road.B2Troad)
right_boundary(road::TwoWayVroad) = right_boundary(road.T2Broad)

mutable struct TwoWayIntersectingRoads <: Road
    leftRoadSeg::TwoWayHroad
    rightRoadSeg::TwoWayHroad
    topRoadSeg::TwoWayVroad
    bottomRoadSeg::TwoWayVroad
end

function TwoWayIntersectingRoads(Ypos, startXpos, endXpos, Xpos, startYpos, endYpos;
    vnumlanes=P.V_NUM_LANES,
    vlanewidth=P.V_LANE_WIDTH,
    vmedianwidth=P.V_MEDIAN_WIDTH,
    hnumlanes=P.H_NUM_LANES,
    hlanewidth=P.H_LANE_WIDTH,
    hmedianwidth=P.H_MEDIAN_WIDTH
)
    horizontalWidth = (2 * vnumlanes * vlanewidth) + vmedianwidth
    verticalWidth = (2 * hnumlanes * hlanewidth) + hmedianwidth
    leftRoadSeg = TwoWayHroad(Ypos, startXpos, Xpos - horizontalWidth / 2)
    rightRoadSeg = TwoWayHroad(Ypos, Xpos + horizontalWidth / 2, endXpos)
    topRoadSeg = TwoWayVroad(Xpos, startYpos, Ypos + verticalWidth / 2)
    bottomRoadSeg = TwoWayVroad(Xpos, Ypos - verticalWidth / 2, endYpos)
    return TwoWayIntersectingRoads(leftRoadSeg, rightRoadSeg, topRoadSeg, bottomRoadSeg)
end

roadsegs(road::TwoWayIntersectingRoads) = [road.leftRoadSeg, road.topRoadSeg, road.rightRoadSeg, road.bottomRoadSeg]
signals(road::TwoWayIntersectingRoads) = U.flatten(map(x->signals(x),roadsegs(road)))
leftSpawnPos(road::TwoWayIntersectingRoads) = leftSpawnPos(road.leftRoadSeg)
rightSpawnPos(road::TwoWayIntersectingRoads) = rightSpawnPos(road.rightRoadSeg)
topSpawnPos(road::TwoWayIntersectingRoads) = topSpawnPos(road.topRoadSeg)
bottomSpawnPos(road::TwoWayIntersectingRoads) = bottomSpawnPos(road.bottomRoadSeg)
spawnPos(road::TwoWayIntersectingRoads) = vcat(leftSpawnPos(road), topSpawnPos(road), rightSpawnPos(road), bottomSpawnPos(road))

abstract type CrossPath end
mutable struct HcrossPath <: CrossPath
    road::Union{HorizontalRoad,TwoWayHroad}
    xpos::Float64
end

mutable struct VcrossPath <: CrossPath
    road::Union{VerticalRoad, TwoWayVroad}
    ypos::Float64
end








