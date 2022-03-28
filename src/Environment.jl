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

abstract type Road end

half_width(road::Road) = road.numLanes * road.laneWidth / 2
half_width(numLanes, laneWidth) = numLanes * laneWidth / 2

struct HorizontalRoad <: Road
    startPos::NTuple{2,Float64}
    endPos::NTuple{2,Float64}
    numLanes::Int64
    laneWidth::Float64
    spawnPos::Vector{NTuple{2,Float64}}
    function HorizontalRoad(Ypos, startXpos, endXpos; numLanes=2, laneWidth=50)
        startPos = (startXpos, Ypos)
        endPos = (endXpos, Ypos)
        lanemid = (Ypos - half_width(numLanes, laneWidth)) + laneWidth/2
        spawnPos = []
        for _ in 1:numLanes
            spawnPos = push!(spawnPos,(startXpos, lanemid))
            lanemid += laneWidth
        end
        return new(startPos, endPos, numLanes, laneWidth, spawnPos)
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
    function VerticalRoad(Xpos, startYpos, endYpos; numLanes=2, laneWidth=50)
        startPos = (Xpos, startYpos)
        endPos = (Xpos, endYpos)
        lanemid = (Xpos - half_width(numLanes, laneWidth)) + laneWidth/2
        spawnPos = []
        for _ in 1:numLanes
            spawnPos = push!(spawnPos,(lanemid, startYpos))
            lanemid += laneWidth
        end 
        return new(startPos, endPos, numLanes, laneWidth, spawnPos)
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
        R2Lroad = HorizontalRoad(R2LroadYpos, startXpos, endXpos; numLanes=numLanes, laneWidth=laneWidth)
        return new(L2Rroad, R2Lroad, medianWidth, Ypos)
    end
end

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
        B2Troad = VerticalRoad(B2TroadXpos, startYpos, endYpos; numLanes=numLanes, laneWidth=laneWidth)
        return new(T2Broad, B2Troad, medianWidth, Xpos)
    end
end

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
        vlanewidth = 50,
        vmedianwidth =2,
        hnumlanes=2,
        hlanewidth = 50,
        hmedianwidth = 2
        )
        horizontalWidth = (2 * vnumlanes * vlanewidth) + vmedianwidth
        verticalWidth = (2 * hnumlanes * hlanewidth) + hmedianwidth
        leftRoadSeg = TwoWayHroad(Ypos, startXpos, Xpos - horizontalWidth/2)
        rightRoadSeg = TwoWayHroad(Ypos, Xpos + horizontalWidth/2,endXpos)
        topRoadSeg = TwoWayVroad(Xpos, startYpos, Ypos + verticalWidth/2)
        bottomRoadSeg = TwoWayVroad(Xpos, Ypos - verticalWidth/2, endYpos)
        return new(leftRoadSeg, rightRoadSeg, topRoadSeg, bottomRoadSeg)
    end 
end

leftSpawnPos(road::TwoWayIntersectingRoads) = leftSpawnPos(road.leftRoadSeg)
rightSpawnPos(road::TwoWayIntersectingRoads) = rightSpawnPos(road.rightRoadSeg)
topSpawnPos(road::TwoWayIntersectingRoads) = topSpawnPos(road.topRoadSeg)
bottomSpawnPos(road::TwoWayIntersectingRoads) = bottomSpawnPos(road.bottomRoadSeg)

draw_line!(pos1, pos2; kwargs...)= lines!([pos1[1], pos2[1]], [pos1[2], pos2[2]]; kwargs...)
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

function drawRoad!(interRoad::TwoWayIntersectingRoads)
    drawRoad!(interRoad.leftRoadSeg)
    drawRoad!(interRoad.topRoadSeg)
    drawRoad!(interRoad.rightRoadSeg)
    drawRoad!(interRoad.bottomRoadSeg)
end

#intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)

#plot_environment!()

#end

# using .Environment

#plot_environment!()




