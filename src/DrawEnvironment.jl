# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

using Agents
using InteractiveDynamics
using GLMakie
using LinearAlgebra

include("Utils.jl")
include("Parameters.jl")

import .Parameters as P
import .Utils as U

#=
FUNCTIONS FOR DRAWING ON THE PLOT
=#
draw_line!(pos1, pos2; kwargs...) = lines!([pos1[1], pos2[1]], [pos1[2], pos2[2]]; kwargs...)

function draw_road_boundaries!(road::HorizontalRoad)
    draw_line!(
        top_boundary(road)...;
        color=P.ROAD_BOUNDARY_COLOR,
        linewidth=P.ROAD_BOUNDARY_LW
    )
    draw_line!(
        bottom_boundary(road)...;
        color=P.ROAD_BOUNDARY_COLOR,
        linewidth=P.ROAD_BOUNDARY_LW
    )
    return nothing
end

function draw_cross_path!(cp::HcrossPath)
    topy = top_boundary(cp.road)[1][2]
    boty = bottom_boundary(cp.road)[1][2]
    hw = P.CROSS_PATH_WIDTH /2
    line1 = ((cp.xpos - hw, topy), (cp.xpos - hw, boty))
    line2 = ((cp.xpos + hw, topy), (cp.xpos + hw, boty))
    for line in [line1, line2]
        draw_line!(
            line...;
            color=P.CP_COLOR,
            linewidth=P.CP_LW
        )
    end
end

function draw_cross_path!(cp::VcrossPath)
    leftx = left_boundary(cp.road)[1][1]
    rightx = right_boundary(cp.road)[1][1]
    hw = P.CROSS_PATH_WIDTH /2
    line1 = ((leftx, cp.ypos - hw), (rightx, cp.ypos - hw))
    line2 = ((leftx, cp.ypos + hw), (rightx, cp.ypos + hw))
    for line in [line1, line2]
        draw_line!(
            line...;
            color=P.CP_COLOR,
            linewidth=P.CP_LW
        )
    end
end

function draw_road_boundaries!(road::VerticalRoad)
    draw_line!(
        right_boundary(road)...;
        color=P.ROAD_BOUNDARY_COLOR,
        linewidth=P.ROAD_BOUNDARY_LW
    )
    draw_line!(
        left_boundary(road)...;
        color=P.ROAD_BOUNDARY_COLOR,
        linewidth=P.ROAD_BOUNDARY_LW
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
        last_boundary = U.reduce_y(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=P.LANE_MARKER_COLOR, linestyle=P.LANE_MARKER_LS)
    end
    return nothing
end

function draw_lane_markers!(road::VerticalRoad)
    last_boundary = right_boundary(road)
    for _ in 2:road.numLanes
        last_boundary = U.reduce_x(last_boundary, road.laneWidth)
        draw_line!(last_boundary..., color=P.LANE_MARKER_COLOR, linestyle=P.LANE_MARKER_LS)
    end
    return nothing
end

function draw_pedestrian_walkway!(road::HorizontalRoad)
    pedWay = nothing
    if road.startPos[1] < road.endPos[1]
        pedWay = U.increase_y(top_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    else
        pedWay = U.reduce_y(bottom_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    end
    draw_line!(pedWay..., color=P.PW_COLOR)
end

function draw_pedestrian_walkway!(road::VerticalRoad)
    pedWay = nothing
    if road.startPos[2] > road.endPos[2]
        pedWay = U.increase_x(right_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    else
        pedWay = U.reduce_x(left_boundary(road), P.PEDESTRIAN_WALKWAY_WIDTH)    
    end
    draw_line!(pedWay..., color=P.PW_COLOR)
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
    draw_line!(startPt, endPt, color=P.MEDIAN_COLOR, linewidth=road.medianWidth)
    return nothing
end

function drawMedian!(road::TwoWayVroad)
    startPt = (road.Xpos, startYPos(road))
    endPt = (road.Xpos, endYPos(road))
    draw_line!(startPt, endPt, color=P.MEDIAN_COLOR, linewidth=road.medianWidth)
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
