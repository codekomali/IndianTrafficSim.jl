# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# module Vehicles

# export plot_vehicles!

using Agents
using InteractiveDynamics
using GLMakie
using Observables

include("Environment.jl")
include("Parameters.jl")
include("DrawEnvironment.jl")
include("AgentTypes.jl")
include("CarFollowingModels.jl")

import .Parameters as P

function t_add_vehicle!(model, horizontalRoad)
    add_vehicle!(horizontalRoad.spawnPos[1].pos .+ (250,0), model)
    initial_vel=(P.VEHICLE_INITIAL_SPEED, 0.0) .* 1.5
    add_vehicle!(horizontalRoad.spawnPos[1].pos, model, initial_vel)
    # add_vehicle!(horizontalRoad.spawnPos[1].pos .+ (500,0), model)
    # add_vehicle!(horizontalRoad.spawnPos[2].pos, model)
end

function initialize()
    space2d = ContinuousSpace((P.EXTENT_WIDTH, P.EXTENT_HEIGHT))
    properties = Dict()
    properties[:tick] = 0
    properties[:steptext] = Observable("Step: " * string(properties[:tick]))
    properties[:debugtext] = Observable("")
    #intersectingRoads = TwoWayIntersectingRoads(2000, 0, 4000, 2000, 4000, 0)
    horizontalRoadL2R = HorizontalRoad(2000, 0, 4000)
    properties[:env] = horizontalRoadL2R
    properties[:spawn_rate] = 1400
    properties[:tracked_agent] = -1
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly; properties=properties)
    t_add_vehicle!(model, model.env)
    return model
end


function add_vehicle!(spawn_pos, model, initial_vel=(P.VEHICLE_INITIAL_SPEED, 0.0), tracked = false)
    add_agent!(
        spawn_pos,
        model,
        initial_vel,
        nothing, # during creation no preceding vehicle
        tracked,
        ""
    )
end

# Generalize
function vehicle_poly()
    hw = P.VEHICLE_WIDTH / 2
    pt = Point2f[(0, -hw), (P.VEHICLE_LENGTH, -hw), (P.VEHICLE_LENGTH, hw), (0, hw)]
    return Polygon(pt)
end

function vehicle_marker(v::VehicleAgent)
    φ = atan(v.vel[2], v.vel[1]) 
    rotate2D(vehicle_poly(), φ)
end

function vehicle_color(v::VehicleAgent)
    return v.tracked ? :brown : :green
end

function nearest_agent(this_agent::VehicleAgent, others, model; 
    considerThresh = false,
    nearbyThresh = P.AGENT_NEARBY_THRESH)
    nearest_agent = nothing
    nearest_dist = Inf
    for o in others
        dist = edistance(this_agent,o, model)
        if dist < nearest_dist && (!considerThresh || dist <= nearbyThresh)
            nearest_dist = dist
            nearest_agent = o
        end
    end
    return nearest_agent
end

function nearest_agent(pos::NTuple{2, Float64}, others, model;
    considerThresh = false,
    nearbyThresh = P.AGENT_NEARBY_THRESH)
    nearest_agent = nothing
    nearest_dist = Inf
    for o in others
        #Note: A potential to contribute to Agents.jl
        # edistance should work for Pos<->Agent pair (currently not supported)
        dist = edistance(pos,o.pos, model)
        if dist < nearest_dist && (!considerThresh || dist <= nearbyThresh)
            nearest_dist = dist
            nearest_agent = o
        end
    end
    return nearest_agent
end

function preceding_vehicle(this_agent, model)
    nearby_agentset = Set(nearby_agents(this_agent, model, 300.0)) #Magic number
    preceding_agentset = filter(other -> isPreceding(this_agent, other), nearby_agentset)
    nearest_agent(this_agent, preceding_agentset, model)
end

function vehicle_step!(agent, model)
    agent.debugInfo = "" # resetting debugInfo after each step
    agent.pv = preceding_vehicle(agent, model)
    agent.vel = computeIDMvelocity(agent, model)
    move_agent!(agent, model)
end

function plot_environment!(model)
    custom_setup_environment!(model.env)
    drawRoad!(model.env)
end

# Move this to drawing the TWIR instead of here
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
    foreach(draw_cross_path!, cross_paths(twir))
end

function custom_setup_environment!(road)
    println("Nothing to customize!")
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

# Clean up move to draw or create draw_debug
function mark_spawn_positions(twir::TwoWayIntersectingRoads)
    for sp in spawnPos(twir)
        spawnPt = Point2f(sp.pos...)
        scatter!(spawnPt, color=:green, markersize=P.SIGNAL_MS)
    end
end

function debug_info(model)
    id = model.tracked_agent
    id != -1 || return "Not tracking"
    agent = model[id]
    pvdist = agent.pv !== nothing ? edistance(agent, agent.pv, model) : 0
    """
    ID = $(id)
    Vel = $(agent.vel)
    PV Dist = $(round(pvdist, digits=3))
    $(agent.debugInfo)
    """
end

function model_step!(model)
    model.tick += 1
    model.steptext[] = "Step: " * string(model.tick)
    dbinfo = debug_info(model)
    model.debugtext[] = dbinfo
    println(dbinfo)
    # if (model.tick % model.spawn_rate == 0)
    #     rnd_spawn_pos = rand(spawnPos(model.env))
    #     @show rnd_spawn_pos
    #     spawn_vel = rnd_spawn_pos.orient .* P.VEHICLE_INITIAL_SPEED
    #     @show spawn_vel
    #     add_vehicle!(rnd_spawn_pos.pos, model, spawn_vel)
    # end
    # draw_signal!(model.env)
end

function plot_vehicles!()
    model = initialize()
    axiskwargs = (title=P.PLOT_TITLE, titlealign=P.PLOT_TITLE_ALIGN)
    params = Dict(
        :spawn_rate => P.VSR_MIN:P.VSR_INC:P.VSR_MAX,
    )
    fig,ax, _ = abmplot(
        model;
        agent_step! = vehicle_step!,
        model_step! = model_step!,
        am=vehicle_marker,
        params=params,
        ac=vehicle_color,
        axis=axiskwargs
    )
    text!(model.steptext, position = (1000, 1000), textsize = 10)
    text!(model.debugtext, position = (1000, 3000), textsize = 10)
    plot_environment!(model)
    return (fig, ax, model)
end



#intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)

#a macro for easing development only. will be removed later
macro pv()
    quote
        fig, ax, model = plot_vehicles!()
        fig
    end
end

f,a,m = plot_vehicles!()

# on(events(f).mousebutton, priority = 0) do event
#     if event.button == Mouse.left
#         if event.action == Mouse.press
#             pos = mouseposition(f.scene)
#             println(pos)
#         else
#             # do something else when the mouse button is released
#         end
#     end
#     # Do not consume the event
#     return Consume(false)
# end

import GLMakie.register_interaction!

clicked_pos(p::Point{2, Float32}) = (p[1],p[2]) .|> Float64

function reset_tracking()
    if m.tracked_agent != -1
        m[m.tracked_agent].tracked = false
        m.tracked_agent = -1
    end
end

function track_agent(pos)
    @show pos
    reset_tracking()
    # TODO: nearby_ids is acting unreliably, explore further
    # and file an issue, till then use all_agents
    # ids = nearby_ids(pos, m, 40, exact=false) |> collect
    # @show ids
    # agents = map(id -> m[id],ids)
    agents = allagents(m) |> collect
    n_agent = nearest_agent(pos, agents, m, considerThresh=true)
    if n_agent !== nothing 
        n_agent.tracked = true
        m.tracked_agent = n_agent.id
    end
    println("tracking agent: ", m.tracked_agent)
end

register_interaction!(a, :my_interaction) do event::MouseEvent, axis
    if event.type === MouseEventTypes.leftclick
        #println("You clicked on the axis at datapos $(event.data)")
        event.data |> clicked_pos |> track_agent
    end
end


f
# end

