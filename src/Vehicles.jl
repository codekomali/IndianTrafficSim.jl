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

function t_add_vehicles!(model, spawnPos)
    car_spawn_vel = spawnPos.orient .* initial_speed(:car)
    spawn_pos1 = spawnPos.pos .+ (spawnPos.orient .* 500)
    add_vehicle!(spawn_pos1, model, car_spawn_vel, :car)
    truck_spawn_vel = spawnPos.orient .* initial_speed(:truck)
    spawn_pos2 = spawnPos.pos .+ (spawnPos.orient .* 1)
    add_vehicle!(spawn_pos2, model, truck_spawn_vel, :truck)
end

function t_add_vehicle!(model)
    foreach(spawnPos(model.env)) do sp
        t_add_vehicles!(model, sp)
    end
end

function spawn_vehicle!(model)
    rnd_spawn_pos = rand(spawnPos(model.env))
    rnd_type = rand([:car, :truck, :mc])
    sds = safeDistToSpawn(rnd_type)
    spv = preceding_vehicle(rnd_spawn_pos, model)
    if (spv === nothing || realdistance(rnd_spawn_pos, spv, rnd_type) > sds)
        println("Spawning $(rnd_type) at position $(rnd_spawn_pos.pos)")
        add_vehicle!(rnd_spawn_pos.pos, model, rnd_spawn_pos.orient .* initial_speed(rnd_type), rnd_type)
    end
end

function spawn_pedestrians!(model)
    println(" in spawn pedestrian")
    rnd_spawn_pos = rand(pedestrian_spawn_pos(model.cross_paths))
    ped = Pedestrian(length(model.pedestrians), rnd_spawn_pos)
    println("Spawning pedestrian at position $(ped.pos)")
    println(model.pedestrians)
    push!(model.pedestrians, ped)
    println(" done spawn pedestrian")
end

function move_pedestrians!(model)
    for ped in model.pedestrians
        ped.pos[] = ped.pos[] .+ ped.vel
    end
end

function kill_pedestrians!(model)
    peds = Vector{Pedestrian}()
    for ped in model.pedestrians
        # keep only pedestrians who are more than 5.0 away from dest
        if U.euc_dist(ped.pos[].data, ped.dest) > 5.0
            push!(peds, ped)
        else
            delete!(m.axis, ped.scatter)
        end
    end
    model.pedestrians = peds
end


function initialize()
    space2d = ContinuousSpace((P.EXTENT_WIDTH, P.EXTENT_HEIGHT))
    properties = Dict()
    properties[:tick] = 0
    properties[:steptext] = Observable("Step: " * string(properties[:tick]))
    properties[:debugtext] = Observable("")
    properties[:trackpt] = Observable(Point2f((0, 0)))
    properties[:axis] = nothing
    # MAKE THE BELOW TYPE AWARE
    properties[:cross_paths] = Vector{CrossPath}()
    properties[:pedestrians] = Vector{Pedestrian}()
    intersectingRoads = TwoWayIntersectingRoads(3380 / 2, 0, 3380, 3380 / 2, 3380, 0)
    #horizontalRoadL2R = HorizontalRoad(2000, 0, 3380)
    horizontalRoadR2L = HorizontalRoad(2000, 3380, 0)
    verticalRoadT2B = VerticalRoad(1690, 3380, 0)
    verticalRoadB2T = VerticalRoad(1690, 0, 3380)
    properties[:env] = intersectingRoads
    properties[:spawn_rate] = 1400
    properties[:tracked_agent] = -1
    model = ABM(VehicleAgent, space2d, scheduler=Schedulers.randomly; properties=properties)
    #t_add_vehicle!(model)
    spawn_vehicle!(model)
    return model
end


function add_vehicle!(spawn_pos, model, initial_vel=(P.VEHICLE_INITIAL_SPEED, 0.0), type=:car, tracked=false)
    add_agent!(
        spawn_pos,
        model,
        initial_vel,
        type,
        U.orientation(initial_vel),
        nothing, # during creation no preceding vehicle
        nothing, # during creation no preceding signal
        tracked,
        "" # empty debug info
    )
end

function nearest_agent(this_agent::VehicleAgent, others, model;
    considerThresh=false,
    nearbyThresh=P.AGENT_NEARBY_THRESH)
    nearest_agent = nothing
    nearest_dist = Inf
    for o in others
        dist = edistance(this_agent, o, model)
        if dist < nearest_dist && (!considerThresh || dist <= nearbyThresh)
            nearest_dist = dist
            nearest_agent = o
        end
    end
    return nearest_agent
end

function nearest_agent(pos::NTuple{2,Float64}, others, model;
    considerThresh=false,
    nearbyThresh=P.AGENT_NEARBY_THRESH)
    nearest_agent = nothing
    nearest_dist = Inf
    for o in others
        #Note: A potential to contribute to Agents.jl
        # edistance should work for Pos<->Agent pair (currently not supported)
        dist = edistance(pos, o.pos, model)
        if dist < nearest_dist && (!considerThresh || dist <= nearbyThresh)
            nearest_dist = dist
            nearest_agent = o
        end
    end
    return nearest_agent
end


function nearest_signal(pos::NTuple{2,Float64}, signals, model;
    considerThresh=false,
    nearbyThresh=P.AGENT_NEARBY_THRESH)
    nearest_sig = nothing
    nearest_dist = Inf
    for sig in signals
        #Note: A potential to contribute to Agents.jl
        # edistance should work for Pos<->Agent pair (currently not supported)
        # TODO: later, may be edistance will work
        dist = U.euc_dist(pos, sig.pos)
        if dist < nearest_dist && (!considerThresh || dist <= nearbyThresh)
            nearest_dist = dist
            nearest_sig = sig
        end
    end
    return nearest_sig
end

function nearest_signal(v::VehicleAgent, signals, model; kwargs...)
    nearest_signal(v.pos, signals, model, kwargs...)
end

function preceding_vehicle(this_agent::VehicleAgent, model)
    nearby_agentset = Set(nearby_agents(this_agent, model, 300.0)) #Magic number
    preceding_agentset = filter(other -> isPreceding(this_agent, other), nearby_agentset)
    nearest_agent(this_agent, preceding_agentset, model)
end

function preceding_vehicle(this_pos::SpawnPosition, model)
    preceding_agentset = filter(other -> isPreceding(this_pos, other), collect(allagents(model)))
    nearest_agent(this_pos.pos, preceding_agentset, model)
end

function nearby_signals(this_agent, model, dist_thresh=675) # ≈ 50 meters (magic number)
    #TODO: better way is to import and specialize edistance from Agents.jl
    filter(signal -> signal.active && realdistance(this_agent, signal) <= dist_thresh, signals(model.env))
end

function preceding_signal(this_agent, model)
    signals = nearby_signals(this_agent, model, 300.0)
    preceding_signal = filter(other -> isPreceding(this_agent, other), signals)
    nearest_signal(this_agent, preceding_signal, model)
end

function is_within_extent(pos)
    x = pos[1]
    y = pos[2]
    x_within_extent(x) = x >= 0 && x <= P.EXTENT_WIDTH
    y_within_extent(y) = y >= 0 && y <= P.EXTENT_HEIGHT
    return x_within_extent(x) && y_within_extent(y)
end

function vehicle_step!(agent, model)
    agent.debugInfo = "" # resetting debugInfo after each step
    agent.pv = preceding_vehicle(agent, model)
    agent.ps = preceding_signal(agent, model)
    agent.vel = computeIDMvelocity(agent, model)
    next_pos = agent.pos .+ agent.vel
    if is_within_extent(next_pos)
        move_agent!(agent, model)
    else
        kill_agent!(agent, model)
    end
end

function plot_environment!(model)
    custom_setup_environment!(model.env, model)
    drawRoad!(model.env)
end

# Move this to drawing the TWIR instead of here
function custom_setup_environment!(twir::TwoWayIntersectingRoads, model::AgentBasedModel)
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
    model.cross_paths = cross_paths(twir)
    foreach(draw_cross_path!, model.cross_paths)
end

function pedestrian_spawn_pos(cps::Vector{CrossPath})
    reduce(vcat, map(pedestrian_spawn_pos, cps))
end

function pedestrian_spawn_pos(cp::HcrossPath)
    topy = top_boundary(cp.road)[1][2]
    boty = bottom_boundary(cp.road)[1][2]
    return [
        ((cp.xpos, topy + 10), (cp.xpos, boty - 10)),
        ((cp.xpos, boty - 10), (cp.xpos, topy + 10))
    ]
end

function pedestrian_spawn_pos(cp::VcrossPath)
    leftx = left_boundary(cp.road)[1][1]
    rightx = right_boundary(cp.road)[1][1]
    return [
        ((leftx - 10, cp.ypos), (rightx + 10, cp.ypos)),
        ((rightx + 10, cp.ypos), (leftx - 10, cp.ypos))
    ]
end

function custom_setup_environment!(road, model)
    println("Nothing to customize!")
end

function cross_paths(twir::TwoWayIntersectingRoads)
    cps = Vector{CrossPath}()
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

# Clean up: move to draw or create draw_debug
function mark_spawn_positions(twir::TwoWayIntersectingRoads)
    for sp in spawnPos(twir)
        spawnPt = Point2f(sp.pos...)
        scatter!(spawnPt, color=:green, markersize=P.SIGNAL_MS)
    end
end

function debug_info(model)
    id = model.tracked_agent
    id != -1 || return "Not tracking"
    agent = get(m.agents, id, nothing)
    if agent !== nothing
        model.trackpt[] = agent.pos
        """
        ID = $(id)
        Vel = $(agent.vel)
        Pos = $(agent.pos)
        $(agent.debugInfo)
        """
    else
        model.trackpt[] = (0.0, 0.0)
        """
        agent $(id) does not exist. probably dead!
        """
    end
end

function pedestrian_step!(model)
    kill_pedestrians!(model)
    if length(model.pedestrians) < 1
        spawn_pedestrians!(model)
    end
    move_pedestrians!(model)
end


function model_step!(model)
    model.tick += 1
    model.steptext[] = "Step: " * string(model.tick)
    dbinfo = debug_info(model)
    model.debugtext[] = dbinfo
    println("Tick: $(model.tick)")
    println(dbinfo)
    pedestrian_step!(model)
    spawn_vehicle!(model)
    draw_signal!(model.env)
end

function vehicle_poly1()
    hw = P.CAR_WIDTH / 2
    pt = Point2f[(0, -hw), (P.CAR_LENGTH, -hw), (P.CAR_LENGTH, hw), (0, hw)]
    return Polygon(pt)
end

function vehicle_marker1(v::VehicleAgent)
    φ = atan(v.vel[2], v.vel[1])
    rotate2D(vehicle_poly1(), φ)
end

ac(a) = a.type == :car ? "#2b2b33" : a.type == :truck ? "#bf2642" : "#338c54"

function plot_vehicles!()
    model = initialize()
    axiskwargs = (title=P.PLOT_TITLE, titlealign=P.PLOT_TITLE_ALIGN)
    params = Dict(
        :spawn_rate => P.VSR_MIN:P.VSR_INC:P.VSR_MAX,
    )
    fig, ax, _ = abmplot(
        model;
        (agent_step!)=vehicle_step!,
        (model_step!)=model_step!,
        am=vehicle_marker,
        params=params,
        # TODO: File bug with InteractiveDynamics.jl team that function for ac is broken
        # especially if you spawn new agents during model evolution
        ac=:green,
        axis=axiskwargs
    )
    model.axis = ax
    text!(model.steptext, position=(1000, 1000), textsize=10)
    text!(model.debugtext, position=(1000, 3000), textsize=10)
    plot_environment!(model)
    scatter!(model.trackpt, color=:red, markersize=5)
    return (fig, ax, model)
end

# intersectingRoads = TwoWayIntersectingRoads(2000,0,4000,2000,4000,0)

#a macro for easing development only. will be removed later
macro pv()
    quote
        fig, ax, model = plot_vehicles!()
        fig
    end
end

f, a, m = plot_vehicles!()

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

clicked_pos(p::Point{2,Float32}) = (p[1], p[2]) .|> Float64

function reset_tracking()
    if m.tracked_agent != -1 && haskey(m.agents, m.tracked_agent)
        m[m.tracked_agent].tracked = false
        m.tracked_agent = -1
        m.trackpt[] = (0.0, 0.0)
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

