# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
    type::Symbol
    orient::NTuple{2,Float64}
    pv:: Union{VehicleAgent, Nothing}
    ps:: Union{Signal, Nothing}
    tracked::Bool
    debugInfo::String
end

#Orientation should be part of VehicleAgent
#orientation(agent::VehicleAgent) = U.orientation(agent.vel)
#CLEAN UP
orientation(agent::VehicleAgent) = agent.orient
orientation(sp::SpawnPosition) = sp.orient

initial_speed(agent::VehicleAgent) = initial_speed(agent.type)
vehicle_width(agent::VehicleAgent) = vehicle_width(agent.type)
vehicle_length(agent::VehicleAgent) = vehicle_length(agent.type)
jam_dist_S0(agent::VehicleAgent) = jam_dist_S0(agent.type)
safe_time_T(agent::VehicleAgent) = safe_time_T(agent.type)
max_acc(agent::VehicleAgent) = max_acc(agent.type)
comfort_dec(agent::VehicleAgent) = comfort_dec(agent.type)
pref_vel_V0(agent::VehicleAgent) = pref_vel_V0(agent.type)
color(agent::VehicleAgent) = color(agent.type)

initial_speed(type::Symbol) =
if type === :car
    P.CAR_INITIAL_SPEED
elseif type === :truck 
    P.TRUCK_INITIAL_SPEED
elseif type === :mc 
    P.MC_INITIAL_SPEED
end

vehicle_width(type::Symbol) =
if type === :car 
    P.CAR_WIDTH
elseif type === :truck 
    P.TRUCK_WIDTH
elseif type === :mc 
    P.MC_WIDTH
end

vehicle_length(type::Symbol) =
if type === :car 
    P.CAR_LENGTH
elseif type === :truck 
    P.TRUCK_LENGTH
elseif type === :mc 
    P.MC_LENGTH
end

jam_dist_S0(type::Symbol) =
if type === :car 
    P.CAR_S0_jam
elseif type === :truck 
    P.TRUCK_S0_jam
elseif type === :mc 
    P.MC_S0_jam
end

safe_time_T(type::Symbol) =
if type === :car 
    P.CAR_Safe_T
elseif type === :truck 
    P.TRUCK_Safe_T
elseif type === :mc 
    P.MC_Safe_T
end

max_acc(type::Symbol) =
if type === :car 
    P.CAR_A_max
elseif type === :truck 
    P.TRUCK_A_max
elseif type === :mc 
    P.MC_A_max
end

comfort_dec(type::Symbol) =
if type === :car 
    P.CAR_B_dec
elseif type === :truck 
    P.TRUCK_B_dec
elseif type === :mc 
    P.MC_B_dec
end

pref_vel_V0(type::Symbol) =
if type === :car 
    P.CAR_V0_pref
elseif type === :truck 
    P.TRUCK_V0_pref
elseif type === :mc 
    P.MC_V0_pref
end

color(type::Symbol) =
if type === :car 
    P.CAR_COLOR
elseif type === :truck 
    P.TRUCK_COLOR
elseif type === :mc 
    P.MC_COLOR
end


isSameOrientation(agent1::VehicleAgent, agent2::VehicleAgent) = orientation(agent1) == orientation(agent2)
# TODO: clean this up (merge with Vehicle Agent maybe)
isSameOrientation(agent1::SpawnPosition, agent2::VehicleAgent) = orientation(agent1) == orientation(agent2)


function isSameLane(agent1::VehicleAgent, agent2::VehicleAgent) 
    isSameOrientation(agent1, agent2) || return false
    if orientation(agent1) == P.R2L_ORIENTATION || orientation(agent1) == P.L2R_ORIENTATION
        ## handle later for floating point equality
        agent2.pos[2] == agent1.pos[2] # if y is same
    elseif orientation(agent1) == P.B2T_ORIENTATION || orientation(agent1) == P.T2B_ORIENTATION
            ## handle later for floating point equality
        agent2.pos[1] == agent1.pos[1] # if x is same
    else
        false
    end
end

# TODO: clean this up (merge with Vehicle Agent maybe)
function isSameLane(agent1::SpawnPosition, agent2::VehicleAgent) 
    isSameOrientation(agent1, agent2) || return false
    if orientation(agent1) == P.R2L_ORIENTATION || orientation(agent1) == P.L2R_ORIENTATION
        ## handle later for floating point equality
        agent2.pos[2] == agent1.pos[2] # if y is same
    elseif orientation(agent1) == P.B2T_ORIENTATION || orientation(agent1) == P.T2B_ORIENTATION
            ## handle later for floating point equality
        agent2.pos[1] == agent1.pos[1] # if x is same
    else
        false
    end
end

function isPreceding(agent1::VehicleAgent, agent2::VehicleAgent)
    isSameLane(agent1, agent2) || return false
    if orientation(agent1) == P.L2R_ORIENTATION
        agent2.pos[1] > agent1.pos[1] # if x is more
    elseif orientation(agent1) == P.R2L_ORIENTATION
        agent2.pos[1] < agent1.pos[1] # if x is less
    elseif orientation(agent1) == P.B2T_ORIENTATION
        agent2.pos[2] > agent1.pos[2] # if y is more
    elseif orientation(agent1) == P.T2B_ORIENTATION
        agent2.pos[2] < agent1.pos[2] # if y is less
    else
        false
    end
end

# TODO: clean this up (merge with Vehicle Agent maybe)
function isPreceding(agent1::SpawnPosition, agent2::VehicleAgent)
    isSameLane(agent1, agent2) || return false
    if orientation(agent1) == P.L2R_ORIENTATION
        agent2.pos[1] > agent1.pos[1] # if x is more
    elseif orientation(agent1) == P.R2L_ORIENTATION
        agent2.pos[1] < agent1.pos[1] # if x is less
    elseif orientation(agent1) == P.B2T_ORIENTATION
        agent2.pos[2] > agent1.pos[2] # if y is more
    elseif orientation(agent1) == P.T2B_ORIENTATION
        agent2.pos[2] < agent1.pos[2] # if y is less
    else
        false
    end
end

function isSameRoad(agent1::VehicleAgent, sig::Signal)
    #TODO: currently we are only checking for orientation
    # later, we need to check if its exactly the same road
    orientation(agent1) == sig.orientation
end

function isPreceding(agent1::VehicleAgent, sig::Signal)
    isSameRoad(agent1, sig) || return false
    if orientation(agent1) == P.L2R_ORIENTATION
        sig.pos[1] > agent1.pos[1] # if x is more
    elseif orientation(agent1) == P.R2L_ORIENTATION
        sig.pos[1] < agent1.pos[1] # if x is less
    elseif orientation(agent1) == P.B2T_ORIENTATION
        sig.pos[2] > agent1.pos[2] # if y is more
    elseif orientation(agent1) == P.T2B_ORIENTATION
        sig.pos[2] < agent1.pos[2] # if y is less
    else
        false
    end
end

function vehicle_poly(v::VehicleAgent)
    hw = vehicle_width(v) / 2
    pt = Point2f[(0, -hw), (vehicle_length(v), -hw), (vehicle_length(v), hw), (0, hw)]
    return Polygon(pt)
end

function vehicle_marker(v::VehicleAgent)
    φ = atan(v.orient[2], v.orient[1]) 
    rotate2D(vehicle_poly(v), φ)
end

function vehicle_color(v::VehicleAgent)
    return v.tracked ? :brown : color(v)
end

function addDebugInfo(agent::VehicleAgent, debugInfo)
    P.DEBUG_MODE || return # only if debug mode is true
    # we may restrict this later for only tracked vehicle(s)
    agent.debugInfo *= (debugInfo * "\n")
end