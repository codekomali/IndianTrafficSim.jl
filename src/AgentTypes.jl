# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
    pv:: Union{VehicleAgent, Nothing}
    tracked::Bool
    debugInfo::String
end

orientation(agent::VehicleAgent) = U.orientation(agent.vel)
isSameOrientation(agent1::VehicleAgent, agent2::VehicleAgent) = orientation(agent1) == orientation(agent2)

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

function isPreceding(agent1::VehicleAgent, agent2::VehicleAgent)
    isSameLane(agent1, agent2) || return false
    if orientation(agent1) == P.R2L_ORIENTATION
        agent2.pos[1] > agent1.pos[1] # if x is more
    elseif orientation(agent1) == P.L2R_ORIENTATION
        agent2.pos[1] < agent1.pos[1] # if x is less
    elseif orientation(agent1) == P.B2T_ORIENTATION
        agent2.pos[2] > agent1.pos[2] # if y is more
    elseif orientation(agent1) == P.T2B_ORIENTATION
        agent2.pos[2] < agent1.pos[2] # if y is less
    else
        false
    end
end

function addDebugInfo(agent::VehicleAgent, debugInfo)
    P.DEBUG_MODE || return # only if debug mode is true
    # we may restrict this later for only tracked vehicle(s)
    agent.debugInfo *= (debugInfo * "\n")
end