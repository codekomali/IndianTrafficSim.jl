# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

mutable struct VehicleAgent <: AbstractAgent
    id::Int
    pos::NTuple{2,Float64}
    vel::NTuple{2,Float64}
end

orientation(agent::VehicleAgent) = U.orientation(agent.vel)
isSameOrientation(agent1::VehicleAgent, agent2::VehicleAgent) = orientation(agent1) == orientation(agent2)

function isSameLane(agent1::VehicleAgent, agent2::VehicleAgent) 
    isSameOrientation(agent1, agent2) || return false
    if orientation(agent1) == P.R2L_ORIENTATION
        ## handle later for floating point equality
        agent2.pos[2] == agent1.pos[2] # if y is same
    else
        false
    end
end

function isPreceding(agent1::VehicleAgent, agent2::VehicleAgent)
    isSameLane(agent1, agent2) || return false
    if orientation(agent1) == P.R2L_ORIENTATION
        agent2.pos[1] > agent1.pos[1] # if x is more
    else
        false
    end
end
