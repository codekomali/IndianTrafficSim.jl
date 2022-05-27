# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


vel(agent) = U.magnitude(agent.vel)
function Δv(this, prec) 
    dv = vel(this) - vel(prec)
    addDebugInfo(this, "DV: $(dv)")
    return dv
end

function safeDistByVel(this) 
   sdv = P.S0_jam + (P.Safe_T * vel(this))
   addDebugInfo(this, "SDV: $(sdv)")
   return sdv
end

function safeDistByDeltaVel(this, prec) 
    sddv = (vel(this) * Δv(this,prec)) / (2 * √(P.A_max * P.B_dec))
    addDebugInfo(this, "SDDV: $(sddv)")
    return sddv
end

desiredDistance(this, prec) = safeDistByVel(this) + safeDistByDeltaVel(this, prec)

accTendency(this) =  1 - (vel(this)/ P.V0_pref)^4

# deceleration tendency when there is a preceding vehicle
decTendency(this::VehicleAgent, prec::VehicleAgent, model) = (desiredDistance(this, prec) / edistance(this, prec, model))^2
# deceleration tendency on a free road (no preceding vehicle)
decTendency(this::VehicleAgent, prec::Nothing, model) = 0

function accelerationIDM(this, prec, model) 
    accT = accTendency(this)
    addDebugInfo(this, "Acc Tend: $(accT)")
    decT = decTendency(this, prec, model)
    addDebugInfo(this, "Dec Tend: $(decT)")
    return P.A_max * (accT - decT)
end

function computeIDMvelocity(this, model)
    acc = accelerationIDM(this, this.pv, model)
    return this.vel .+ (orientation(this) .* acc)
end

# Later implement improved version of IDM called (IIDM)

