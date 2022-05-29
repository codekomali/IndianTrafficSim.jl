# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


vel(agent::VehicleAgent) = U.magnitude(agent.vel) |> abs
vel(s::Signal) = 0

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

function desiredDistance(this, prec) 
    dd = safeDistByVel(this) + safeDistByDeltaVel(this, prec)
    addDebugInfo(this, "DD: $(dd)")
    return dd
end

accTendency(this) =  1 - (vel(this)/ P.V0_pref)^4

# the real distance between vehicle and other things (subtracting vehicle distance)
realdistance(vpos, pos) = U.euc_dist(vpos, pos) - P.VEHICLE_LENGTH

# deceleration tendency when there is a preceding vehicle
decTendency(this::VehicleAgent, prec::VehicleAgent) = (desiredDistance(this, prec) / realdistance(this.pos, prec.pos))^2
# deceleration tendency on a free road (no preceding vehicle)
decTendency(this::VehicleAgent, prec::Nothing) = 0
# deceleration tendency when there is a preceding signal
decTendency(this::VehicleAgent, prec::Signal) = (desiredDistance(this, prec) / realdistance(this.pos, prec.pos))^2

function accelerationIDM(this, prec, model) 
    accT = accTendency(this)
    addDebugInfo(this, "Acc Tend: $(accT)")
    decT = decTendency(this, prec)
    addDebugInfo(this, "Dec Tend: $(decT)")
    return P.A_max * (accT - decT)
end

function computeIDMvelocity(this, model)
    acc = accelerationIDM(this, this.pv, model)
    if this.ps !== nothing && (this.ps.state == :red || this.ps.state == :yellow)
        ps_acc = accelerationIDM(this, this.ps, model)
        addDebugInfo(this, "PS acc: $(ps_acc)")
        acc = acc < ps_acc ? acc : ps_acc
    end
    res_vel = this.vel .+ (orientation(this) .* acc)
    # This is just to ensure that we never just go in reverse
    res_vel = isapprox(U.orientation(res_vel), orientation(this)) ? res_vel : (0,0)
    addDebugInfo(this, "RES VEL: $(res_vel)")
    return res_vel
end


# Later implement improved version of IDM called (IIDM)

