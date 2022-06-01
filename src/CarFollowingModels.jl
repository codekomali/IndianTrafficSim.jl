# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# TODO: Move 'vel' to appropriate place (also maybe rename to speed)
vel(agent::VehicleAgent) = U.magnitude(agent.vel) |> abs
vel(s::Signal) = 0

function Δv(this, prec) 
    dv = vel(this) - vel(prec)
    addDebugInfo(this, "DV: $(dv)")
    return dv
end

function safeDistByVel(this) 
   sdv = jam_dist_S0(this) + (safe_time_T(this) * vel(this))
   addDebugInfo(this, "SDV: $(sdv)")
   return sdv
end

function safeDistByDeltaVel(this, prec) 
    sddv = (vel(this) * Δv(this,prec)) / (2 * √(max_acc(this) * comfort_dec(this)))
    addDebugInfo(this, "SDDV: $(sddv)")
    return sddv
end

function desiredDistance(this, prec) 
    dd = safeDistByVel(this) + safeDistByDeltaVel(this, prec)
    addDebugInfo(this, "DD: $(dd)")
    return dd
end

accTendency(this) =  1 - (vel(this)/ pref_vel_V0(this))^4

# the real distance between vehicle and other things (subtracting vehicle distance)
# TODO: Change Name and move as appropriate
realdistance(this::VehicleAgent, prec::VehicleAgent) = U.euc_dist(this.pos, prec.pos) - vehicle_length(this)

# TODO: Change Name and move as appropriate
function realdistance(this::VehicleAgent, sig::Signal)
    if isapprox(this.orient, P.L2R_ORIENTATION) || isapprox(this.orient, P.R2L_ORIENTATION)
        abs(sig.pos[1] - this.pos[1]) 
    elseif isapprox(this.orient, P.T2B_ORIENTATION) || isapprox(this.orient, P.B2T_ORIENTATION)
        abs(sig.pos[2] - this.pos[2]) 
    end
end

# deceleration tendency when there is a preceding vehicle
decTendency(this::VehicleAgent, prec::VehicleAgent) = (desiredDistance(this, prec) / realdistance(this, prec))^2
# deceleration tendency on a free road (no preceding vehicle)
decTendency(this::VehicleAgent, prec::Nothing) = 0
# deceleration tendency when there is a preceding signal
# TODO: Merge with decTendency definition of VehicleAgent (maybe Union)
decTendency(this::VehicleAgent, prec::Signal) = (desiredDistance(this, prec) / realdistance(this, prec))^2

function accelerationIDM(this, prec, model) 
    accT = accTendency(this)
    addDebugInfo(this, "Acc Tend: $(accT)")
    decT = decTendency(this, prec)
    addDebugInfo(this, "Dec Tend: $(decT)")
    return max_acc(this) * (accT - decT)
end

function computeIDMvelocity(this, model)
    addDebugInfo(this, "\nPV Comp:")
    pvdist = this.pv === nothing ? 0 : realdistance(this, this.pv)
    addDebugInfo(this, "PV Dist $(round(pvdist,digits=3))")
    acc = accelerationIDM(this, this.pv, model)
    addDebugInfo(this, "PV acc: $(acc)")
    # TODO: CAN WE MOVE STATE TO PRECEDING_SIGNAL ITSELF and SIMPLIFY THIS
    if this.ps !== nothing && (this.ps.state == :red || this.ps.state == :yellow)
        addDebugInfo(this, "\nPSComp:")
        psdist = realdistance(this, this.ps)
        addDebugInfo(this, "PS Dist $(round(psdist,digits=3))")
        ps_acc = accelerationIDM(this, this.ps, model)
        addDebugInfo(this, "PS acc: $(ps_acc)")
        acc = acc < ps_acc ? acc : ps_acc
    end
    res_vel = this.vel .+ (orientation(this) .* acc)
    # This is just to ensure that we never just go in reverse
    res_vel = isapprox(U.orientation(res_vel), orientation(this)) ? res_vel : (0,0)
    addDebugInfo(this, "\nRES VEL: $(res_vel)")
    return res_vel
end


# TODO: Later implement improved version of IDM called (IIDM)

