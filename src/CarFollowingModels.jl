# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


vel(agent) = U.magnitude(agent.vel)
Δv(this, prec) = vel(this) - vel(prec)

safeDistByVel(this) = P.S0_jam + (P.Safe_T * vel(this))
safeDistByDeltaVel(this, prec) = (vel(this) * Δv(this,prec)) / (2 * √(P.A_max * P.B_dec))
desiredDistance(this, prec) = safeDistByVel(this) + safeDistByDeltaVel(this, prec)

accTendency(this) =  1 - (vel(this)/ P.V0_pref)^4

# deceleration tendency when there is a preceding vehicle
decTendency(this::VehicleAgent, prec::VehicleAgent, model) = (desiredDistance(this, prec) / edistance(this, prec, model))^2
# deceleration tendency on a free road (no preceding vehicle)
decTendency(this::VehicleAgent, prec::Nothing, model) = 0

accelerationIDM(this, prec, model) = P.A_max * (accTendency(this) - decTendency(this,prec,model))

function computeIDMvelocity(this, model)
    acc = accelerationIDM(this, this.pv, model)
    return this.vel .+ (orientation(this) .* acc)
end

