# Copyright (c) 2022 Code Komali
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

module Utils

using Agents
using InteractiveDynamics
using GLMakie
using LinearAlgebra

flatten(x) = reduce(vcat, x)

reduce_y(line, val) = (line[1] .- (0, val), line[2] .- (0, val))

increase_y(line,val)= (line[1] .+ (0, val), line[2] .+ (0, val))

reduce_x(line, val) = (line[1] .- (val, 0), line[2] .- (val, 0))

increase_x(line, val) = (line[1] .+ (val, 0), line[2] .+ (val, 0))

# RCL: removed rounding because causing issue 29 May 2022
toVectorPos(tuple::NTuple{2,Float64})= [ x for x in tuple ]

toVectorPos(tuple::NTuple{2,Int64})= [ x for x in tuple ]

magnitude(tuple::NTuple{2,Float64}) = √(tuple[1]^2 + tuple[2]^2)

# RCL: removed rounding because causing issue 29 May 2022
function toTuplePos(vector::Vector{Float64})
    length(vector)==2 || return error("Position vector has more than 2 dims")
    return NTuple{2, Float64}(i for i in vector)
end

orientation(startPos, endPos) = endPos .- startPos |> toVectorPos |> normalize |> toTuplePos
orientation(val::NTuple{2,Float64}) = val |> toVectorPos |> normalize |> toTuplePos

import Base.isapprox
# TODO: use isapprox where there is a orientation comparison
isapprox(val1::NTuple{2,Float64}, val2::NTuple{2,Float64}) = isapprox(val1[1],val2[1]) && isapprox(val1[2], val2[2])
euc_dist(p1,p2)=√((p2[1]-p1[1])^2 + (p2[2]-p1[2])^2)


# Conversion Utilities

# 1 m/s2 =   0.000001 m/ms2
ms2_to_mms2(ms2) = ms2 * 0.000001

# 1 m/ms2 = 1600 m/tick2 (because 1 tick = 40 ms)
mms2_to_mt2(mms2) = mms2 * 1600

# 1 m/t2 = 13.5 u/t2 (as 1m = 13.5u)
mt2_to_ut2(mt2) = mt2 * 13.5

# conversion from m/s2 to u/tick2 
ms2_to_ut2(ms2) = ms2 |> ms2_to_mms2 |> mms2_to_mt2 |> mt2_to_ut2

# KMPH to MPS divide by 3.6
kmph_to_mps(kmph) = kmph / 3.6 

# M/S to M/MS divide by 1000
mps_to_mpms(mps) = mps / 1000

# M/MS to M/tick multiply by 40 (1 tick to 40 ms)
mpms_to_mpt(mpms) = mpms * 40

# M/tick to units/tick multiply by 13.5 (1 m = 13.5 units)
mpt_to_upt(mpt) = mpt * 13.5

mps_to_upt(mps) = mps|> mps_to_mpms |> mpms_to_mpt |> mpt_to_upt

kmph_to_upt(kmph) = kmph |> kmph_to_mps |> mps_to_upt


end