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

euc_dist(p1,p2)=√( (p2[1]-p1[1])^2 + (p2[2]-p2[2])^2 ) 

end