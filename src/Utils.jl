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

toVectorPos(tuple::NTuple{2,Float64})= [ round(x, digits=2) for x in tuple ]

toVectorPos(tuple::NTuple{2,Int64})= [ x for x in tuple ]

magnitude(tuple::NTuple{2,Float64}) = √(tuple[1]^2 + tuple[2]^2)

function toTuplePos(vector::Vector{Float64})
    length(vector)==2 || return error("Position vector has more than 2 dims")
    return NTuple{2, Float64}(round(i, digits=2) for i in vector)
end

orientation(startPos, endPos) = endPos .- startPos |> toVectorPos |> normalize |> toTuplePos
orientation(vel::NTuple{2,Float64}) = vel |> toVectorPos |> normalize |> toTuplePos

end