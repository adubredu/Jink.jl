module Jink

using Reexport
using JuMP
using OSQP

include("types.jl")



#= 
    Robot submodules 
=#
# Digit submodule 
include("robots/digit/Digit.jl")
@reexport using .Digit
const digit = Digit
export digit

end
