module Jink

using Reexport
using JuMP
using OSQP
using LinearAlgebra

include("types.jl")
include("utils.jl")
include("solve_ik.jl")

export KinTask,
       JinkSolver

export initialize_solver,
       solve_ik

#= ############################
    Robot submodules 
=# ############################

# Digit submodule 
include("robots/digit/Digit.jl")
@reexport using .Digit
const digit = Digit
export digit

# PickleRick submodule
include("robots/picklerick/PickleRick.jl")
@reexport using .PickleRick
const picklerick = PickleRick
export picklerick

end
