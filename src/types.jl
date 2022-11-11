mutable struct SE3
    position::Union{Nothing, AbstractArray}
    orientation::Union{Nothing, Matrix}

    function SE3()
        new(nothing, nothing)
    end
end

mutable struct KinTask 
    name::Union{Nothing, Symbol}
    target::Union{Nothing, SE3, Vector}
    task_map::Union{Nothing, Function}
    task_map_jacobian::Union{Nothing, Function}
    weight::Union{Nothing, AbstractArray, Float64}
    selection_matrix::Union{Nothing, Matrix} 

    function KinTask()
        new(nothing, nothing, nothing, 
            nothing, nothing, nothing)
    end
end

# mutable struct PostureTask 
#     name::Union{Nothing, Symbol}
#     target::Union{Nothing, Vector}
#     task_map::Union{Nothing, Function}
#     task_map_jacobian::Union{Nothing, Function}
#     weight::Union{Nothing, AbstractArray, Float64}
#     selection_matrix::Union{Nothing, Matrix} 

#     function PostureTask()
#         new(nothing, nothing, nothing, 
#             nothing, nothing, nothing)
#     end
# end

mutable struct JinkSolver
    model::Model
    Δt::Float64 
end