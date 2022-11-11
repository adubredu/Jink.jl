#=
    minimize Σᵢ wᵢ⋅||Jᵢqᵢ̇ - Kᵢvᵢ||²
    subject to  q̇̄ ≤ q̇ ≤ q̇⁺ 

=#
function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    @variable(model, q̇[1:N])
    solver = JinkSolver(model, Δt)
    return solver
end

function build_objective(tasks::AbstractArray{KinTask}, θ, Δt; K=1.0) 
    Js = []
    vs = []
    ws = [] 
    for task in tasks 
        push!(Js, task.task_map_jacobian(θ))
        push!(ws, task.weight)
        p_current = task.task_map(θ)
        e = pose_error(task.target, p_current)
        push!(vs, K*e)
    end
    return Js, vs, ws
end

function build_equality_constraints(tasks::AbstractArray{KinTask}, θ, Δt )

    return A, b
end

function build_inequality_constraints(tasks::AbstractArray{KinTask} , θ, Δt)

    return G, h 
end

function compute_velocity_limits(θ, qlims, Δt::Float64; K=0.5)
    q_min, q_max = qlims
    q̇_min = K*(q_min - θ)
    q̇_max = K*(q_max - θ)
    return q̇_min, q̇_max
end

function solve_ik(tasks::AbstractArray{KinTask}, θ, qlims,
        solver::JinkSolver)
    model = solver.model
    set_silent(model)
    Δt = solver.Δt
    q̇ = solver.model.obj_dict[:q̇]
    Js, vs, ws = build_objective(tasks, θ, Δt) 
    q̇_min, q̇_max = compute_velocity_limits(θ, qlims, Δt) 

    try delete(model, model.ext[:lims])  catch nothing; end 

    @objective(model, Min, 
            sum([w*(J*q̇ - v)'*(J*q̇ - v) for (J, v, w) in zip(Js, vs, ws)]))

    model.ext[:lims] = @constraint(model, q̇_min .≤ q̇ .≤ q̇_max)

    optimize!(model)
    Δq = value.(q̇)
    q̇sol = Δq/Δt
    return q̇sol
end