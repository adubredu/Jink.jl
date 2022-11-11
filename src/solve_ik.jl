#=
    minimize 0.5*x'Px + q'x
    subject to  Gx ≤ h
                Ax = b 
                xmin ≤ x ≤ xmax

=#
function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    @variable(model, q̇[1:N])
    solver = JinkSolver(model, Δt, q̇)
    return solver
end

function build_objective(tasks::AbstractArray{KinTask}, θ, Δt; K=0.5) 
    N = length(θ)
    P = zeros(N, N); r = zeros(1, N)
    for task in tasks  
        w = task.weight
        p_current = task.task_map(θ)
        J = task.task_map_jacobian(θ)  
        e = pose_error(task.target, p_current)
        v = e./Δt 
        v = hcat(v)  
        r += -w*K*v'*J 
        P += w*J'*J
    end
    return P, r
end

function build_equality_constraints(tasks::AbstractArray{KinTask}, θ, Δt )

    return A, b
end

function build_inequality_constraints(tasks::AbstractArray{KinTask} , θ, Δt)

    return G, h 
end

function compute_velocity_limits(θ, qlims, Δt::Float64; K=0.5)
    q_min, q_max = qlims
    q̇_min = K*(q_min - θ)/Δt
    q̇_max = K*(q_max - θ)/Δt
    return q̇_min, q̇_max
end

function solve_ik(tasks::AbstractArray{KinTask}, θ, qlims,
        solver::JinkSolver)
    model = solver.model
    set_silent(model)
    Δt = solver.Δt
    q̇ = solver.q̇
    P, r = build_objective(tasks, θ, Δt)
    # A, b = build_equality_constraints(tasks, θ, Δt)
    # G, h = build_inequality_constraints(tasks, θ, Δt)
    q̇_min, q̇_max = compute_velocity_limits(θ, qlims, Δt)
    # @show q̇_min, q̇_max
    # try delete(model, model.ext[:ineq])  catch nothing; end
    # try delete(model, model.ext[:eq])  catch nothing; end
    try delete(model, model.ext[:lims])  catch nothing; end 

    @objective(model, Min, 0.5*(q̇'*P*q̇) + r⋅q̇)
    # model.ext[:ineq] = @constraint(model, G*q̇ .≤ h)
    # model.ext[:eq] = @constraint(model, A*q̇ .== b)
    model.ext[:lims] = @constraint(model, q̇_min .≤ q̇ .≤ q̇_max)

    optimize!(model)
    Δq = value.(q̇)
    q̇sol = Δq/Δt
    return q̇sol
end