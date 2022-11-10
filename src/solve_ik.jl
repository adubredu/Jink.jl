#=
    minimize 0.5*x'Px + q'x
    subject to  Gx ≤ h
                Ax = b 
                xmin ≤ x ≤ xmax

=#
function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    @variable(model, q̇[1:N])
    solver = JinkSolver(model, Δt)
    return solver
end

function build_objective(tasks::::AbstractArray{Task}, robot)
    θ, θ̇ = get_configuration(robot)
    for task in tasks  
        p_current, R_current = task.task_map(θ)
        current_pose = SE3(p_current, R_current) 
        e = pose_error(task.target, current_pose)

    end

    return P, r
end

function build_equality_constraints(tasks::AbstractArray{Task}, robot)

    return A, b
end

function build_inequality_constraints(tasks::AbstractArray{Task}, robot)

    return G, h 
end

function compute_velocity_limits(tasks::AbstractArray, Δt::Float64)

    return q̇_min, q̇_max
end

function solve_ik(tasks::AbstractArray{Task}, 
        solver::JinkSolver, robot; Δt::Float64=1e-3)
    model = solver.model
    Δt = solver.Δt
    P, r = build_objective(tasks, robot)
    A, b = build_equality_constraints(tasks, robot)
    G, h = build_inequality_constraints(tasks, robot)
    q̇_min, q̇_max = compute_velocity_limits(tasks, Δt)

    try delete(model, model.ext[:ineq])  catch nothing; end
    try delete(model, model.ext[:eq])  catch nothing; end
    try delete(model, model.ext[:lims])  catch nothing; end


    @objective(model, Min, 0.5*q̇'*P*q̇ + r'*q̇)
    model.ext[:ineq] = @constraint(model, G*q̇ .≤ h)
    model.ext[:eq] = @constraint(model, A*q̇ .== b)
    model.ext[:lims] = @constraint(model, q̇_min .≤ q̇ .≤ q̇_max)

    optimize!(model)
    Δq = value.(x)
    q̇sol = Δq/Δt
    return q̇sol
end