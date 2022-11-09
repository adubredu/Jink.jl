#=
    minimize 0.5*x'Px + q'x
    subject to  Gx ≤ h
                Ax = b 
                xmin ≤ x ≤ xmax

=#
function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    @variable(model, v[1:N])
    solver = JinkSolver(model, Δt)
    return solver
end

function build_objective(tasks::::AbstractArray{Task})

    return P, q 
end

function build_equality_constraints(tasks::AbstractArray{Task})

    return A, b
end

function build_inequality_constraints(tasks::AbstractArray{Task})

    return G, h 
end

function compute_velocity_limits(tasks::AbstractArray, Δt::Float64)
function solve_ik(tasks::AbstractArray{Task}, 
        solver::JinkSolver; Δt::Float64=1e-3)
    model = solver.model
    Δt = solver.Δt
    P, q = build_objective(tasks)
    A, b = build_equality_constraints(tasks)
    G, h = build_inequality_constraints(tasks)
    v_min, v_max = compute_velocity_limits(tasks, Δt)

    @objective(model, Min, 0.5*v'*P*v + q'*v)
    model.ext[:ineq] = @constraint(model, G*v .≤ h)
    model.ext[:eq] = @constraint(model, A*v .== b)
    model.ext[:lims] = @constraint(model, v_min .≤ v .≤ v_max)

    optimize!(model)
    Δq = value.(x)
    q̇ = Δq/Δt
    return q̇
end