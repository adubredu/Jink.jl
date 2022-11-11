using Revise
using Jink 
using LinearAlgebra
using FiniteDiff

N = 10
hand_task = KinTask()
hand_task.name = :Hand_task 
hand_task.target = [1.0, 1.0]
hand_task.task_map = picklerick.left_hand_pose
hand_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(picklerick.left_hand_pose, σ)
hand_task.weight = 1e0
hand_task.selection_matrix = I(N)

tasks = [hand_task]

# θ = picklerick.get_generalized_coordinates()
θ = [π/2, π, 0.0, 0.3, 0.3, 0.4, 0.3, π/3, 0.1, 0.0]
solver = initialize_solver(N)
for i=1:10
    global θ
    θ̇ = solve_ik(tasks, θ, [-π*ones(N), π*ones(N)], solver)
    θ = θ̇*solver.Δt
    @show "solution is ", picklerick.left_hand_pose(θ)
end
picklerick.visualize!(θ)