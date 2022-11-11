# Solving time: 5ms

using Revise
using Jink 
using LinearAlgebra
using FiniteDiff

N = 10
left_hand_task = KinTask()
left_hand_task.name = :left_Hand_task 
left_hand_task.target = [-1.0, 3.0]
left_hand_task.task_map = picklerick.left_hand_pose
left_hand_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(picklerick.left_hand_pose, σ)
left_hand_task.weight = 1e0
left_hand_task.selection_matrix = I(N)

right_hand_task = KinTask()
right_hand_task.name = :right_Hand_task 
right_hand_task.target = [1.0, 3.0]
right_hand_task.task_map = picklerick.right_hand_pose
right_hand_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(picklerick.right_hand_pose, σ)
right_hand_task.weight = 1e0
right_hand_task.selection_matrix = I(N)

com_task = KinTask()
com_task.name = :com_task 
com_task.target = [0.0, 2.0]
com_task.task_map = picklerick.com_pose
com_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(picklerick.com_pose, σ)
com_task.weight = 1e0
com_task.selection_matrix = I(N)

posture_task = KinTask()
posture_task.name = :posture_task 
posture_task.target = [π/2, 2π/3, π/6, 2π/3, π/6, π/2, π/3, 7π/12, 2π/3, 5π/12]
posture_task.task_map = σ->σ
posture_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(ω->ω, σ)
posture_task.weight = 1e-1
posture_task.selection_matrix = I(N)

tasks = [left_hand_task, right_hand_task, com_task, posture_task]

θ = picklerick.get_generalized_coordinates()
# θ = [π/2, π, 0.0, 0.3, 0.3, 0.4, 0.3, π/3, 0.1, 0.0]

solver = initialize_solver(N) 
@time begin
for i=1:10
    global θ
θ̇ = solve_ik(tasks, θ, [-π*ones(N), π*ones(N)], solver)
θ = θ + θ̇ *solver.Δt
end
end

@show picklerick.left_hand_pose(θ)
@show picklerick.right_hand_pose(θ)
@show picklerick.com_pose(θ)

norm_error = norm([picklerick.left_hand_pose(θ); picklerick.right_hand_pose(θ); picklerick.com_pose(θ); θ] -
                [left_hand_task.target; right_hand_task.target; com_task.target; posture_task.target])
@show norm_error

picklerick.visualize!(θ)
1