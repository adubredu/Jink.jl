# using Pkg
# Pkg.add(url="https://github.com/adubredu/MeshCatMechanisms.jl")
# Pkg.add(url="https://github.com/adubredu/RigidBodyDynamics.jl")
# Pkg.add(url="https://github.com/adubredu/DigitVisualizer.jl")
using DigitVisualizer 
using DigitVisualizer.MeshCat
using DigitVisualizer.RigidBodyDynamics

using Revise
using Jink
using LinearAlgebra 
using FiniteDiff

# set up visualizer
vis = Visualizer()
initialize_arena!(vis)
digit_viz = DigitViz(vis)
load_digit_vis(digit_viz)

N = 30
left_hand_task = KinTask()
left_hand_task.name = :left_hand_task 
left_hand_task.target = [0.3,  0.32, 1.3]
left_hand_task.task_map = σ->digit.left_wrist_pose(σ)[1]
left_hand_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(ω->digit.left_wrist_pose(ω)[1], σ)
left_hand_task.weight = 1e0
s = ones(N); s[1:6] .= 0.0
left_hand_task.selection_matrix = diagm(s)

right_hand_task = KinTask()
right_hand_task.name = :right_hand_task 
right_hand_task.target = [0.3,  -0.5, 1.3]
right_hand_task.task_map = σ->digit.right_wrist_pose(σ)[1]
right_hand_task.task_map_jacobian = σ->FiniteDiff.finite_difference_jacobian(ω->digit.right_wrist_pose(ω)[1], σ)
right_hand_task.weight = 1e0
s = ones(N); s[1:6] .= 0.0
right_hand_task.selection_matrix = diagm(s)

tasks = [left_hand_task, right_hand_task]

θ = get_generalized_coordinates(digit_viz)

solver = initialize_solver(N) 
@time begin
for i=1:5
    global θ
θ̇ = solve_ik(tasks, θ, [-2π*ones(N), 2π*ones(N)], solver)
θ = θ + θ̇ *solver.Δt
end
end

@show digit.left_wrist_pose(θ)[1]

norm_error = norm([digit.left_wrist_pose(θ)[1]; digit.right_wrist_pose(θ)[1]]
    - [left_hand_task.target; right_hand_task.target])
@show norm_error

set_joint_positions!(θ, digit_viz)
open(digit_viz.mvis)