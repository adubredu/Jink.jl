module Digit 

using Rotations 

include("utils.jl")
include("kinematics/kinematics.jl")


left_foot_pose = p_left_foot
right_foot_pose = p_right_foot
left_wrist_pose = p_left_wrist
right_wrist_pose = p_right_wrist
left_foot_jacobian = Jp_left_foot
right_foot_jacobian = Jp_right_foot
left_wrist_jacobian = Jp_left_wrist
right_wrist_jacobian = Jp_right_wrist
com_position = centerOfMass

end