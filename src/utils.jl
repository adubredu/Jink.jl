function pose_error(target_pose::Vector, current_pose::Vector)
    res = target_pose - current_pose
    return collect(res)
end