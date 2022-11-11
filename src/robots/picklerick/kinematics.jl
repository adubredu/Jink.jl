# foot as reference
function link_poses(θ)
    l1 = 1.0
    l2 = 1.0
    l3 = 1.0
    l4 = 1.0
    l5 = 1.0
    l6 = 1.0
    l7 = 1.0
    l8 = 1.0
    l9 = 1.0
    l10 = 1.0
    l0 = 0.75
    w = 0.5 
    x8 = -w
    y8 = 0.0
    x10 = w 
    y10 = 0.0
    x7 = x8 + l8 * cos(θ[8])
    y7 = y8 + l8 * sin(θ[8])
    x9 = x10 + l10 * cos(θ[10])
    y9 = y10 + l10 * sin(θ[10]) 
    x6 = 0.5*(x7 + l7 * cos(θ[7]) + x9 + l9 * cos(θ[9]))
    y6 = 0.5*(y7 + l7 * sin(θ[7]) + y9 + l9 * sin(θ[9]))
    x1 = x6 + l1 * cos(θ[6])
    y1 = y6 + l1 * sin(θ[6])
    x2 = x1 - l2 * sin(θ[2])
    y2 = y1 + l2 * cos(θ[2])
    x3 = x2 - l3 * sin(θ[3])
    y3 = y2 + l3 * cos(θ[3])
    x4 = x1 + l4 * sin(θ[4])
    y4 = y1 + l4 * cos(θ[4])
    x5 = x4 + l5 * sin(θ[5])
    y5 = y4 + l5 * cos(θ[5])
    x0 = x1 + l0 * cos(θ[1])
    y0 = y1 + l0 * sin(θ[1])
    xn = x1 + 0.5*l0 * cos(θ[1])
    yn = y1 + 0.5*l0 * sin(θ[1]) 

    return [[[x6, y6], [x7, y7], [x8, y8]], [[x6, y6], [x9, y9], [x10, y10]], 
    [[x6, y6], [x1, y1], [x0, y0]], [[x1, y1], [x4, y4], [x5, y5]], [[x1, y1], [x2, y2], [x3, y3]], [[xn, yn]]]
end

function left_hand_pose(θ)
    poses = link_poses(θ)
    return poses[5][end]
end

function right_hand_pose(θ)
    poses = link_poses(θ)
    return poses[4][end]
end

function left_foot_pose(θ)
    poses = link_poses(θ)
    return poses[1][end]
end

function right_foot_pose(θ)
    poses = link_poses(θ)
    return poses[2][end]
end

function head_pose(θ)
    poses = link_poses(θ)
    return poses[3][end]
end

function neck_pose(θ)
    poses = link_poses(θ)
    return poses[6][end]
end

function get_generalized_coordinates()
    return zeros(10)
end

function com_pose(θ)
    l1 = 1.0
    l2 = 1.0
    l3 = 1.0
    l4 = 1.0
    l5 = 1.0
    l6 = 1.0
    l7 = 1.0
    l8 = 1.0
    l9 = 1.0
    l10 = 1.0
    l0 = 0.75
    w = 0.5
    m_links = 0.05
    m_head = 0.1
    x8 = -w
    y8 = 0.0
    x10 = w 
    y10 = 0.0
    x7 = x8 + l8 * cos(θ[8])
    y7 = y8 + l8 * sin(θ[8])
    x9 = x10 + l10 * cos(θ[10])
    y9 = y10 + l10 * sin(θ[10]) 
    x6 = 0.5*(x7 + l7 * cos(θ[7]) + x9 + l9 * cos(θ[9]))
    y6 = 0.5*(y7 + l7 * sin(θ[7]) + y9 + l9 * sin(θ[9]))
    x1 = x6 + l1 * cos(θ[6])
    y1 = y6 + l1 * sin(θ[6])
    x2 = x1 - l2 * sin(θ[2])
    y2 = y1 + l2 * cos(θ[2])
    x3 = x2 - l3 * sin(θ[3])
    y3 = y2 + l3 * cos(θ[3])
    x4 = x1 + l4 * sin(θ[4])
    y4 = y1 + l4 * cos(θ[4])
    x5 = x4 + l5 * sin(θ[5])
    y5 = y4 + l5 * cos(θ[5])
    x0 = x1 + l0 * cos(θ[1])
    y0 = y1 + l0 * sin(θ[1])

    xcm = (sum([m_links*x for x in [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10]]) + m_head*x0) / (10*m_links + m_head)
    ycm = (sum([m_links*y for y in [y1, y2, y3, y4, y5, y6, y7, y8, y9, y10]]) + m_head*y0) / (10*m_links + m_head)
    
    return [xcm, ycm]
end