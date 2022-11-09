function rpy_to_quat(q::Vector{Float64})
    ypr = RotZYX(q...)
    qr = QuatRotation(ypr)
    quat = [qr.q.v3, qr.q.v2, qr.q.v1, qr.q.s]
    return quat
end

function qall_to_q_pinocchio(q_frost)
    q_pin = zeros(31)
    q_pin[1:3] = q_frost[1:3]
    q_pin[4:7] = rpy_to_quat(q_frost[4:6])  
    q_pin[8:end] = q_frost[7:end] 
    return q_pin
end