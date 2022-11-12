include("com.jl")
include("com_wrt_feet.jl")
include("J_left_foot.jl")
include("J_left_wrist.jl")
include("J_right_foot.jl")
include("J_right_wrist.jl")
include("left_foot.jl")
include("left_wrist.jl")
include("right_foot.jl")
include("right_wrist.jl")

function com(θ)
    p = com_wrt_feet(θ)
    res = [0.5(p[1]+p[4]);
           p[2]+p[5];
           0.5(p[3]+p[6]) ]
    return res 
end

