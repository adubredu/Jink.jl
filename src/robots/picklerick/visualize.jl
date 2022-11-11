function visualize!(θ) 
    fig = GLMakie.Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-5.,5.,-1.,5.)) 

    chains = link_poses(θ)
    chain1 = Observable([SVector(a[1], a[2]) for a in chains[1]])
    chain2 = Observable([SVector(a[1], a[2]) for a in chains[2]])
    chain3 = Observable([SVector(a[1], a[2]) for a in chains[3]])
    chain4 = Observable([SVector(a[1], a[2]) for a in chains[4]])
    chain5 = Observable([SVector(a[1], a[2]) for a in chains[5]])
    head = Observable([SVector(chains[3][3][1], chains[3][3][2])])

    lines!(ax, chain1; linewidth=5, color=:purple)
    lines!(ax, chain2; linewidth=5, color=:purple)
    lines!(ax, chain3; linewidth=5, color=:purple)
    lines!(ax, chain4; linewidth=5, color=:purple)
    lines!(ax, chain5; linewidth=5, color=:purple)

    scatter!(ax, chain1; marker=:circle, color=:black, markersize=0.2, markerspace=:data)
    scatter!(ax, chain2; marker=:circle, color=:black, markersize=0.2, markerspace=:data)
    scatter!(ax, chain3; marker=:circle, color=:black, markersize=0.2, markerspace=:data)
    scatter!(ax, chain4; marker=:circle, color=:black, markersize=0.2, markerspace=:data)
    scatter!(ax, chain5; marker=:circle, color=:black, markersize=0.2, markerspace=:data)
    scatter!(ax, head; marker=:circle, color=:black, markersize=0.6, markerspace=:data)
     
    body_observables = [chain1, chain2, chain3, chain4, chain5, head]
    # hidedecorations!(ax)
    display(fig)
    return ax, fig, body_observables
end