

module Visualize

using Plots, LinearAlgebra
export plot_cvrp

function plot_cvrp(inst, sol; save_path="cvrp_solution.png")
    colors = [:purple, :teal, :darkorange, :crimson, :steelblue]

    p = plot(
        title      = "CVRP Solution  |  dist=$(round(sol.total_dist, digits=2))  Q=$(inst.Q)",
        xlabel     = "x", ylabel = "y",
        legend     = :topright,
        size       = (720, 620),
        grid       = true,
        gridalpha  = 0.15,
        framestyle = :box,
        dpi        = 150
    )

    #  each vehicle's route 
    for (k, route) in enumerate(sol.routes)
        xs = [inst.coords[i][1] for i in route]
        ys = [inst.coords[i][2] for i in route]

        customers  = filter(i -> i != 1, route)
        demand_sum = sum(inst.d[i] for i in customers)
        dist_k     = round(sol.route_dists[k], digits=1)
        c          = colors[mod1(k, length(colors))]

        # the route line
        plot!(p, xs, ys,
            color     = c,
            linewidth = 2.5,
            label     = "V$k  load=$demand_sum/$(inst.Q)  dist=$dist_k"
        )

        #  direction arrows manually on each arc midpoint
        for i in 1:length(route)-1
            x1, y1 = inst.coords[route[i]]
            x2, y2 = inst.coords[route[i+1]]

            # midpoint of the arc
            mx = (x1 + x2) / 2
            my = (y1 + y2) / 2

            # direction vector scaled to arrow size
            dx = (x2 - x1) * 0.01
            dy = (y2 - y1) * 0.01

            quiver!(p, [mx], [my],
                quiver   = ([dx], [dy]),
                color    = c,
                linewidth = 2
            )
        end
    end

    # depot 
    scatter!(p, [inst.coords[1][1]], [inst.coords[1][2]],
        markersize        = 16,
        markercolor       = :black,
        markerstrokecolor = :white,
        markerstrokewidth = 2,
        marker            = :star5,
        label             = "Depot"
    )

    savefig(p, save_path)
    println("Saved → $save_path")
    display(p)
    return p
end

end