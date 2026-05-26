# run.jl — single entry point

include("src/data.jl")
include("src/model.jl")
include("src/solve.jl")
include("src/visualize.jl")

using .Data, .VRPModel, .Solve, .Visualize

# build instance, solve, plot
inst = Data.default_instance(K=3, Q=15)
sol  = Solve.solve_cvrp(inst, VRPModel.build_model; silent=true)

println("Status:         ", sol.status)
println("Total distance: ", round(sol.total_dist, digits=4))
println()

for (k, route) in enumerate(sol.routes)
    println("Vehicle $k: ", join(route, " → "),
            "  |  demand=$(sol.demands[k])/$(inst.Q)",
            "  |  dist=$(round(sol.route_dists[k], digits=2))")
end

# plot
Visualize.plot_cvrp(inst, sol)