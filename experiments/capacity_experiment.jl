

coords = [(5,5),(1,8),(3,9),(7,9),(9,7),
          (9,3),(7,1),(3,1),(1,3)]
d      = [0, 3, 4, 5, 6, 4, 3, 5, 2]

println("Q\t| Vehicles used\t| Total distance\t| Avg route demand")
println("-"^60)

let  
    for Q in [8, 10, 12, 15, 20]
        inst = Data.CVRPInstance(coords, d, 4, Q)
        sol  = Solve.solve_cvrp(inst, VRPModel.build_model; silent=true)

        avg_demand = isempty(sol.demands) ? 0 :
                     round(sum(sol.demands) / length(sol.demands), digits=1)

        println("$Q\t| $(length(sol.routes))\t\t| ",
                "$(round(sol.total_dist, digits=2))\t\t| $avg_demand")
    end
end