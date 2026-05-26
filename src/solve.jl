

module Solve

using JuMP
export solve_cvrp, CVRPSolution

struct CVRPSolution
    status      :: String
    total_dist  :: Float64
    routes      :: Vector{Vector{Int}}
    demands     :: Vector{Int}
    route_dists :: Vector{Float64}
end

function extract_routes(x, n, K)
    routes = Vector{Int}[]
    for k in 1:K
        route = let current = 1
            r = Int[]
            for _ in 1:n
                push!(r, current)
                nxt = findfirst(
                    j -> value(x[current, j, k]) > 0.5, 1:n)
                nxt === nothing && break
                current = nxt
                current == 1 && break
            end
            push!(r, 1)
            r
        end
        push!(routes, route)
    end
    return routes
end


function solve_cvrp(inst, build_model_fn; silent=true)
    model = build_model_fn(inst; silent)
    optimize!(model)

    status = string(termination_status(model))
    routes = extract_routes(model[:x], inst.n, inst.K)

    active  = filter(r -> length(r) > 2, routes)
    demands = [sum(inst.d[i] for i in filter(i -> i != 1, r))
               for r in active]
    dists   = [sum(inst.dist[r[i], r[i+1]]
               for i in 1:length(r)-1)
               for r in active]

    return CVRPSolution(string(status), objective_value(model),
                        active, demands, dists)
end

end