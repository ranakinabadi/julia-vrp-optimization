
# CVRP compact MILP formulation using Miller-Tucker-Zemlin (MTZ)
# load variables for simultaneous subtour elimination and
# capacity enforcement.
#
# Mathematical formulation:
# ─────────────────────────────────────────────────────────────
#   Sets:
#     N  = {1,...,n}   all nodes (1 = depot)
#     C  = {2,...,n}   customers
#     K  = {1,...,K}   vehicles
#
#   Variables:
#     x[i,j,k] ∈ {0,1}   1 if vehicle k travels arc i→j
#     load[i,k] ∈ [0,Q]  cumulative load on vehicle k after node i
#
#   Objective:
#     min  Σ dist[i,j] * x[i,j,k]   ∀ i,j,k
#
#   Constraints:
#     (1) No self-loops
#     (2) Each customer visited by exactly one vehicle
#     (3) Flow conservation at customer nodes
#     (4) Each vehicle departs depot exactly once
#     (5) Each vehicle returns to depot exactly once
#     (6) Load = 0 at depot (vehicles depart empty)
#     (7) Load propagation via big-M: enforces capacity
#         AND eliminates customer-only subtours


module VRPModel

using JuMP, HiGHS


export build_model

function build_model(inst; silent=true)
    (; n, K, Q, dist) = inst

    m = JuMP.Model(HiGHS.Optimizer)
    silent && set_silent(m)

    # decision variables 
    # x[i,j,k] = 1 if vehicle k travels from node i to node j
    @variable(m, x[1:n, 1:n, 1:K], Bin)

    # load[i,k] = cumulative demand loaded on vehicle k
    #             when it departs node i
    @variable(m, 0 <= load[1:n, 1:K] <= Q)

    # objective: minimize total travel distance 
    @objective(m, Min,
        sum(dist[i,j] * x[i,j,k]
            for i in 1:n, j in 1:n, k in 1:K))

    # constraint 1: no self-loops
    @constraint(m, [i in 1:n, k in 1:K], x[i,i,k] == 0)

    # constraint 2: each customer visited exactly once 
    # summing over all vehicles and all predecessors ensures
    # exactly one (vehicle, arc) pair covers each customer.
    @constraint(m, [j in 2:n],
        sum(x[i,j,k] for i in 1:n, k in 1:K if i != j) == 1)

    # constraint 3: flow conservation at customers 
    # arcs entering h must equal arcs leaving h per vehicle.
    # prevents vehicles from dead-ending at a customer.
    @constraint(m, [h in 2:n, k in 1:K],
        sum(x[i,h,k] for i in 1:n if i != h) ==
        sum(x[h,j,k] for j in 1:n if j != h))

    # constraints 4 & 5: depot flow
    # each vehicle departs depot AT MOST once 
    @constraint(m, [k in 1:K], sum(x[1,j,k] for j in 2:n) <= 1)

    # each vehicle returns to depot AT MOST once
    @constraint(m, [k in 1:K], sum(x[i,1,k] for i in 2:n) <= 1)

    # constraint 6: vehicles depart depot empty 
    @constraint(m, [k in 1:K], load[1,k] == 0)

    # constraint 7: load propagation (big-M) 
    # if x[i,j,k]=1: load[j,k] >= load[i,k] + d[j]  (active)
    # if x[i,j,k]=0: load[j,k] >= load[i,k] + d[j] - Q
    #                              always satisfied   (inactive)
    # this simultaneously tracks capacity AND prevents
    @constraint(m,
        [i in 1:n, j in 2:n, k in 1:K; i != j],
        load[j,k] >= load[i,k] + inst.d[j] - Q * (1 - x[i,j,k]))

    return m
end

end