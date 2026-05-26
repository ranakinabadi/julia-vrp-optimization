
# problem instance definition for the CVRP.


module Data

export CVRPInstance

struct CVRPInstance
    n       :: Int             # total nodes (1 = depot)
    K       :: Int             # number of vehicles
    Q       :: Int             # vehicle capacity
    coords  :: Vector{Tuple{Int,Int}}
    d       :: Vector{Int}     # demand per node (depot = 0)
    dist    :: Matrix{Float64} # precomputed distance matrix
end

function CVRPInstance(coords, d, K, Q)
    n    = length(coords)
    dist = [sqrt((coords[i][1]-coords[j][1])^2 +
                 (coords[i][2]-coords[j][2])^2)
            for i in 1:n, j in 1:n]
    CVRPInstance(n, K, Q, coords, d, dist)
end

# default small instance (9 nodes: 1 depot + 8 customers)
function default_instance(; K=3, Q=15)
    coords = [(5,5),(1,8),(3,9),(7,9),(9,7),
              (9,3),(7,1),(3,1),(1,3)]
    d      = [0, 3, 4, 5, 6, 4, 3, 5, 2]
    CVRPInstance(coords, d, K, Q)
end

end