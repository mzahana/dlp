from cvxpy_dlp import DistLP

prob = DistLP()

prob.set_myID(1)
prob.set_Nrows(10)
prob.set_Ncols(10)
prob.set_Nattackers(3)
prob.set_Ndefenders(3)
prob.set_Base([1])
prob.set_BaseRef([2,11,12])
prob.set_neighbor_R(1)
prob.set_d_current_locations([20,40,50])
prob.set_e_current_locations([33,34,35])
prob.set_Tp(10)
prob.set_strategy_weights(0.0,-1.0)
prob.set_sensing_R(2)
prob.set_grid_resolution(1,1)
prob.set_origin_shifts([0.,0.])

prob.setup_problem()

prob.solve()

s,d = prob.get_min_dist_sectors2sectors([1,2,3],[4,5,6,12,13,3])

print s,d
