# test the distlp class
from distlp import DistLP
import time

obj = DistLP()
obj.Nrows = 7
obj.Ncols = 7
obj.Tp = 3

obj.setup_problem()
obj.solve()
#obj.setup_dynamics_constraints()
