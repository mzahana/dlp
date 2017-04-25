# test the distlp class
from distlp import DistLP
import time

obj = DistLP()
obj.Nrows = 5
obj.Ncols = 5

obj.Tp = 3

obj.setup_problem()
obj.solve()
#obj.scipy_solve()
#obj.setup_dynamics_constraints()
