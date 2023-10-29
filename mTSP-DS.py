import gurobipy as gp
from gurobipy import GRB
from Customer import Customer
from Truck import Truck

#Parameters
n = 10 # customers
m = 1  # drone stations
Kn = 1 # trucks
C = 1
alpha = 0.5
eps = 100 #metri


def solve():
    model = gp.Model("mTSP-DS")
    
    # Variabili di decisione
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")
    
    for k in range(Kn): 
        x2 = model.addVar(vtype=GRB.INTEGER, name="x2")

def test_Truck_class():
    truck = Truck(1,2)
    print(truck.index)
    print(truck.location)
    

if __name__ == "__main__":
    solve()
    test_Truck_class()
