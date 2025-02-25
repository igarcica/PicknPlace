### This scrpit implements an M-estimator using the Huber-like update rule

import numpy as np
import statistics as sts

class CostUpdater:
    def __init__(self, cost_table, alpha=0.3, c=30):
        """
        Initialize with:
        - cost_table: initial cost matrix (numpy array)
        - alpha: learning rate for updates
        - c: threshold for Huber loss
        """
        self.cost_table = cost_table
        self.alpha = alpha
        self.c = c

    def huber_psi(self, r):
        """Huber influence function"""
        # print("sign x: ", self.c * np.sign(r))
        huber = np.where(np.abs(r) <= self.c, r, self.c * np.sign(r)) #sign indicates wether to increment or decrease cost
        # print("Huber: ", huber)
        return huber

    def update_cost(self, i, j, observed_cost):
        """Update cost entry (i, j) using Huber M-estimator"""
        print("Previous cost: ", self.cost_table[0, 0])
        print("Observation: ", observed_cost)

        residual = observed_cost - self.cost_table[i, j]
        print("Residual: ", residual)

        self.cost_table[i, j] += self.alpha * self.huber_psi(residual)
        print("Cost update: ", self.alpha * self.huber_psi(residual))

    def get_cost_table(self):
        """Return the updated cost table"""
        return self.cost_table

## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
initial_cost_table = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # Init cost table
updater = CostUpdater(initial_cost_table)

# ## ---- TEST - several simulated executions for placing vertical and A def class ----
# observations = [20, 18, 25, 20, 40, 0, 15, 14, 30, 20, 25, 2, 15] #placing error results
# # print(sts.stdev(observations))
# for i in range(0, len(observations)): 
#     updater.update_cost(0,0, observations[i]) #Update just cell 0,0 (placing vertical + A def class)
#     print("Updated Cost Table:\n", updater.get_cost_table())


## ---- Single observation ----
#Input:
placing_strategy = "vertical"
def_class = "A"
observation = 20

if(placing_strategy=="vertical"):
    i=0
elif(placing_strategy=="diagonal"):
    i=1
elif(placing_strategy=="rotating"):
    i=2

if(def_class=="A"):
    j=0
elif(def_class=="B"):
    j=1
elif(def_class=="C"):
    j=2
    
# Update costs based on observation
updater.update_cost(i, j, observation)
print("Updated Cost Table:\n", updater.get_cost_table())



