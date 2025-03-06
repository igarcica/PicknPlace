### This scrpit implements an M-estimator using the Huber-like update rule

import numpy as np
import statistics as sts

class CostUpdater:
    def __init__(self, cost_table, alpha_0=1, alpha_stab=0.3, c=30, beta=1):
        """
        Initialize with:
        - cost_table: initial cost matrix (numpy array)
        - alpha: learning rate for updates
        - c: threshold for Huber loss
        """
        self.cost_table = cost_table
        self.alpha_0 = alpha_0
        self.alpha_stab = alpha_stab
        self.c = c
        self.beta = beta                #beta 1.5 stabilize alpha at step 6

    def get_alpha(self, t):
        """Compute decayed alpha based on the selected decay type."""
        # self.alpha = self.alpha_0 # Fized alpha
        # self.alpha = self.alpha_0 / (1 + self.beta * t)     # inverse time decay
        alpha = self.alpha_0 * np.exp(-self.beta * t) + self.alpha_stab   # exponential (stabilizes in alpha=0.3)
        print("Alpha: ", alpha)
        return alpha

    def huber_psi(self, r):
        """Huber influence function"""
        huber = np.where(np.abs(r) <= self.c, r, self.c * np.sign(r)) #sign indicates wether to increment or decrease cost
        print("Huber: ", huber)
        return huber

    def update_cost(self, i, j, observed_cost, n_exp):
        """Update cost entry (i, j) using Huber M-estimator"""
        print("Previous cost: ", self.cost_table[0, 0])
        print("Observation: ", observed_cost)

        residual = observed_cost - self.cost_table[i, j]
        print("Residual: ", residual)

        self.huber = self.huber_psi(residual)

        self.alpha = self.get_alpha(n_exp)
        # self.cost_table[i, j] += self.alpha * self.huber_psi(residual)
        # print("Cost update: ", self.alpha * self.huber_psi(residual))
        self.cost_table[i, j] += self.alpha * self.huber
        print("Cost update: ", self.alpha * self.huber)

    def get_cost_table(self):
        """Return the updated cost table"""
        return self.cost_table

## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [0, 0, 0],
    [0, 0, 0], 
    [15, 15, 15], 
])  
pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [0, 0, 0],
    [0, 0, 0], 
    [45, 45, 43], 
])  
n_exp = 1
## ---- PLACE observation ----
placing_strategy = "rotating"
def_class = "C"
placing_error = 22
## ---- PILE observation ----
placing_strategy_pile = "diagonal"
def_class_pile = "C"
placing_error_pile = 76


## Identify casilla
if(def_class=="A"):
    i=0
elif(def_class=="B"):
    i=1
elif(def_class=="C"):
    i=2

if(placing_strategy=="vertical"):
    j=0
elif(placing_strategy=="diagonal"):
    j=1
elif(placing_strategy=="rotating"):
    j=2


### PLACE COST TABLE
# updater = CostUpdater(initial_cost_table)
updater = CostUpdater(place_initial_cost_table, 1, 0.3, 30, 1.5)
updater.update_cost(i, j, placing_error, n_exp) # Update costs based on observation
print("Updated Cost Table:\n", updater.get_cost_table())


# ## ---- TEST - several simulated executions for placing vertical and A def class ----
# observations = [20, 18, 25, 20, 40, 0, 15, 14, 30, 20, 25, 2, 15] #placing error results
# # print(sts.stdev(observations))
# for i in range(0, len(observations)): 
#     updater.update_cost(0,0, observations[i]) #Update just cell 0,0 (placing vertical + A def class)
#     print("Updated Cost Table:\n", updater.get_cost_table())


### PILE COST TABLE
## Identify casilla
def_class = def_class_pile
placing_strategy = placing_strategy_pile
if(def_class=="A"):
    i=0
elif(def_class=="B"):
    i=1
elif(def_class=="C"):
    i=2

if(placing_strategy=="vertical"):
    j=0
elif(placing_strategy=="diagonal"):
    j=1
elif(placing_strategy=="rotating"):
    j=2

updater = CostUpdater(pile_initial_cost_table, 1, 0.3, 30, 1.5)
updater.update_cost(i, j, placing_error_pile, n_exp) # Update costs based on observation
print("Updated Cost Table:\n", updater.get_cost_table())



# class DecayingAlpha:
#     def __init__(self, alpha_0=0.1, beta=0.01, decay_type="inverse_time"):
#         """
#         Initialize the decaying alpha function.
        
#         Parameters:
#         - alpha_0: Initial learning rate
#         - beta: Decay factor (higher = faster decay)
#         - decay_type: "inverse_time", "exponential", or "step"
#         """
#         self.alpha_0 = alpha_0
#         self.beta = beta
#         self.t = 0  # Time step counter
#         self.decay_type = decay_type

#     def get_alpha(self):
#         """Compute decayed alpha based on the selected decay type."""
#         self.t += 1  # Increment time step
        
#         if self.decay_type == "inverse_time":
#             return self.alpha_0 / (1 + self.beta * self.t)
#         elif self.decay_type == "exponential":
#             return self.alpha_0 * np.exp(-self.beta * self.t) +0.3
#         elif self.decay_type == "step":
#             gamma = 0.5  # Decay factor per step
#             T = 10  # Step interval
#             return self.alpha_0 * (gamma ** (self.t // T))
#         else:
#             raise ValueError("Invalid decay type. Choose 'inverse_time', 'exponential', or 'step'.")

# # Example Usage
# decay = DecayingAlpha(alpha_0=1, beta=1.5, decay_type="exponential")

# for t in range(1, 21):
#     print(f"Step {t}, Alpha: {decay.get_alpha():.4f}")



