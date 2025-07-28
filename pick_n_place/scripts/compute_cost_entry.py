#!/usr/bin/env python3
import rospy
import numpy as np
from pick_n_place.srv import ComputeCostEntry, ComputeCostEntryResponse
import cost_update as cost_update



def handle_service(req):
    
    # Inputs: Cost matrix, deformation class, placement strategy, placement quality
    # Output: New cost

    flat_cost_table = req.cost_table #reshape the flattened matrix to a 3x3 matrix
    cost_table = [flat_cost_table[i:i+3] for i in range(0, 9, 3)]
    cost_table = np.array(cost_table)
    def_class = req.def_class
    placing_str = req.placing_str
    placement_error = 100 - req.placing_qual
    
    if(def_class=="A"):
        i=0
    elif(def_class=="B"):
        i=1
    elif(def_class=="C"):
        i=2
    if(placing_str=="placevert"):
        j=0
    elif(placing_str=="placediag"):
        j=1
    elif(placing_str=="placerot"):
        j=2
    
    updater = cost_update.CostUpdater(cost_table, 0.5, 0.3, 60, 0.5)

    print("Previous cost for state-action ", def_class, "-", placing_str, " was ", cost_table[i,j])
    new_cost = updater.update_cost(i,j, placement_error, 8) # trial fixed to 8 to have a constant learning rate of 0.3 since these trials are executed after system adaptability experiments once the cost table is learned
    new_cost_table = updater.get_cost_table()
    print("New cost for state-action ", def_class, "-", placing_str, " is ", new_cost)
    # print("New cost table: ", new_cost_table)
    
    flat_new_cost_table = new_cost_table.flatten()
    # flat_new_cost_table = [item for row in new_cost_table for item in row] #flatten 3x3 matrix
    # print("New flat cost table:", flat_new_cost_table)
    
    return ComputeCostEntryResponse(flat_new_cost_table, new_cost) #return new_cost_table and new_cost

if __name__ == '__main__':
    rospy.init_node('compute_cost_entry', anonymous=True)
    rospy.loginfo("compute_cost_entry: Node ready")
    s = rospy.Service('/pick_n_place/compute_new_cost', ComputeCostEntry, handle_service)
    rospy.spin()