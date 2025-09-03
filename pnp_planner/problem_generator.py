import csv
import sys

# def get_params():
#     graph_file = ''
#     graph_file = str(input("Insert graph file:\n"))
#     domain_name = str(input("Insert Domain file name:\n"))
#     problem_name = str(input("Insert Problem file name:\n"))
#     gt_o = input("Insert GT Init:\n")
#     gl_o = input("Insert GL Init:\n")
#     cc_o = input("Insert CC Init:\n")
#     gt_d = input("Insert GT Goal:\n")
#     gl_d = input("Insert GL Goal:\n")
#     cc_d = input("Insert CC Goal:\n")

#     print "Init state: " + str(gt_o) + ", " + str(gl_o) + ", " + str(cc_o)
#     print "Goal state: " + str(gt_d) + ", " + str(gl_d) + ", " + str(cc_d)
#     return graph_file, domain_name, problem_name, gt_o, gl_o, cc_o, gt_d, gl_d, cc_d


def generate_problem(objects, placing_costs, piling_costs):
    obj1 = objects[0]

    content = ""
    # content += "(define (problem " + str(problem_name) + ")\n"
    # content += "(:domain " + str(domain_name) + ")\n" 
    content += "(define (problem PICKNPLACEpileclass) \n"
    content += "(:domain PICKNPLACEpileclass)\n" 
    
    ## Objects
    content += "(:objects \n "
    content += "\t table "
    for idx, obj in enumerate(objects):
        content += str(obj) + " "
    content += "- garment \n"

    content += "\t placevert placediag placerot - placing \n\t long short - grasp \n\t grws rotws - workspace \n\t grasped placed notgrasped lifted - state \n\t home high_pose else drag_pose - position \n\t flat A B C - defclass \n ) \n"
    
    ## init
    content += "\n"
    content += "(:init (garment_state table placed) (robot_at else) (robot_empty) \n"
    content += "\n"
    content += "\t ;; object to place (cloth-to-table costs) \n"
    content += "\t (known_obj " + str(obj1) + ") (garment_state " + str(obj1) + " notgrasped) (garment_at " + str(obj1) + " rotws) (at_pose " + str(obj1) + " long) (not (corners_pos_known " + str(obj1) + ")) (defstate " + str(obj1) + " flat) \n"
    content += "\n"
    content += "\t ;; objects to pile (cloth-to-cloth costs) \n"
    for idx, obj in enumerate(objects[1:]):    
        content += "\t (not (known_obj " + obj + ")) (garment_state " + str(obj) + " notgrasped) (garment_at " + str(obj) + " rotws) (at_pose " + str(obj) + " long) (not (corners_pos_known " + str(obj) + ")) (defstate " + str(obj) + " flat) \n"
    content += "\n"
    content += "\t (= (time_cost) 0) \n\t (= (place_qual) 0) \n"

    ## cloth-to-table costs
    content += "\n\t ;;placing costs \n"
    content += "\t (= (place_succ " + str(obj1) + " A placevert) " + str(placing_costs[0]) + ")\n"
    content += "\t (= (place_succ " + str(obj1) + " A placediag) " + str(placing_costs[1]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " A placerot) " + str(placing_costs[2]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " B placevert) " + str(placing_costs[3]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " B placediag) " + str(placing_costs[4]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " B placerot) " + str(placing_costs[5]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " C placevert) " + str(placing_costs[6]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " C placediag) " + str(placing_costs[7]) + ") \n"
    content += "\t (= (place_succ " + str(obj1) + " C placerot) " + str(placing_costs[8]) + ") \n"

    ## cloth-to-cloth costs
    content += "\n\t ;;piling costs \n "
    for idx, obj in enumerate(objects[1:]):
        content += "\t (= (place_succ " + str(obj) + " A placevert) " + str(piling_costs[0]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " A placediag) " + str(piling_costs[1]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " A placerot) " + str(piling_costs[2]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " B placevert) " + str(piling_costs[3]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " B placediag) " + str(piling_costs[4]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " B placerot) " + str(piling_costs[5]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " C placevert) " + str(piling_costs[6]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " C placediag) " + str(piling_costs[7]) + ") \n"
        content += "\t (= (place_succ " + str(obj) + " C placerot) " + str(piling_costs[8]) + ") \n\n"

    ## Deformation classes
    for idx, obj in enumerate(objects):
        content += "\t (obj_grasp_class " + str(obj) + " short A) \n"
        content += "\t (obj_grasp_class " + str(obj) + " long A) \n"
    content += ")\n\n"

    ## GOAL - objects to pile
    # content += "(:goal (and (on " + str(obj1) + " table) (on " + str(obj) + str(obj1) + ") (on checkered1 waffle1) )) \n"
    # for idx, obj in enumerate(objects[2:]):
    #     content += " (on )" + str(obj) + str(obj) + ") "
    # content += "\n\n"
    content += "(:goal (and (on " + str(obj1) + " table) "
    for i in range(1,len(objects)):
        content += "(on " + str(objects[i]) + " " + str(objects[i-1]) + ") "
    content += "))\n\n"

    ## Metric
    # content += "(:metric minimize (+ (time_cost) (place_qual))) \n"
    content += "(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual)))) \n"
    content += "\n )"

    return content

def main():

    # Get object names through term
    # objects = ["checkered1", "checkered2"]
    input_string = input("Enter names separated by commas: ")
    objects = [obj_names.strip() for obj_names in input_string.split(",")] # Split the string into a list of names, stripping whitespace

    # Get cloth-to-table costs
    # placing_costs = [17, 1, 1, 8, 22, 6, 30, 25, 6]
    placing_costs = [0,0,0,0,0,0,0,0,0]
    # input_costs = input("Enter cloth-to-table costs: ")
    # placing_costs = [costs.strip() for costs in input_costs.split(",")]

    # Get cloth-to-cloth costs
    # piling_costs = [7, 3, 4, 30, 14, 9, 30, 30, 30]
    piling_costs = [0,0,0,0,0,0,0,0,0]
    # input_costs = input("Enter cloth-to-table costs: ")
    # piling_costs = [costs.strip() for costs in input_costs.split(",")]

    ## Create PDDL problem file
    content = generate_problem(objects, placing_costs, piling_costs)
    print(content)
    
    # Write problem
    path = './rosplan_problem.pddl'
    problem_file = open(path, "w")
    problem_file.write(content)
    problem_file.close()
    print("Problem created!")


if __name__ == "__main__":
    main()