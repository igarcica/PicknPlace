echo "Generating a Problem.."
rosservice call /rosplan_problem_interface/problem_generation_server

echo "Planning.."
rosservice call /rosplan_planner_interface/planning_server

echo "The plan is: "
rostopic echo /rosplan_planner_interface/planner_output -p -n 1

echo "Executing the Plan..."

echo "The parsed plan is: "
rosservice call /rosplan_parsing_interface/parse_plan

echo "Dispatching plan..."
rosservice call /rosplan_plan_dispatcher/dispatch_plan
