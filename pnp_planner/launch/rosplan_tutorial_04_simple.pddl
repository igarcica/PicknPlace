<?xml version="1.0"?>
<launch>

	<!-- arguments -->
	<arg name="domain_path"	default="$(find pnp_planner)/rosplan_domain.pddl" />
	<arg name="problem_path"	default="$(find pnp_planner)/rosplan_problem.pddl" />

	<!-- ROSPlan -->
	<include file="$(find rosplan_planning_system)/launch/interfaced_planning_system.launch" >
		<arg name="use_problem_topic"    value="true" />
		<arg name="domain_path"          value="$(arg domain_path)" />
		<arg name="problem_path"         value="$(find pnp_planner)/pddl/problem.pddl" />
		<arg name="data_path"            value="$(find pnp_planner)/pddl/" />
		<arg name="planner_interface"    value="metricff_planner_interface" />
		<!--<arg name="planner_command"      value="timeout 10 $(find rosplan_planning_system)/common/bin/popf DOMAIN PROBLEM" /> -->
		<arg name="planner_command"      value="timeout 10 $(find rosplan_planning_system)/common/bin/Metric-FF -o DOMAIN -f PROBLEM -s 4 -w 1" />
	
	</include>

	<!-- sim actions -->
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="grasp" />
		<arg name="action_duration" value="5" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="approach" />
		<arg name="action_duration" value="15" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="rotate" />
		<arg name="action_duration" value="15" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="lift" />
		<arg name="action_duration" value="5" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="release" />
		<arg name="action_duration" value="5" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="placevert" />
		<arg name="action_duration" value="5" />
	</include>
	<include file="$(find rosplan_planning_system)/launch/includes/simulated_action.launch" >
		<arg name="pddl_action_name" value="placediag" />
		<arg name="action_duration" value="5" />
	</include>


</launch>