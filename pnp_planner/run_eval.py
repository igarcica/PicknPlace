from jinja2 import Environment, FileSystemLoader
import pandas as pd
import subprocess
import random
import time
import os
import argparse
import PlanOutputParser
import shutil

random.seed(10)

class Evaluation:
    def __init__(self, current_path, domain, problem_template, planners_loc, n_tests):
        self.current_path = current_path
        self.domain_path = current_path + '/pddl/' + domain
        self.problem_path = current_path + '/pddl/current_problem.pddl'
        self.problem_template = problem_template
        self.planners_loc = planners_loc

        self.n_tests = n_tests
        self.csv_output = current_path + '/eval/csv_out_smart_insertion' + time.strftime("%Y%m%d-%H%M%S") + '.csv'

    def update_test_data(self, goals, test_number):
        data = {
                'name': test_number,

                'cooked_pan_cost_human' : random.randrange(0,110,10),
                'cooked_oven_cost_human' : random.randrange(0,110,10),
                'clean_mop_cost_human' : random.randrange(0,110,10),
                'clean_cloth_cost_human' : random.randrange(0,110,10),
                'cooked_pan_cost_robot' : random.randrange(0,110,10),
                'cooked_oven_cost_robot' : random.randrange(0,110,10),
                'clean_mop_cost_robot' : random.randrange(0,110,10),
                'clean_cloth_cost_robot' : random.randrange(0,110,10),

                'cooked_pan_duration_human' : random.randrange(10,110,10),
                'cooked_oven_duration_human' : random.randrange(10,110,10),
                'clean_mop_duration_human' : random.randrange(10,110,10),
                'clean_cloth_duration_human' : random.randrange(10,110,10),
                'cooked_pan_duration_robot' : random.randrange(10,110,10),
                'cooked_oven_duration_robot' : random.randrange(10,110,10),
                'clean_mop_duration_robot' : random.randrange(10,110,10),
                'clean_cloth_duration_robot' : random.randrange(10,110,10),

                'n_goals' : len([goal for goal in goals if goal!='']),

                'goal1': goals[0],
                'goal2': goals[1],
                'goal3': goals[2],
                'goal4': goals[3],
                'goal5': goals[4],
                'goal6': goals[5],
                'goal7': goals[6],
                'goal8': goals[7],
                'goal9': goals[8],
                'goal10': goals[9]
            }
        return data
    
    def update_problem_file(self, data, save=False):
        templateLoader = FileSystemLoader(searchpath='./pddl/')
        templateEnv = Environment(loader=templateLoader)
        template_file = self.problem_template
        template = templateEnv.get_template(template_file)
        output = template.render(data=data)
        if save:
            problem_path = self.current_path + '/pddl/problems_seed10/problem_' + data['name'] + '.pddl'
        else: problem_path = self.problem_path
        with open(problem_path, 'w') as problem:
             problem.write(output)
    
    def generate_goals(self):
        n_goals = random.randrange(1,11)
        predicates = ['cooked', 'cleaned']
        goals = [''] * 10
        for n in range(n_goals):
            predicate = random.choice(predicates)
            if predicate == 'cooked':
                goal = '(cooked food' + str(n+1) + ')'
            else: goal = '(cleaned floor' + str(n+1) + ')' 
            goals[n] = goal
        return goals

    def main(self):

        csv_header = pd.DataFrame([], columns=['test_number', 'input_data', 'planner_config', 'planner_output', 'metric', 'plan', 'time', 'makespan'])
        csv_header.to_csv(self.csv_output)

        # oneminute is a '#!/bin/bash ulimit -t 60 $@' system cmd created in an executable in /usr/bin
        popf_cmd = 'oneminute ' + self.planners_loc + 'popf -n ' + self.domain_path + ' ' + self.problem_path
        rewrite_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-improved --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path
        rewrite_aees_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-improved --aees --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path
        rewrite_aees2_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-improved --aees2 --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path
        rewrite_aees2_fhat_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-smart-insertion --insert-in-fhat-order --aees2 --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path
        rewrite_aees2_f_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-smart-insertion --insert-in-f-order --aees2 --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path

        rewrite_aees2_fhat_window_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-new-fhat --insert-in-fhat-order --aees2 --optimise --forbid-self-overlapping-actions --keep-heuristic-correction-in-window ' + self.domain_path + ' ' + self.problem_path

        rewrite_aees2_fhat_nowindow_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-new-fhat --insert-in-fhat-order --aees2 --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path

        rewrite_aees2_fhat_semaphores_window_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-new-fhat --use-semaphores-for-hhat --insert-in-fhat-order --aees2 --optimise --forbid-self-overlapping-actions --keep-heuristic-correction-in-window ' + self.domain_path + ' ' + self.problem_path

        rewrite_aees2_fhat_semaphores_nowindow_cmd = 'oneminute ' + self.planners_loc + 'rewrite-no-lp-new-fhat --use-semaphores-for-hhat --insert-in-fhat-order --aees2 --optimise --forbid-self-overlapping-actions ' + self.domain_path + ' ' + self.problem_path

        lpgtd_cmd = self.planners_loc + 'lpg-td-1.0 -o ' + self.domain_path + ' -f ' + self.problem_path + ' -quality -cputime 60'
        
        
        #planner_cfgs = [['popf', popf_cmd], ['rewrite', rewrite_cmd], ['rewrite aees', rewrite_aees_cmd], ['rewrite aees2', rewrite_aees2_cmd]]
        #planner_cfgs = [['rewrite aees2 fhat', rewrite_aees2_fhat_cmd], ['rewrite aees2 f', rewrite_aees2_f_cmd]]

        #planner_cfgs = [['rewrite aees2 fhat window', rewrite_aees2_fhat_window_cmd], ['rewrite aees2 fhat nowindow', rewrite_aees2_fhat_nowindow_cmd], ['rewrite aees2 fhat semaphores window', rewrite_aees2_fhat_semaphores_window_cmd], ['rewrite aees2 fhat semaphores nowindow', rewrite_aees2_fhat_semaphores_nowindow_cmd]]

        planner_cfgs = [['lpgtd', lpgtd_cmd]]

        lpgtd_cmd = self.planners_loc + 'lpg-td-1.0 -o ' + self.domain_path + ' -f ' + self.problem_path + ' -quality -cputime 60'
        
        planner_cfgs = [['popf', popf_cmd], ['rewrite', rewrite_cmd], ['rewrite aees', rewrite_aees_cmd], ['rewrite aees2', rewrite_aees2_cmd], ['lpgtd', lpgtd_cmd]]

        for test in range(self.n_tests):
            test_n = test
            test = 'test_' + str(test)
            goals = self.generate_goals()
            test_data = self.update_test_data(goals, test)
            self.update_problem_file(test_data)

            if test_n >= 0:
                for cmd in enumerate(planner_cfgs):
                    result = subprocess.run(cmd[1][1].split(), stdout=subprocess.PIPE)
                    output = result.stdout.decode('utf-8')
                    if cmd[1][0] == 'popf':
                        data = PlanOutputParser.POPFOutputParser(output)
                    elif cmd[1][0] == 'lpgtd':
                        data = PlanOutputParser.LPGTDOutputParser(output)
                    else:
                        data = PlanOutputParser.RewriteOutputParser(output)
                    row = [test, test_data, cmd[1][0], output, data.metric, data.plan, data.time, data.makespan]
                    pd.DataFrame([row]).to_csv(self.csv_output, mode='a', header=False)
    
    def generate_problem_files(self):
        for test in range(self.n_tests):
            test = 'test_' + str(test)
            goals = self.generate_goals()
            test_data = self.update_test_data(goals, test)
            self.update_problem_file(test_data, True)
        
if __name__ == "__main__":

    current_path = os.getcwd() # path to kcl_planner_cost_time (run script from this folder)

    # Read arguments from command line
    parser = argparse.ArgumentParser()
    
    parser.add_argument('-d', '--domain', type=str, help='name of the domain file, as stored in the folder pddl', required=True)
    parser.add_argument('-p', '--problem', type=str, help='name of the problem template file, as stored in the folder pddl', required=True)
    parser.add_argument('-c', '--planner_loc', type=str, help='path to the compiled planners (popf and rewritelp)', required=True)
    parser.add_argument('-n', '--n_tests', type=str, help='number of tests', required=True)
    
    args = parser.parse_args()
    domain = args.domain
    problem = args.problem
    planners_loc = args.planner_loc
    n_tests = int(args.n_tests)

    # domain = 'domain_kitchen_2agents_diff_durations.pddl'
    # problem = 'problem_kitchen_2agents_diff_durations_template.pddl'
    # planners_loc = '/home/silvia.izquierdo/compiled_planners/'
    # n_tests = 10
    # command: python3 scripts/run_eval.py -d 'domain_kitchen_2agents_diff_durations.pddl' -p 'problem_kitchen_2agents_diff_durations_template.pddl' -c './compiled_planners/' -n 270

    # Run evaluation
    evaluate = Evaluation(current_path, domain, problem, planners_loc, n_tests)
    evaluate.main()
    # evaluate.generate_problem_files()
