import os
import glob
import argparse
import sys
import numpy as np
import scipy
from scipy import stats
import subprocess
import time
import shutil
import difflib
import logging
from difflib import Differ
from serializer import Serializer
from gen_ik_solution import *
from librobot import *
from libutil import *



class SimulationRunner:
    def __init__(self, problem, config):
        self.abs_path = os.path.dirname(os.path.abspath(__file__))
        self.serializer = Serializer()
        self.config_file = config
        self.config = self.serializer.read_config(self.abs_path + "/" + config)        
        self.ik_solution_generator = IKSolutionGenerator()        
        self.clear_stats(problem)
        self.run(problem)
        
    def clear_stats(self, problem):
        for file in glob.glob(self.abs_path + "/../problems/" + problem + "/log.log"):
            os.remove(file)
        for file in glob.glob(self.abs_path + "/../problems/" + problem + "/log.log"):
            os.remove(file)
        if os.path.isdir(self.abs_path + "/stats"):
            cmd = "rm -rf " + self.abs_path + "/stats/*"        
            os.system(cmd)
        else:
            os.makedirs(self.abs_path + "/stats")
        
    def write_rewards(self, log_file_path):
        rewards = []
        for line in tuple(open(log_file_path, 'r')):
            if "Reward" in line:                
                rewards.append(float(line.split(":")[1][1:]))        
        n, min_max, mean_rewards, reward_variances, skew, kurt = stats.describe(np.array(rewards))            
        if np.isnan(np.asscalar(reward_variances)):
            reward_variances = np.array(0.0)
            
        with open(log_file_path, "a+") as f:
            f.write("Reward variance: " + str(np.asscalar(reward_variances)) + " \n")
        with open(log_file_path, "a+") as f:
            f.write("Reward standard deviation: " + str(np.asscalar(np.sqrt(reward_variances))) + " \n")
    
    def read_float_data(self, log_file_path, searchstring):
        data = 0.0
        for line in tuple(open(log_file_path, 'r')):
            if searchstring in line:
                data = float(line.split(":")[1].split(" ")[1].rstrip("\n"))
        return data
    
    def read_int_data(self, log_file_path, searchstring):
        data = 0.0
        for line in tuple(open(log_file_path, 'r')):
            if searchstring in line:
                data = int(float(line.split(":")[1].split(" ")[1].rstrip("\n")))
        return data      
    
    def read_avg_path_length(self, log_file_path, num_runs):
        l = 0
        for line in tuple(open(log_file_path, 'r')):
            if "S:" in line:
                l += 1
        return l / num_runs
                
    def replace_option_in_config(self, option_name, value, problem, is_bool=False):
        bool_str = None
        if is_bool:
            bool_str = "false"
            if value:
                bool_str = "true"    
        lines = list(open(self.abs_path + "/../problems/" + problem + "/default.cfg", 'r'))
        idx = -1       
        for i in xrange(len(lines)):
            if option_name in lines[i]:
                idx = i 
        if not idx == -1:
            if is_bool:
                lines[idx] = option_name + " = " + str(bool_str) + "\n"
            else:
                lines[idx] = option_name + " = " + str(value) + "\n"
        os.remove(self.abs_path + "/../problems/" + problem + "/default.cfg")
        with open(self.abs_path + "/../problems/" + problem + "/default.cfg", 'a+') as f:
            for line in lines:
                f.write(line)
                
    def compareEnvironmentToTmpFiles(self, problem, model_file, environment_path, environment_file):
        if not os.path.exists(self.abs_path + "/tmp/" + problem):
            os.makedirs(self.abs_path + "/tmp/" + problem)
            return False
        
        if not (os.path.exists(self.abs_path + '/tmp/' + problem + '/' + environment_file) and
                os.path.exists(self.abs_path + '/tmp/' + problem + '/' + self.config_file) and
                os.path.exists(self.abs_path + '/tmp/' + problem + '/' + model_file)):            
            return False
        
        with open(self.abs_path + '/../problems/' + problem + "/environment/" + environment_file, 'r') as f1, open(self.abs_path + '/tmp/' + problem + '/' + environment_file, 'r') as f2:
            missing_from_b = [
                diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
                if diff.startswith('-')
            ]
            if len(missing_from_b) != 0:
                return False
            
        with open(self.abs_path + '/../problems/' + problem + "/model/" + model_file, 'r') as f1, open(self.abs_path + '/tmp/' + problem + '/' + model_file, 'r') as f2:
            missing_from_b = [
                diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
                if diff.startswith('-')
            ]
            if len(missing_from_b) != 0:
                return False
            
        
        with open(self.abs_path + "/" + self.config_file, 'r') as f1, open(self.abs_path + '/tmp/' + problem + '/' + self.config_file, 'r') as f2:
            missing_from_b = [
                diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
                if diff.startswith('-')
            ]
            
            for i in xrange(len(missing_from_b)):
                if ("num_links" in missing_from_b[i] or
                    "workspace_dimensions" in missing_from_b[i] or
                    "goal_position" in missing_from_b[i] or
                    "goal_radius" in missing_from_b[i]):
                    return False
        
        """ If same, use existing goalstates """
        try:        
            shutil.copy2(self.abs_path + '/tmp/' + problem + '/goalstates.txt', self.abs_path + '/../problems/' + problem + "/goalstates.txt")
        except:
            return False
        return True        
        
    def copyToTmp(self, problem, model_file, environment_file):
        shutil.copy2(self.abs_path + '/../problems/' + problem + "/environment/" + environment_file, self.abs_path + '/tmp/' + problem + '/' + environment_file)
        shutil.copy2(self.abs_path + '/../problems/' + problem + "/model/" + model_file, self.abs_path + '/tmp/' + problem + '/' + model_file)
        shutil.copy2(self.abs_path + '/../problems/' + problem + "/goalstates.txt", self.abs_path + '/tmp/' + problem + '/goalstates.txt')
        shutil.copy2(self.abs_path + "/" + self.config_file, self.abs_path + '/tmp/' + problem + '/' + self.config_file) 
        
    def get_average_distance_to_goal_area(self, goal_position, goal_radius, cartesian_coords):        
        avg_dist = 0.0
        goal_pos = np.array(goal_position)        
        for i in xrange(len(cartesian_coords)):            
            cart = np.array(cartesian_coords[i])            
            dist = np.linalg.norm(goal_pos - cart)            
            if dist < goal_radius:
                dist = 0.0
            avg_dist += dist
        if avg_dist == 0.0:
            return avg_dist        
        return np.asscalar(avg_dist) / len(cartesian_coords)
    
    def write_mean_num_collisions(self, log_file_path, num_runs):
        num_collisions = 0.0        
        with open(log_file_path, 'r') as f:
            for line in f:
                if "Trans: Collision detected: True" in line:
                    num_collisions += 1
        mean_num_collisions = num_collisions / num_runs
        with open(log_file_path, "a+") as f:
            f.write("Mean num collisions per run: " + str(mean_num_collisions) + " \n")
        
    def run(self, problem):        
        alg = "alg"
        if "manipulator_discrete" in problem:
            alg = "abt"
        elif "manipulator_continuous" in problem:
            alg = "gps"
        num_runs = self.config['num_simulation_runs']
        init_state = self.config['init_state']        
        discount = self.config['discount_factor']
        control_rate = self.config['control_rate']        
        illegal_move_penalty = self.config['illegal_move_penalty']
        illegal_action_penalty = self.config['illegal_action_penalty']
        step_penalty = self.config['step_penalty']
        exit_reward = self.config['exit_reward']
        planning_time = self.config['planning_time']
        particle_count = self.config['particle_count']
        num_steps = self.config['num_steps']
        save_particles = self.config['save_particles']
        plot_particles = self.config['plot_particles']        
        verbose = self.config['verbose']
        verbose_rrt = self.config['verbose_rrt']
        rrt_stretching_factor = self.config['rrt_stretching_factor']        
        histories_per_step = self.config['histories_per_step']
        prune_every_step = self.config['prune_every_step']
        continuous_collision = self.config['continuous_collision_check']        
        check_linear_path = self.config['check_linear_path']
        enforce_constraints = self.config['enforce_constraints']
        planner = self.config['planner']
        dynamic_problem = self.config['dynamic_problem']        
        simulation_step_size = self.config['simulation_step_size']
        planning_velocity = self.config['planning_velocity']
        show_viewer = self.config['show_viewer']
        max_observation_distance = self.config['max_observation_distance']
        particle_replenish_timeout = self.config['particle_replenish_timeout']
        state_sampling_strategy = self.config['state_sampling_strategy']
        num_samples_weighted_lazy = self.config['num_samples_weighted_lazy']
        gravity = self.config['gravity']
        particle_plot_limit = self.config['particle_plot_limit']
        num_effective_particles = self.config['num_effective_particles']
        save_policy = self.config['save_policy']
        load_initial_policy = self.config['load_initial_policy']
        initial_planning_time = self.config['initial_planning_time']
        particle_filter = self.config['particle_filter']
        num_actions_per_joint = self.config['num_actions_per_joint']
        environment_file = self.config['environment_file']
        robot_file = self.config['robot_file']
        
        model_file = "test.xml"
        environment_path = os.path.abspath(self.abs_path + "/../problems/" + problem + "/environment/" + environment_file)        
        
        logging_level = logging.WARN
        if verbose:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)
        robot = Robot(os.path.abspath(self.abs_path + "/../problems/" + problem + "/model/" + robot_file))
        utils = Utils()        
        obstacles = utils.loadObstaclesXML(environment_path)
        
        ja = v_double()
        init_angles = [init_state[i] for i in xrange(len(init_state) / 2)]
        ja[:] = init_angles
        goal_position = v_double()
        robot.getEndEffectorPosition(ja, goal_position)
        
        goal_area = v_double()
        utils.loadGoalArea(environment_path, goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        goal_position = [goal_area[i] for i in xrange(0, 3)]
        goal_radius = goal_area[3] 
        
        if not self.compareEnvironmentToTmpFiles(problem, model_file, environment_path, environment_file):
            self.ik_solution_generator.setup(robot,
                                             obstacles,                                             
                                             planning_velocity, 
                                             1.0 / control_rate,
                                             planner,
                                             100000.0,
                                             True)        
            
            ik_solutions = self.ik_solution_generator.generate(init_state,
                                                               goal_position,
                                                               goal_radius,
                                                               1)                   
            if len(ik_solutions) == 0:
                return None            
            self.serializer.serialize_ik_solutions([ik_solutions[i] for i in xrange(len(ik_solutions))], 
                                                   problem,
                                                   self.abs_path)
            self.copyToTmp(problem, model_file, environment_file)
        
        log_file_path = self.abs_path + "/../problems/" + str(problem) +"/log.log"
        initial_policy_file_path = self.abs_path + "/../problems/" + str(problem) +"/pol.pol"        
        
        if os.path.exists(log_file_path):
            os.remove(log_file_path)        
        
        """
        Set the number of simulation runs in the config file
        """  
        #self.replace_goal_angles_in_conf(ik_solution, problem)
        self.replace_option_in_config('logPath', log_file_path, problem)
        self.replace_option_in_config("policyPath", initial_policy_file_path, problem)        
        self.replace_option_in_config('historiesPerStep', histories_per_step, problem)        
        self.replace_option_in_config("nRuns", num_runs, problem) 
        self.replace_option_in_config("init_state", init_state, problem)
        self.replace_option_in_config('control_rate', control_rate, problem) 
        self.replace_option_in_config('verbose', verbose, problem, True)
        self.replace_option_in_config('rrt_verb', verbose_rrt, problem, True)
        self.replace_option_in_config('discountFactor', discount, problem)
        self.replace_option_in_config('illegalMovePenalty', illegal_move_penalty, problem)
        self.replace_option_in_config('illegalActionPenalty', illegal_action_penalty, problem)
        self.replace_option_in_config('stepPenalty', step_penalty, problem)
        self.replace_option_in_config('exitReward', exit_reward, problem)
        self.replace_option_in_config('nSteps', num_steps, problem)
        self.replace_option_in_config("maximumDepth", num_steps, problem)
        self.replace_option_in_config('saveParticles', save_particles, problem, True)
        self.replace_option_in_config('stretching_factor', rrt_stretching_factor, problem)
        self.replace_option_in_config('pruneEveryStep', prune_every_step, problem, True)
        self.replace_option_in_config('continuous_collision_check', continuous_collision, problem, True)
        self.replace_option_in_config('check_linear_path', check_linear_path, problem, True)
        self.replace_option_in_config('enforce_constraints', enforce_constraints, problem, True)
        self.replace_option_in_config('planner', planner, problem)
        self.replace_option_in_config('dynamicProblem', dynamic_problem, problem, True)
        self.replace_option_in_config('showViewer', show_viewer, problem, True)
        self.replace_option_in_config("simulation_step_size", simulation_step_size, problem)    
        self.replace_option_in_config('planning_velocity', planning_velocity, problem)
        self.replace_option_in_config('maxObservationDistance', max_observation_distance, problem)
        self.replace_option_in_config('particleReplenishTimeout', particle_replenish_timeout, problem)        
        self.replace_option_in_config('stateSamplingStrategy', state_sampling_strategy, problem)
        self.replace_option_in_config('numSamplesWeightedLazy', num_samples_weighted_lazy, problem)
        self.replace_option_in_config('gravity', gravity, problem)
        self.replace_option_in_config('particlePlotLimit', particle_plot_limit, problem)
        self.replace_option_in_config('numEffectiveParticles', num_effective_particles, problem)
        self.replace_option_in_config('savePolicy', save_policy, problem, True)        
        self.replace_option_in_config('loadInitialPolicy', load_initial_policy, problem, True)
        self.replace_option_in_config("particleFilter", particle_filter, problem)
        self.replace_option_in_config("num_input_steps", num_actions_per_joint, problem)
        self.replace_option_in_config("environment_path", self.abs_path + "/../problems/" + problem + "/environment/" + environment_file, problem)
        self.replace_option_in_config("robot_path", self.abs_path + "/../problems/" + problem + "/model/" + robot_file, problem) 
               
        
        """
        Now we do the simulation for different covariance values
        """
        min_process_covariance = self.config['min_process_covariance']
        max_process_covariance = self.config['max_process_covariance']
        min_observation_covariance = self.config['min_observation_covariance']
        max_observation_covariance = self.config['max_observation_covariance']
        inc_covariance = self.config['inc_covariance']
        
        self.replace_option_in_config('inc_covariance', inc_covariance, problem)
        if inc_covariance  == 'process':
            m_cov = np.linspace(min_process_covariance, 
                                max_process_covariance, 
                                self.config['covariance_steps'])
        elif inc_covariance  == 'observation':
            m_cov = np.linspace(min_observation_covariance, 
                                max_observation_covariance, 
                                self.config['covariance_steps'])              
        
        for cov in m_cov: 
            if inc_covariance == 'process':
                self.replace_option_in_config('process_covariance', cov, problem)
                self.replace_option_in_config('observation_covariance', min_observation_covariance, problem)
            elif inc_covariance == "observation":
                self.replace_option_in_config('process_covariance', min_process_covariance, problem)
                self.replace_option_in_config('observation_covariance', cov, problem)
            self.replace_option_in_config('minParticleCount', particle_count, problem)
            
            if load_initial_policy: 
                """
                Construct an initial policy first by running ./solve
                """               
                self.replace_option_in_config('stepTimeout', initial_planning_time, problem)
                args = self.abs_path + "/../problems/" + problem + "/solve"
                popen = subprocess.Popen(args)
                popen.wait()
                #cmd= "mv " + self.abs_path + "/pol.pol " + self.abs_path + "../problems/" + problem + "/pol.pol"                
                #os.system(cmd)
            self.replace_option_in_config('stepTimeout', planning_time, problem)
            
            """
            Run the simulation and collect the final state 
            """ 
            logging.info("Run simulations with process covariance " + str(cov))
            args = self.abs_path + "/../problems/" + problem + "/simulate"
            popen = subprocess.Popen(args, stdout=1)
            popen.wait()
            
            self.write_mean_num_collisions(log_file_path, num_runs)            
            self.write_rewards(log_file_path)
            
            cmd = "mv " + log_file_path + " " + self.abs_path + "/stats/log_" + str(cov) + ".log"
            stats_file_path = self.abs_path + "/stats/log_" + str(cov) + ".log"            
            os.system(cmd)
            if (os.path.exists(initial_policy_file_path)):
                cmd = "mv " + initial_policy_file_path + " " + self.abs_path + "/stats/init_pol_" + str(cov) + ".pol"
            
            #cmd = "mv " + self.abs_path + "/log.log " + self.abs_path + "/../problems/" + problem + "/log.log" 
            #cmd = "mv log.log ../problems/" + problem + "/log.log"
            #os.system(cmd)
            
            
                       
            avg_path_lengths = [self.read_avg_path_length(stats_file_path, num_runs)]            
            self.serializer.save_list_data(avg_path_lengths, 
                                           path=self.abs_path + "/stats", 
                                           filename="avg_path_lengths_" + alg + ".yaml")
            self.serializer.append_end_sign("avg_path_lengths_" + alg + ".yaml", path=self.abs_path + "/stats")
            
            logging.info("reading mean number of steps per run")
            mean_num_steps_per_run = [self.read_float_data(stats_file_path, "Mean number of steps")]            
            self.serializer.save_list_data(mean_num_steps_per_run,
                                           path=self.abs_path + "/stats",
                                           filename="mean_num_steps_per_run_" + alg + ".yaml")
            self.serializer.append_end_sign("mean_num_steps_per_run_" + alg + ".yaml", path=self.abs_path + "/stats")            
           
            logging.info("reading mean planning times per step...")
            mean_times_per_step = [self.read_float_data(stats_file_path, "Mean time per step")]
            self.serializer.save_list_data(mean_times_per_step, 
                                           path=self.abs_path + "/stats", 
                                           filename="mean_planning_times_per_step_" + alg + ".yaml")
            self.serializer.append_end_sign("mean_planning_times_per_step_" + alg + ".yaml", path=self.abs_path +  "/stats")  
            
            #mean_times_per_step.append(self.read_float_data(problem, "Mean time per step"))
            logging.info("reading mean planning times per run...")
            mean_times_per_run = [self.read_float_data(stats_file_path, "Mean time per run")]
            self.serializer.save_list_data(mean_times_per_run, 
                                           path=self.abs_path + "/stats", 
                                           filename="mean_planning_times_per_run_" + alg + ".yaml")
            self.serializer.append_end_sign("mean_planning_times_per_run_" + alg + ".yaml", path=self.abs_path + "/stats") 
            
            logging.info("reading mean number of sampled histories")
            mean_num_histories_per_step = [self.read_float_data(stats_file_path, "Mean number of histories per step")]
            self.serializer.save_list_data(mean_num_histories_per_step,
                                           path=self.abs_path + "/stats",
                                           filename="mean_num_generated_paths_per_step_" + alg + ".yaml")
            self.serializer.append_end_sign("mean_num_generated_paths_per_step_" + alg + ".yaml", path=self.abs_path + "/stats")                       
            
            mean_num_histories_per_run = [self.read_float_data(stats_file_path, "Mean number of histories per run")]
            self.serializer.save_list_data(mean_num_histories_per_run,
                                           path=self.abs_path + "/stats",
                                           filename="mean_num_generated_paths_per_run_" + alg + ".yaml")
            self.serializer.append_end_sign("mean_num_generated_paths_per_run_" + alg + ".yaml", path=self.abs_path + "/stats")
            
            logging.info("reading number of successful runs")
            num_successful_runs = [self.read_float_data(stats_file_path, "Percentage of successful runs")]
            self.serializer.save_list_data(num_successful_runs,
                                           path=self.abs_path + "/stats",
                                           filename="num_successes_" + alg + ".yaml")
            self.serializer.append_end_sign("num_successes_" + alg + ".yaml", path=self.abs_path + "/stats")
            
            """
            Clear the nohup log
            """
            try:
                cmd = "cat /dev/null > " + self.abs_path + "/nohup.out"
                #cmd = "cat /dev/null > nohup.out"
                os.system(cmd)
            except:
                pass
        stats = dict(m_cov = m_cov.tolist())
        logging.info("Save stats...")
        self.serializer.save_stats(stats, path=self.abs_path + "/stats")
        logging.info("Save end effector paths..." )
        
        cmd = "cp " + self.abs_path + "/" + self.config_file + " " + self.abs_path + "/stats/" 
        #cmd = "cp " + self.abs_path + "/" + self.config_file + " stats/"        
        os.system(cmd)        
        try: 
            os.makedirs(self.abs_path + "/stats/environment")
        except OSError:            
            if not os.path.isdir(self.abs_path + "/stats/environment"):                
                raise 
        try: 
            os.makedirs(self.abs_path + "/stats/model")
        except OSError:            
            if not os.path.isdir(self.abs_path + "/stats/model"):                
                raise
        cmd = "cp " + self.abs_path + "/../problems/" + str(problem) + "/environment/* " + self.abs_path + "/stats/environment/"        
        #cmd = "cp ../problems/" + str(problem) + "/environment/* stats/environment"        
        os.system(cmd)
        cmd = "cp " + self.abs_path + "/../problems/" + str(problem) + "/model/* " + self.abs_path + "/stats/model/"
        #cmd = "cp ../problems/" + str(problem) + "/model/* stats/model"        
        os.system(cmd) 
        print "Done"             
        
if __name__ == "__main__":
    abs_path = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='ABT')
    parser.add_argument("-p", "--problem", help="The problem to be simulated")
    parser.add_argument("-c", "--config", 
                        help="The config yaml file (has to be in the same folder as this script). Default is 'config.yaml",
                        default="config.yaml")
    args = parser.parse_args()
    if args.problem == None:
        print "No problem given!"
    else:    
        if os.path.isdir(abs_path + "/../problems/" + args.problem):    
            SimulationRunner(args.problem, args.config)
        else:
            print "Problem '" + args.problem + "' doesn't exist or has not been compiled yet"
        
        
