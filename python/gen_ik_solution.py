from libobstacle import *
from serializer import Serializer
from libpath_planner import *
from gen_ik_solution import *
from librobot import v_double, v2_double, v_string
import ik
import time
import logging

class IKSolutionGenerator:
    def __init__(self):
        self.serializer = Serializer()    
    
    def setup(self,
              robot,
              obstacles, 
              max_velocity, 
              delta_t,
              planning_algorithm,
              path_timeout,
              continuous_collision):        
        """
        Generate the obstacles
        """                
        self.robot = robot        
        logging.info("IKSolutionGenerator: Setup")
        self.link_dimensions = v2_double()
        self.robot.getActiveLinkDimensions(self.link_dimensions)             
        self.path_planner = PathPlanner(robot,
                                        delta_t,
                                        continuous_collision,
                                        max_velocity,
                                        1.0,
                                        False,
                                        False,
                                        planning_algorithm)       
        
        self.path_planner.setup()        
        self.path_planner.setObstacles(obstacles)
        self.obstacles = obstacles
        self.path_timeout = path_timeout
        print "setup"
           
        
    def transform_goal(self, goal_position):
        """
        Transform goal position to first joint frame
        """
        active_joints = v_string()
        self.robot.getActiveJoints(active_joints)
        joint_origins = v2_double()
        self.robot.getJointOrigin(active_joints, joint_origins) 
        joint_origin_first_joint = [joint_origins[0][i] for i in xrange(3)]
        goal_position = [goal_position[i] - joint_origin_first_joint[i] for i in xrange(len(goal_position))]
        return goal_position   

    def generate(self, 
                 start_state, 
                 goal_position, 
                 goal_threshold,
                 num_generated_goal_states): 
        """
        Goal position is w.r.t. base frame
        """       
        goal_position = self.transform_goal(goal_position)         
        possible_ik_solutions = ik.get_goal_states(self.robot, 
                                                   goal_position, 
                                                   self.obstacles, 
                                                   num=num_generated_goal_states)            
        solutions = []
        n = 0
        logging.warn("IKSolutionGenerator: " + str(len(possible_ik_solutions)) + " possible ik solutions found")
        for i in xrange(len(possible_ik_solutions)):                   
            logging.info("IKSolutionGenerator: Checking ik solution " + str(i) + " for validity")            
            ik_solution = [possible_ik_solutions[i][k] for k in xrange(len(start_state) / 2)]                        
            ik_solution.extend([0.0 for j in xrange(len(start_state) / 2)])
            goal_states = v2_double()
            goal_state = v_double()
            goal_state[:] = ik_solution
            goal_states[:] = [goal_state]
            
            goal_position_vec = v_double()
            goal_position_vec[:] = goal_position
            self.path_planner.setGoalStates(goal_states, goal_position_vec, goal_threshold)            
            #self.path_planner.set_start_and_goal(start_state, [ik_solution], goal_position, goal_threshold)
            start_state_vec = v_double()
            start_state_vec[:] = start_state                   
            path = self.path_planner.solve(start_state_vec, self.path_timeout)            
            if len(path) != 0:
                logging.warn("IKSolutionGenerator: ik solution " + str(i) + " is a valid ik solution")
                solutions.append(ik_solution)
            else:
                logging.warn("IKSolutionGenerator: Path has length 0")                
            n += 1
        self.path_planner = None        
        if not len(solutions) == 0: 
            print "IKSolutionGenerator: Found " + str(len(solutions)) + " valid goal states"            
            return solutions                  
        else:
            logging.error("IKSoultionGenerator: Couldn't find a valid IK solution. Defined problem seems to be infeasible.")
            self.path_planner = None            
            return []
    
if __name__ == "__main__":
    IKSolutionGenerator()
    