/** @file RockSampleModel.cpp
 *
 * Contains the implementations for the core functionality of the RockSample POMDP.
 */
#include "ManipulatorModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <initializer_list>
#include <iostream>                     // for cout
#include <queue>
#include <map>
#include <memory>                       // for unique_ptr, default_delete
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <tuple>                        // for tie, tuple
#include <unordered_map>                // for unordered_map<>::value_type, unordered_map
#include <utility>                      // for move, pair, make_pair
#include <vector>                       // for vector, vector<>::reference, __alloc_traits<>::value_type, operator==

#include "global.hpp"                     // for RandomGenerator, make_unique

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator<<

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State

#include "solver/indexing/RTree.hpp"
#include "solver/indexing/FlaggingVisitor.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "ManipulatorOptions.hpp"
#include "ManipulatorAction.hpp"
#include "ManipulatorTextSerializer.hpp"
#include <boost/timer.hpp>
#include <boost/random/random_device.hpp>

using std::cout;
using std::endl;

using namespace shared;

namespace manipulator_continuous {

/*----------------------------------- Helpers --------------------------------*/

template<typename T>
inline T square(const T x) { return x*x; }

std::unique_ptr<ContActionPool::ContinuousActionContainerBase> ContActionPool::createActionContainer(BeliefNode* /*node*/) const {
	return std::make_unique<solver::ContinuousActionContainer<ContActionConstructionData>>(ContActionConstructionData::HashEqualOptions(0.03333333333333));
}

std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase> ContActionPool::createActionConstructionData(const double* constructionDataVector, const BeliefNode* /*belief*/) const {
	return std::make_unique<ContActionConstructionData>(constructionDataVector);
}

std::unique_ptr<solver::Action> ContActionPool::createAction(const double* constructionDataVector, const BeliefNode* /*belief*/) const { 
	return std::make_unique<ManipulatorAction>(constructionDataVector);
}

std::unique_ptr<solver::Action> ContActionPool::createAction(const ContinuousActionConstructionDataBase& constructionData) const {    
	return std::make_unique<ManipulatorAction>(static_cast<const ContActionConstructionData&>(constructionData));
}

std::vector<std::pair<double, double>> ContActionPool::getInitialBoundingBox(BeliefNode* /*belief*/) const {
	return initial_bounding_box_;
}

void ContActionPool::setInitialBoundingBox(std::vector<double> &max_joint_inputs){	
    initial_bounding_box_.clear();
    for (size_t i = 0; i < max_joint_inputs.size(); i++) {
    	initial_bounding_box_.push_back(std::make_pair(-max_joint_inputs[i], max_joint_inputs[i]));
    	//initial_bounding_box_.push_back(std::make_pair(-200, 200));
    }
}

std::vector<std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase>> ContActionPool::createFixedActions(const BeliefNode* /*belief*/) const {

	
	std::vector<std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase>> result;
	return result;
	/**size_t fixedActionResolution = model.getFixedActionResolution();

	std::vector<std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase>> result;
	
	std::vector<std::pair<double, double>> initial_bounding_box = getInitialBoundingBox(nullptr);
	if (fixedActionResolution > 0) {
		const double low = -1;
		const double high = 1;
		const double delta = (high-low) / fixedActionResolution;
		result.reserve(square(fixedActionResolution));
		
	} 

	if (fixedActionResolution > 0) {
		const double low = -1;
		const double high = 1;
		const double delta = (high-low) / fixedActionResolution;

		result.reserve(square(fixedActionResolution));

		for (size_t ix = 0; ix < fixedActionResolution; ix++) {
			for (size_t iy = 0; iy < fixedActionResolution; iy++) {
			    for (size_t iz = 0; iz < fixedActionResolution; iz++) {
				    result.push_back(std::make_unique<ContActionConstructionData>(low + delta * ( ix + 0.5), low + delta * ( iy + 0.5), low + delta * (iz + 0.5)));
				}
			}
		}
	}

	return result;*/

}


ManipulatorModel::ManipulatorModel(RandomGenerator *randGen, std::unique_ptr<ManipulatorOptions> options) :
            shared::ModelWithProgramOptions("Manipulator", randGen, std::move(options)),
            options_(const_cast<ManipulatorOptions *>(static_cast<ManipulatorOptions const *>(getOptions()))),
			dynamic_problem_(options_->dynamic_problem),
			robot_(boost::make_shared<shared::Robot>(options_->robot_path)),
            illegalMovePenalty_(options_->illegalMovePenalty),
            illegalActionPenalty_(options_->illegalActionPenalty),
            stepPenalty_(options_->stepPenalty),
            exitReward_(options_->exitReward),             
            goalPosition_(),
			initValues_({options_->init_angle_0, 
				         options_->init_angle_1, 
				         options_->init_angle_2,
			             options_->init_velocity_0,
			             options_->init_velocity_1,
			             options_->init_velocity_2}),            
            delta_t_(1.0 / options_->control_rate), 
            process_covariance(options_->process_covariance),
            observation_covariance(options_->observation_covariance),
			max_joint_inputs_(),
            goal_radius_(),
            discount_factor_(options_->discountFactor),
            obstacles_(),
            terrains_(),
            obstacle_terrain_map(),
			path_planner_(robot_,
			              delta_t_,
			              options_->continuous_collision_check,
				          options_->planning_velocity,                          
			              options_->enforce_constraints,
			              options_->stretching_factor,                         
			              options_->check_linear_path,                  
			              options_->verbose_rrt,						 
			              options_->planner),
            utils_(),
            process_distribution_(nullptr),			
            observation_distribution_(nullptr),
			observation_covariance_matrix_(),
			observation_covariance_matrix_inverse_(),
			normal_dist_term1_()
{		
	boost::random_device rd;
	generator_ = boost::mt19937(rd());	
	options_->numberOfStateVariables = robot_->getDOF();
	utils_.loadObstaclesXML(options_->env_path, obstacles_);	
	path_planner_.setObstacles(obstacles_);
	std::vector<std::vector<double>> gs = utils_.loadGoalStates();
	std::vector<double> goal_area;
	utils_.loadGoalArea(options_->env_path, goal_area);
	if (goal_area.size() != 4) {
		cout << "Goal area is malformed!!!!!!" << endl;
		sleep(1);
	}
	for (size_t i = 0; i < 3; i++) {
		goalPosition_.push_back(goal_area[i]);
	}
	goal_radius_ = goal_area[4];	
	path_planner_.setGoalStates(gs, goalPosition_, goal_radius_);
	robot_->enforceConstraints(options_->enforce_constraints);
	path_planner_.setup();	
	std::vector<std::string> activeJoints;
	robot_->getActiveJoints(activeJoints);
	std::vector<double> jointTorqueLimits;
	robot_->getJointTorqueLimits(activeJoints, jointTorqueLimits);	
	for (auto &k: jointTorqueLimits) {
	    max_joint_inputs_.push_back(k);	    
	}
	
	// Build the diagonal observation covariance matrix
	observation_covariance_matrix_ = MatrixXd::Identity(2 * robot_->getDOF(), 2 * robot_->getDOF());
	observation_covariance_matrix_ = options_->observation_covariance * observation_covariance_matrix_;
	
	// Pre-calculate parameters for the multivariate normal PDF of the observation distribution
	double det = observation_covariance_matrix_.determinant();
	observation_covariance_matrix_inverse_ = observation_covariance_matrix_.inverse();
	normal_dist_term1_ = 1.0 / sqrt(std::pow(2.0 * M_PI, 2 * robot_->getDOF()) * det);
	
	robot_->setGravityConstant(options_->gravityConstant);
	robot_->setParticlePlotLimit(options_->particle_plot_limit);
	
	process_distribution_ = std::make_shared<shared::EigenMultivariateNormal<double>>(generator_);
    observation_distribution_ = std::make_shared<shared::EigenMultivariateNormal<double>>(generator_);
	Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(robot_->getDOF(), 1);
	Eigen::MatrixXd mean_observation = Eigen::MatrixXd::Zero(2 * robot_->getDOF(), 1);
	Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(robot_->getDOF(), robot_->getDOF());	
	for (size_t i = 0; i < cov.rows(); i++) {
		cov(i, i) = options_->process_covariance;
	}
	
	process_distribution_->setMean(mean);
    process_distribution_->setCovar(cov);	
	observation_distribution_->setMean(mean_observation);	
	observation_distribution_->setCovar(observation_covariance_matrix_);
	
	cout << "Initialized model with process covariance " << process_covariance  << endl;
	cout << "and observation covariance " << observation_covariance << endl;
}

/**
Samples an init state. At the moment the init state is always the same (initValues_)
*/
std::unique_ptr<solver::State> ManipulatorModel::sampleAnInitState() {
	std::unique_ptr<shared::ManipulatorState> up = std::make_unique<shared::ManipulatorState>(initValues_, robot_);
    return std::make_unique<shared::ManipulatorState>(initValues_, robot_);   
}

/**
Sample a state uniformly
*/
std::unique_ptr<solver::State> ManipulatorModel::sampleStateUninformed() {	
	std::vector<double> joint_lower_position_limits;
	std::vector<double> joint_upper_position_limits;
	std::vector<double> velocity_limits;
	std::vector<std::string> active_joints;
	robot_->getActiveJoints(active_joints);
	robot_->getJointLowerPositionLimits(active_joints, joint_lower_position_limits);
	robot_->getJointUpperPositionLimits(active_joints, joint_upper_position_limits);
	robot_->getJointVelocityLimits(active_joints, velocity_limits);	
	JointValues jointValues(initValues_.size());
	jointValues.sampleUninformed(joint_lower_position_limits, joint_upper_position_limits, velocity_limits);    
	return std::make_unique<shared::ManipulatorState>(jointValues, robot_);
}

/**
Check if a state is terminal
*/
bool ManipulatorModel::isTerminal(solver::State const &state) {	
    shared::ManipulatorState const &manipulatorState = static_cast<shared::ManipulatorState const &>(state);
    std::shared_ptr<std::vector<double> > endEffectorPosition = manipulatorState.getEndEffectorPosition();
    double sum = 0.0;
    for (size_t i = 0; i < 3; i++) {
        sum += pow(goalPosition_[i] - endEffectorPosition->at(i), 2);
    }
    
    if (sqrt(sum) < goal_radius_) {        
        return true;
    }
    
    return false;
}

/**
Check if a state is valid
*/
bool ManipulatorModel::isValid(solver::State const &state) {
    shared::ManipulatorState const &manipulatorState = static_cast<shared::ManipulatorState const &>(state);
    bool isValid_ =  manipulatorState.getJointValues().isLegal();
    return isValid_;    
}

/**
Use the generative model to generate (s_i, a_i) => (s_(i+1), o_(i+1), r_(i+1))
*/
solver::Model::StepResult ManipulatorModel::generateStep(solver::State const &state,
        solver::Action const &action) {	
    shared::ManipulatorState const &manipulatorState = static_cast<shared::ManipulatorState const &>(state);
    ManipulatorAction const &manipulatorAction = (static_cast<ManipulatorAction const &>(action));    
    solver::Model::StepResult result;
    result.action = action.copy();    
    bool isLegal;
    bool collided;
    std::unique_ptr<shared::ManipulatorState> nextState;
    std::tie(nextState, isLegal, collided) = makeNextState(manipulatorState, manipulatorAction);
    result.observation = makeObservation(manipulatorAction, *nextState);
    result.reward = makeReward(manipulatorState, manipulatorAction, *nextState, isLegal, collided);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);    
    result.transitionParameters = std::make_unique<shared::ManipulatorTransitionParameters>(collided);
          
    return result;
}

void ManipulatorModel::updateModel(solver::Model::StepResult &stepResult,
		                           std::vector<solver::State const *> &particles) {
	if (options_->show_viewer) {		
		robot_->setupViewer(options_->robot_path, options_->env_path);
		//shared::ManipulatorState const *manipulatorStartState = (static_cast<shared::ManipulatorState const *>(stepResult.nextState));
		shared::ManipulatorState const &manipulatorState = static_cast<shared::ManipulatorState const &>(*(stepResult.nextState));
		std::vector<double> state = manipulatorState.asVector();
		std::vector<double> angles;
		std::vector<double> velo;
		
		for (size_t i = 0; i < state.size() / 2; i++) {
			angles.push_back(state[i]);
		}
		
		for (size_t i = state.size() / 2; i < state.size(); i++) {
			velo.push_back(state[i]);
		}
		
		std::vector<std::vector<double>> plot_particles;
		std::vector<std::vector<double>> particle_colors;
		for (auto &particle: particles) {
			shared::ManipulatorState const *manipulator_state = (static_cast<shared::ManipulatorState const *>(particle));
			std::vector<double> joint_values = manipulator_state->getJointValues().asVector();
			std::vector<double> particle_vals;
			std::vector<double> particle_color({1.0, 1.0, 1.0, 0.4});
			for (size_t i = 0; i < joint_values.size() / 2; i++) {
				particle_vals.push_back(joint_values[i]);
			}
			plot_particles.push_back(particle_vals);
			particle_colors.push_back(particle_color);
		}				
		robot_->updateViewerValues(angles, velo, plot_particles, particle_colors);
	}
}

/**
Make the next state
*/
std::tuple<std::unique_ptr<shared::ManipulatorState>, bool, bool> ManipulatorModel::makeNextState(shared::ManipulatorState const &currentState, 
		                                                                                          ManipulatorAction const &action) 
{   
	JointValues jointValues(initValues_.size());
	bool isLegal;   
	const std::vector<double> inputs(action.getJointInputs());
	    
	//Check if the end-effector traverses an traversable terrain and set the inputDamping
	double inputDamping = 1.0;
	/**for (size_t i = 0; i < obstacles_.size(); i++) {
	   if (obstacles_[i]->in_collision(*(currentState.getEndEffectorPosition()))) {
	       if (obstacles_[i]->getTerrain()->isTraversable()) {           
	           inputDamping = 1.0 / (obstacles_[i]->getTerrain()->getInputDamping());
	           break;
	       } 
	       else {
	           //This should never happen
	           cout << "ERROR: Current state places the manipulator inside a non-traversable obstacle" << endl;
	           inputDamping = 0.0; 
	           break;              
	       }          
	   }
	       
	}*/
	    
	std::tie(jointValues, isLegal) = makeNextJointValues(currentState.getJointValues(), inputs);
	// all inputs 0 => illegal!
	if (inputs[0] == 0.0 && inputs[1] == 0.0 && inputs[2] == 0.0) {
	    isLegal = false;
	}
	std::unique_ptr<shared::ManipulatorState> state_ptr = std::make_unique<shared::ManipulatorState>(jointValues, robot_);
	std::vector<double> currentStateVec(currentState.asVector());
	std::vector<double> nextStateVec(state_ptr->asVector());
		
	std::vector<double> v1Temp;
	std::vector<double> v2Temp;
	for (size_t i = 0; i < initValues_.asVector().size() / 2; i++) {
		v1Temp.push_back(currentStateVec[i]);
		v2Temp.push_back(nextStateVec[i]);
	}
	const std::vector<double> vec1(v1Temp);
	const std::vector<double> vec2(v2Temp);
	    
	// Check if the motion collides with a non-traversable obstacle
	bool continuous_collision = true; 
	bool collided = !static_cast<shared::MotionValidator const &>(*(path_planner_.getMotionValidator())).checkMotion(vec1, 
	                                                                                                                 vec2, 
	                                                                                                                 continuous_collision);      
	if (collided) {    	
	    isLegal = false;
	}
	    
	if (!isLegal) {    	
	    if (collided) {
	    	std::vector<double> newValues(currentState.asVector());    		
	    	for (size_t i = newValues.size() / 2; i < newValues.size(); i++) {
	    		newValues[i] = 0;    			
	    	}
	    	JointValues nextJointValues(newValues);
			auto res = std::make_tuple(std::make_unique<shared::ManipulatorState>(nextJointValues, robot_),
									   isLegal,
									   collided);
			return res;
	    }
	    else {
	    	auto res = std::make_tuple(std::make_unique<shared::ManipulatorState>(jointValues, robot_),
	    	                           isLegal,
	    		                       collided);
	        return res;
	    }	        
	}
	    
	auto res = std::make_tuple(std::make_unique<shared::ManipulatorState>(jointValues, robot_),
	                           isLegal,
	                           collided);
	return res;
}

/**
Make the next joint values
*/
std::pair<JointValues, bool> ManipulatorModel::makeNextJointValues(const JointValues &jointValues, 
                                                                   const std::vector<double> &jointInputs) 
{ 
	std::vector<double> current_state = jointValues.asVector();
	std::vector<double> control_input = jointInputs;
	std::vector<double> next_state;
	std::vector<double> control_error;	
	Eigen::MatrixXd sample(control_input.size(), 1);
	process_distribution_->nextSample(sample);
	for (size_t i = 0; i < control_input.size(); i++) {
		control_error.push_back(sample(i));
	}
	double simulation_step_size = 0.0001;
	bool legal = false;
	if (dynamic_problem_) {
	    legal = robot_->propagate(current_state,    			                  
	    			              control_input,
	    			              control_error,    			                                 
	    			              simulation_step_size,    			                  
	    			              delta_t_,
	    			              next_state);    	
	}
    else {
		legal = robot_->propagate_linear(current_state, 
					                     control_input, 
										 control_error,											  
										 delta_t_, 
									     next_state);
	}
	
	JointValues nextJointValues(next_state); 	
	if (legal) {    	   	
	    return std::make_pair(nextJointValues, true);
	}
	    
	return std::make_pair(nextJointValues, false);
}

/**
Make a reward for the resulting state
*/
double ManipulatorModel::makeReward(shared::ManipulatorState const &/*state*/, ManipulatorAction const &action,
        shared::ManipulatorState const &nextState, bool isLegal, bool collided) {        
        const std::vector<double> inputs(action.getJointInputs());
        
        // All joint inputs are 0 => This is an illegal action, so return the illegalActionPenalty
        if (inputs[0] == 0.0 && inputs[1] == 0.0 && inputs[2] == 0.0)
        {        
            return -illegalActionPenalty_;
        }
        
        if (!isLegal) { 
        	if (collided) {        		
        		return -illegalMovePenalty_;
        	}        	
            return -illegalActionPenalty_;
        }        
        
        if (isTerminal(nextState)) {
            return exitReward_;
        }
        
        double traversalCost = 0.0;
        for (size_t i = 0; i < obstacles_.size(); i++) {
            if (obstacles_[i]->in_collision(*(nextState.getEndEffectorPosition()))) {
               if (obstacles_[i]->getTerrain()->isTraversable()) {                   
                   traversalCost = obstacles_[i]->getTerrain()->getTraversalCost();                                  
                   break;
               }
            }
        }        
        return -stepPenalty_ - traversalCost;
}

/**
Generate an observation for the next state
*/
std::unique_ptr<solver::Observation> ManipulatorModel::generateObservation(
        solver::State const */*state*/, solver::Action const &action,
        solver::TransitionParameters const */*tp*/, solver::State const &nextState) {
    return makeObservation(static_cast<ManipulatorAction const &>(action),
            static_cast<shared::ManipulatorState const &>(nextState));
}

/**
Make an observation for the next state
*/
std::unique_ptr<shared::ManipulatorObservation> ManipulatorModel::makeObservation(ManipulatorAction const &/*action*/, 
                                                                                  shared::ManipulatorState const &nextState) {                                                                                
	return std::make_unique<shared::ManipulatorObservation>(nextState, observation_distribution_);
}

/**
Create the continuous action pool
*/
std::unique_ptr<solver::ActionPool> ManipulatorModel::createActionPool(solver::Solver* /*solver*/) { 
    std::unique_ptr<ContActionPool> ptr = std::make_unique<ContActionPool>(*this);
    ptr->setInitialBoundingBox(max_joint_inputs_); 
    return std::move(ptr);
}

/**
Create the continuous observation pool
*/
std::unique_ptr<solver::ObservationPool> ManipulatorModel::createObservationPool(
        solver::Solver *solver) {    
    std::unique_ptr<solver::ObservationPool> ptr = 
    		std::make_unique<solver::ApproximateObservationPool>(solver, options_->max_observation_distance); 
    
    return ptr;
}

/**
Get the default heuristic value, which is the total reward obtained from a deterministic path
obtained from RRTConnect
*/
double ManipulatorModel::getDefaultHeuristicValue(solver::HistoryEntry const *entry,
        solver::State const *state, solver::HistoricalData const *data) {
	    //return -distanceGoal(state);
	    int current_step = 0;
		if (data) {			
		    const PositionData *position_data = static_cast<PositionData const *>(data);
		    current_step = position_data->getCurrentStep();		    
		}
		else {
			cout << "WARNING: No historical data provided in heuristic function!!!!" << endl;
		}
        
        shared::ManipulatorState const *manipulatorStartState = (static_cast<shared::ManipulatorState const *>(state)); 
        const std::vector<double> start_state(manipulatorStartState->asVector());
        std::vector<std::vector<double> > solution_path;        
        double reward = 0.0;            	           
        solution_path = path_planner_.solve(start_state, 10.0);        
	    if (solution_path.size() == 0) {	        	
		    reward -= std::pow(discount_factor_, current_step) * illegalActionPenalty_;		    
		    return reward;		    
	    }
        /**if (serialize_path_) {
            serialize_path_ = false;
            cout << "Write" << endl;
            sleep(1);
            utils_.serializeStatePath(solution_path);
        }*/
	              
	    double traversalCost;
	    std::vector<double> vec1;
	    std::vector<double> vec2;                
        for (size_t i=1; i<solution_path.size(); i++) {
		    traversalCost = 0.0;
		    vec1.clear();
		    vec2.clear();
		    for (size_t k = 0; k < 3; k++) {
		        vec1.push_back(solution_path[i-1][k]);
		        vec2.push_back(solution_path[i][k]);
		    } 
		    JointValues ja(vec2);
		    shared::ManipulatorState s(ja, robot_);
		        
		    if (isTerminal(s)) {                    
		        reward += std::pow(discount_factor_, current_step + i) * exitReward_;		        
		        break;		    
		    }
		    else {
		        // If the end effector collides with a traversable obstacle, determine the traversal cost.
		        /**for (size_t j = 0; j < obstacles_.size(); j++) {                
		            if (obstacles_[j]->in_collision(*(s.getEndEffectorPosition()))) {
		                if (obstacles_[j]->getTerrain()->isTraversable()) {
		                    //cout << "cost " << traversalCost << endl;
		                    traversalCost = obstacles_[j]->getTerrain()->getTraversalCost();                        
		                    break;
		                }                        
		            }                
		        }*/
                reward -= std::pow(discount_factor_, current_step + i) * (stepPenalty_ + traversalCost);
		    }
		        
        }
        
        if (std::isnan(reward)) {
            cout << "whaaaaaaaaaat " << reward << endl;            
        }
        
        return reward;        
}

double ManipulatorModel::distanceGoal(solver::State const *state) {
	shared::ManipulatorState const *manipulatorState = (static_cast<shared::ManipulatorState const *>(state));
	std::shared_ptr<std::vector<double> > endEffectorPosition = manipulatorState->getEndEffectorPosition();
	
	double sum = 0.0;
	for (size_t i = 0; i < 3; i++) {
	    sum += pow(goalPosition_[i] - endEffectorPosition->at(i), 2);
	}
	
	return sqrt(sum);
}
 
std::unique_ptr<solver::HistoricalData> ManipulatorModel::createRootHistoricalData() {
    const shared::ManipulatorState state(initValues_, robot_);    
    return std::make_unique<PositionData>(this, state, 0, true);
}

std::unique_ptr<solver::Serializer> ManipulatorModel::createSerializer(solver::Solver *solver) {
	unsigned int action_space_dim = robot_->getDOF();
    return std::make_unique<ManipulatorTextSerializer>(solver, action_space_dim);    
}

solver::StateInfo * ManipulatorModel::sampleParticle(const std::vector<solver::StateInfo *> &stateInfos) {
	if (options_->stateSamplingStrategy == "max") {		
		long index = 0;
		double max_weight = 0.0;
		double w = 0.0;
		for (size_t i = 0; i < stateInfos.size(); i++) {
			w = static_cast<shared::ManipulatorState const *>(stateInfos[i]->getState())->getWeight();
			if (w > max_weight) {
				max_weight = w;
				index = i;
			}
		}
		
		return stateInfos[index];
	}
	else if (options_->stateSamplingStrategy == "weighted_lazy") {		
		RandomGenerator *randGen = getRandomGenerator();
		std::vector<double> weights;
		std::vector<long> indices;
		double sum_weights = 0.0;
		for (size_t i = 0; i < options_->numSamplesWeightedLazy; i++) {
			long index = std::uniform_int_distribution<long>(0, stateInfos.size() - 1)(*randGen);
			indices.push_back(index);
			double w = static_cast<shared::ManipulatorState const *>(stateInfos[index]->getState())->getWeight();
			weights.push_back(w);
			sum_weights += w;
		} 
		
		std::uniform_real_distribution<double> uniform_dist(0, sum_weights);
	    double rand_num = uniform_dist(generator_);
	    
	    long index = 0;
	    for (size_t j = 0; j < weights.size(); j++) {			
	    	if (rand_num < weights[j]) {
	    		index = indices[j];
	    		break;
	    	}
	    	rand_num -= weights[j];
	    }
	    		
	    return stateInfos[index];
		
	}
	else if (options_->stateSamplingStrategy == "weighted") {
		double sum_weights = 0.0;
		std::vector<double> weights;
		
		// Collect the weights and normalize them
		for (size_t i = 0; i < stateInfos.size(); i++) {			
			double w = static_cast<shared::ManipulatorState const *>(stateInfos[i]->getState())->getWeight();
			sum_weights += w;
			weights.push_back(w);
		}
		//cout << "len " << stateInfos.size() << endl;
		//cout << "sum weights " << sum_weights << endl;
		
		//for (size_t i = 0; i < weights.size(); i++) {
			//weights[i] /= sum_weights;
		//}
		
		// Sample a particle according to their weight
		std::uniform_real_distribution<double> uniform_dist(0, sum_weights);
		double rand_num = uniform_dist(generator_);
		long index = 0;
		for (size_t j = 0; j < weights.size(); j++) {			
			if (rand_num < weights[j]) {
				index = j;
				break;
			}
			rand_num -= weights[j];
		}
		
		return stateInfos[index];		
	}
	
	// Sample a particle using the default sampling strategy
	RandomGenerator *randGen = getRandomGenerator();
	long index = std::uniform_int_distribution<long>(0, stateInfos.size() - 1)(*randGen);	
	return stateInfos[index];
}

std::vector<shared::ManipulatorState const *> ManipulatorModel::sampleParticles(
		std::vector<solver::State const *> const &previousParticles,
		unsigned int num) {
	
	std::vector<shared::ManipulatorState const *> particle_set;
	while (particle_set.size() < num) {
		double sum_of_weights = 0.0;
		for (size_t j = 0; j < previousParticles.size(); j++) { 
			solver::State const *state = previousParticles[j];
			shared::ManipulatorState const * particle = 
					dynamic_cast<shared::ManipulatorState const *>(state);
			sum_of_weights += particle->getWeight();
		}
		
		std::uniform_real_distribution<double> uniform_dist(0, sum_of_weights);
		double rand_num = uniform_dist(generator_);
		for (size_t j = 0; j < previousParticles.size(); j++) { 
			solver::State const *state = previousParticles[j];
			shared::ManipulatorState const *particle = 
					dynamic_cast<shared::ManipulatorState const *>(state);
			if (rand_num < particle->getWeight()) {
				particle_set.push_back(particle);
				break;
			}
			rand_num -= particle->getWeight();
		}
	}
	return particle_set;
}

std::vector<std::unique_ptr<solver::State>> ManipulatorModel::generateParticles(
        solver::BeliefNode *previousBelief, solver::Action const &action, solver::Observation const &obs, long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
	if (options_->particleFilter == "default") {		
		return generateParticlesDefault(previousBelief, action, obs, nParticles, previousParticles);
	}
	else if (options_->particleFilter == "observationModel") {		
		return generateParticlesObservationModel(previousBelief, action, obs, nParticles, previousParticles);
	}
}

std::vector<std::unique_ptr<solver::State>> ManipulatorModel::generateParticlesObservationModel(
        solver::BeliefNode *previousBelief, solver::Action const &action, solver::Observation const &obs, long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
	double startTime = tapir::clock_ms();
	double endTime = startTime + options_->particle_replenish_timeout * 1000.0;
	std::vector<std::unique_ptr<solver::State>> particles;
	solver::ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(action)->getMapping();    
	solver::BeliefNode *childNode = obsMap->getBelief(obs);	
	shared::ManipulatorObservation const &given_obs = 
			static_cast<shared::ManipulatorObservation const &>(obs);
	std::vector<double> observation_vector(given_obs.getObservedJointValues().asVector());	
	double weight_normalization_constant = 0.0;
	std::vector<double> new_weights;
	
	bool all_zeroes = true;
	int attempts1 = 0;
	while (all_zeroes) {
		attempts1 += 1;
		if (attempts1 > 100) {
			cout << "Max number of attempts to replenenish particles reached3" << endl;
			return particles;
		}
		weight_normalization_constant = 0.0;
		particles.clear();
		int attempts2 = 0;
	    while ((long)particles.size() < options_->minParticleCount) {
	    	attempts2 += 1; 
	    	if (attempts2 > 10000) {
	    		cout << "Max number of attempts to replenenish particles reached2" << endl;
	    	    return particles;
	    	}
	    	if (tapir::clock_ms() >= endTime) {
	    	    cout << "WARNING: Timeout while replenishing particles" << endl;
	    	    return particles;
	    	}	
		
	    	//Sample a particle from the previous particle set proportional to their weights
	    	shared::ManipulatorState const *particle = sampleParticles(previousParticles, 1)[0];		
		
	    	ManipulatorAction const &manipulatorAction = (static_cast<ManipulatorAction const &>(action));
	    	
	    	// Propagate the sampled particle using black box dynamics
	    	StepResult result = generateStep(*particle, action);		
	    	shared::ManipulatorState const *next_state = 
	    			dynamic_cast<shared::ManipulatorState const *>(result.nextState.get());		
	    	std::vector<double> next_state_vec = next_state->getJointValues().asVector();	
		
	    	// Update the resulting particle weight according to the observation model p(y | x)		
	    	double pdf = calcPdf(next_state_vec, observation_vector);	    	
		    if (pdf > 0) {		    	
		 	    all_zeroes = false;
		    }
		    double new_weight = next_state->getWeight() * pdf;		
		    weight_normalization_constant += new_weight;		
		    next_state->setWeight(new_weight);
		    particles.push_back(std::move(result.nextState));		
	    }
	}
	
	// Normalize the weights	
 	for (size_t i = 0; i < particles.size(); i++) {
		shared::ManipulatorState const *particle = dynamic_cast<shared::ManipulatorState const *>(particles[i].get());
		double new_weight = particle->getWeight() / weight_normalization_constant;
		new_weights.push_back(new_weight);
		particle->setWeight(new_weight);
	}
	
	// Calculate number of effective particles
	double quat_sum = 0.0;
	for (auto &w: new_weights) {
		quat_sum += std::pow(w, 2);
	}
	
	double n_eff = 1.0 / quat_sum;
	cout << "Num effective particles: " << n_eff << endl;
	// If number of effective particles < num_effective_particles, perform resampling
	if (n_eff < options_->num_effective_particles) {
		cout << "RESAMPLING" << endl;
		std::vector<solver::State const *> pt;		
		for (size_t i = 0; i < particles.size(); i++) {
			shared::ManipulatorState const *ms = dynamic_cast<shared::ManipulatorState const *>(particles[i].get());
			pt.push_back(ms);
		}
		
		// Sample N particles from the current set proportional to their weights
		std::vector<shared::ManipulatorState const *> particles_temp = sampleParticles(pt, options_->minParticleCount);
		
		// Set their weights to 1 / N
		double w = 1.0 / options_->minParticleCount;
		for (size_t i = 0; i < particles_temp.size(); i++) {			
			particles_temp[i]->setWeight(w);
		}
		
		std::vector<std::unique_ptr<solver::State>> returned_particles;
		for (size_t i = 0; i < particles_temp.size(); i++) {
			returned_particles.push_back(particles_temp[i]->copy());
		}
		
		return returned_particles;
	}
	
	return particles;
}

std::vector<std::unique_ptr<solver::State>> ManipulatorModel::generateParticlesDefault(
        solver::BeliefNode *previousBelief, solver::Action const &action, solver::Observation const &obs, long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
    double startTime = tapir::clock_ms();
    double endTime = startTime + 100000.0;
    std::vector<std::unique_ptr<solver::State>> particles;
    solver::ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(action)->getMapping();    
    solver::BeliefNode *childNode = obsMap->getBelief(obs);    
    // Use rejection sampling to generate the required number of particles.
    while ((long)particles.size() < nParticles) {
        if (tapir::clock_ms() >= endTime) {
            cout << "WARNING: Timeout while replenishing particles" << endl;
            return particles;
        }
        // Sample a random particle.
        if (previousParticles.size() == 0) {
            cout << "Previous particle set is empty!!!!!" << endl;
        }
        long index = std::uniform_int_distribution<long>(0,
                previousParticles.size() - 1)(*getRandomGenerator());
        solver::State const *state = previousParticles[index];
        shared::ManipulatorState const *man_state = dynamic_cast<shared::ManipulatorState const *>(state);
        
        ManipulatorAction const &manipulatorAction = (static_cast<ManipulatorAction const &>(action));
        // Now generate a step in the model, and compare the observation to the actual observation.
        // Note that this comparison is done implicitly via the observation mapping, to ensure
        // that approximate observations are treated cleanly.
        StepResult result = generateStep(*state, action);	
        if (obsMap->getBelief(*result.observation) == childNode) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}

double ManipulatorModel::calcPdf(std::vector<double> &state, std::vector<double> &observation) {
	Eigen::VectorXd mu(state.size());
	Eigen::VectorXd x(state.size());
	for (size_t i = 0; i < state.size(); i++) {
		mu(i) = state[i];
		x(i) = observation[i];
	}		
	double term2 = exp((-1.0 / 2.0) * (x - mu).transpose() * observation_covariance_matrix_inverse_ * (x - mu));
	double pdf = normal_dist_term1_ * term2;		
	return pdf;
	/**double dist = 0.0;
	for (size_t i = 0; i < state.size(); i++) {
		dist += std::pow(state[i] - observation[i], 2);
	}
	if (dist > M_PI / 2.0) {
		return 0.0;
	}	
	return 1.0 / sqrt(dist);*/
}

} /* namespace manipulator_continuous */
