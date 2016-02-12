/** @file ManipulatorModel.hpp
 *
 * Contains RockSampleModel, which implements the core Model interface for the RockSample POMDP.
 */
#ifndef MANIPULATOR_MODEL_HPP_
#define MANIPULATOR_MODEL_HPP_

#include <ios>                          // for ostream
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector
#include <random>

#include "problems/shared/JointValues.hpp" 
#include "problems/shared/Obstacle.hpp"
#include "problems/shared/Terrain.hpp"
#include "problems/shared/utils.hpp"
#include "problems/shared/path_planner.hpp"
#include "problems/shared/ManipulatorTransitionParameters.hpp"
#include "problems/shared/ManipulatorState.hpp"
#include "problems/shared/ManipulatorObservation.hpp"
#include "problems/shared/robot.hpp"
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "problems/shared/mult_normal.hpp"

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State
#include "solver/abstract-problem/ModelChange.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model

#include "ManipulatorOptions.hpp"
#include "ManipulatorAction.hpp"
#include "position_history.hpp"

#include <ompl/base/MotionValidator.h>
#include <boost/random.hpp>

#include "global.hpp"                     // for RandomGenerator

namespace solver {
//class ActionMapping;
class StatePool;
class DiscretizedPoint;
} /* namespace solver */

/** A namespace to hold the various classes used for the RockSample POMDP model. */
namespace manipulator_continuous {
//class ManipulatorMdpSolver;
class ManipulatorModel;
//class ManipulatorState;
//class ManipulatorObservation;

class ContActionPool final: public solver::ContinuousActionPool {
public:
	ContActionPool(const ManipulatorModel& theModel): model(theModel) {};
	virtual ~ContActionPool() = default;
	_NO_COPY_OR_MOVE(ContActionPool);


	/** Returns a container to store actions within a ContinuousActionMap */
	virtual std::unique_ptr<ContinuousActionContainerBase> createActionContainer(BeliefNode *node) const override;

	/** Returns an action construction data object based on a vector of numbers that was provided.
	 *
	 * Here, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 */
	virtual std::unique_ptr<ContinuousActionConstructionDataBase> createActionConstructionData(const double* constructionDataVector, const BeliefNode* belief) const override;

	/** Returns an action based on the Construction Data that was provided.
	 *
	 * In this version, constructionData is a pointer to a data array as it is returned by
	 * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
	 * create new actions based on values it seems fit.
	 *
	 * The default version uses createActionConstructionData first and then creates an action based
	 * on the full construction data. This might be inefficient and an implementation can override
	 * this function for a more direct approach.
	 *
	 * TODO: Check whether this function is actually used or can be removed.
	 */
	virtual std::unique_ptr<Action> createAction(const double* constructionDataVector, const BeliefNode* belief) const override;


	/** Returns an action based on the Construction Data that was provided.
	 *
	 * The default version calls createAction(constructionData.data()) which is probably fine
	 * in a purely continuous case, but probably not in a hybrid case.
	 */
	virtual std::unique_ptr<Action> createAction(const ContinuousActionConstructionDataBase& constructionData) const override;


	/** Returns the initial bounding box for the continuous search.
	 *
	 * For each dimension, the first entry of the pair is the lower bound, the second entry is the upper bound.
	 */
	virtual std::vector<std::pair<double, double>> getInitialBoundingBox(BeliefNode* belief) const override;
	
	virtual void setInitialBoundingBox(std::vector<double> &max_joint_inputs);

	/** Returns a shared pointer to a container containing the construction data for the additional fixed actions in a hybrid action space.
	 *
	 * The result is a shared pointer. Thus, the implementation can decide whether it wants to create the container and pass on ownership or it
	 * can return a reference to an internal vector without having to re-create it every time.
	 *
	 * The default version returns null to indicate there are no fixed actions.
	 */
	virtual std::vector<std::unique_ptr<ContinuousActionConstructionDataBase>> createFixedActions(const BeliefNode* belief) const override;	
	

private:
	const ManipulatorModel& model;
	std::vector<std::pair<double, double>> initial_bounding_box_;
};


/** The implementation of the Model interface for the RockSample POMDP.
 *
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class ManipulatorModel final: public shared::ModelWithProgramOptions {
    typedef ManipulatorModel This;
  public:      
	//typedef ManipulatorState State;
	//typedef ManipulatorAction Action;
	//typedef ManipulatorObservation Observation;
  public:
    /** Constructs a new ManipulatorModel instance with the given random number engine, and the
     * given set of configuration options.
     */
    ManipulatorModel(RandomGenerator *randGen, std::unique_ptr<ManipulatorOptions> options);
    ~ManipulatorModel() = default;
    _NO_COPY_OR_MOVE(ManipulatorModel);    


    /* --------------- The model interface proper ----------------- */
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;    
    virtual std::unique_ptr<solver::State> sampleStateUninformed() override;
    virtual bool isTerminal(solver::State const &state) override;
    virtual bool isValid(solver::State const &state) override;


    /* -------------------- Black box dynamics ---------------------- */    
    virtual std::tuple<std::unique_ptr<shared::ManipulatorState>, bool, bool> makeNextState(
            shared::ManipulatorState const &currentState, 
            ManipulatorAction const &action);
    virtual std::pair<JointValues, bool> makeNextJointValues(
            const JointValues &jointValues, 
            const std::vector<double> &jointInputs);
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::State const */*state*/,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const &nextState) override;
    virtual std::unique_ptr<shared::ManipulatorObservation> makeObservation(ManipulatorAction const &action, 
                                                                            shared::ManipulatorState const &nextState);    
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;
    virtual void updateModel(Model::StepResult &stepResult, 
        		             std::vector<solver::State const *> &particles) override;
    

    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
            solver::State const *state, solver::HistoricalData const *data) override;

    /* ------- Customization of more complex solver functionality  --------- */
    
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;
    
    virtual double makeReward(shared::ManipulatorState const &state, ManipulatorAction const &action,
                              shared::ManipulatorState const &nextState, bool isLegal, bool collided);
                              
    
    
    virtual std::unique_ptr<solver::ObservationPool> createObservationPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;
    
    virtual std::unique_ptr<solver::HistoricalData> createRootHistoricalData() override;


    /* ----------- Non-virtual methods for ManipulatorModel ------------- */       
    size_t getFixedActionResolution() const { return options_->fixedActionResolution; }
    
    boost::shared_ptr<shared::Robot> getRobot() { return robot_; }
    
    double distanceGoal(solver::State const *state);
    
    /* ------------ Methods for handling particle depletion -------------- */
        /** Generates particles for RockSample using a particle filter from the previous belief.
          *
          * For each previous particle, possible next states are calculated based on consistency with
          * the given action and observation. These next states are then added to the output vector
          * in accordance with their probability of having been generated.
          */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;
        
    virtual std::vector<std::unique_ptr<solver::State>> generateParticlesDefault(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles);
        
    virtual std::vector<std::unique_ptr<solver::State>> generateParticlesObservationModel(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles);
    
    /**
     * Calculates the PDF of the moultivariate normal distribution p(y | x)
     */
    double calcPdf(std::vector<double> &state, std::vector<double> &observation);
    
    solver::StateInfo * sampleParticle(const std::vector<solver::StateInfo *> &stateInfos) override;
    
    /**
     * Sample 'num' particles from previousParticles according to their weights
     */
    std::vector<shared::ManipulatorState const *> sampleParticles(
    		std::vector<solver::State const *> const &previousParticles,
			unsigned int num);

  private:    
    /** The ManipulatorOptions instance associated with this model. */
    ManipulatorOptions *options_;
    
    bool dynamic_problem_;
    
    boost::shared_ptr<shared::Robot> robot_;
    
    /** The penalty for an illegal move. */
    double illegalMovePenalty_;
        
    /** The penalty for a illegal action */
    double illegalActionPenalty_;
        
    /** The penalty for making a step */
    double stepPenalty_;
    
    /** The exit reward */
    double exitReward_;
    
    std::vector<double> goalPosition_;
    
    JointValues initValues_;
    
    double delta_t_;
    
    double process_covariance;
        
    double observation_covariance;
    
    std::vector<double> max_joint_inputs_;
    
    double goal_radius_;
    
    int seed = 123456789;
    
    double discount_factor_;
    
    std::vector<std::shared_ptr<shared::Obstacle> > obstacles_;

    std::vector<std::shared_ptr<shared::Terrain> > terrains_;

    std::vector<std::pair<std::string, std::string> > obstacle_terrain_map;
    
    shared::PathPlanner path_planner_;

    utils::Utils utils_;
    
    std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution_;
        
    std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution_;
        
    boost::mt19937 generator_;
    
    MatrixXd observation_covariance_matrix_;
        
    MatrixXd observation_covariance_matrix_inverse_;
        
    double normal_dist_term1_;
};
} /* namespace manipulator_continuous */

#endif /* MANIPULATOR_MODEL_HPP_ */
