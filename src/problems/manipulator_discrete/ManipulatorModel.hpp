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
class ActionMapping;
class StatePool;
class DiscretizedPoint;
} /* namespace solver */

/** A namespace to hold the various classes used for the RockSample POMDP model. */
namespace manipulator {
//class ManipulatorMdpSolver;
class ManipulatorState;
class ManipulatorObservation;


/** The implementation of the Model interface for the RockSample POMDP.
 *
 * See this paper http://arxiv.org/ftp/arxiv/papers/1207/1207.4166.pdf
 * for a description of the RockSample problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class ManipulatorModel : public shared::ModelWithProgramOptions {
  public:
    /** Constructs a new ManipulatorModel instance with the given random number engine, and the
     * given set of configuration options.
     */
    ManipulatorModel(RandomGenerator *randGen, std::unique_ptr<ManipulatorOptions> options);
    ~ManipulatorModel() = default;
    _NO_COPY_OR_MOVE(ManipulatorModel);    


    /* --------------- The model interface proper ----------------- */
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<solver::State> sampleStateUninformed() override;
    virtual bool isTerminal(solver::State const &state) override;    
    virtual bool isValid(solver::State const &state) override;


    /* -------------------- Black box dynamics ---------------------- */    
    virtual std::tuple<std::unique_ptr<shared::ManipulatorState>, 
                       bool, 
                       bool,
                       std::shared_ptr<shared::ManipulatorState>> makeNextState(shared::ManipulatorState const &currentState, 
                                                                                ManipulatorAction const &action);     
    virtual std::pair<JointValues, bool> makeNextJointValues(
            const JointValues &jointValues, 
            const std::vector<double> &jointInputs);
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::State const */*state*/,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const &nextState) override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;
    virtual void updateModel(Model::StepResult &stepResult, 
    		                 std::vector<solver::State const *> &particles) override;


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

    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
            solver::State const *state, solver::HistoricalData const *data) override;
    
    
    
    /** Returns all of the available actions in the RockSample POMDP, in enumerated order. */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::HistoricalData> createRootHistoricalData() override;
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;
    
    virtual double makeReward(shared::ManipulatorState const &state, ManipulatorAction const &action,
                              shared::ManipulatorState const &nextState, bool isLegal, bool collided);
                              
    virtual std::unique_ptr<shared::ManipulatorObservation> makeObservation(ManipulatorAction const &action, 
                                                                    shared::ManipulatorState const &nextState);
    
    virtual std::unique_ptr<solver::ObservationPool> createObservationPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;
    
    boost::shared_ptr<shared::Robot> getRobot() { return robot_; }
    
    double distanceGoal(solver::State const *state);

  private:
    /** The RockSampleOptions instance associated with this model. */
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
    
    int numActions_;
    
    double delta_t_;

    double process_covariance;

    double observation_covariance;
    
    std::vector<double> max_joint_inputs_;
    
    int num_input_steps_;
    
    double goal_radius_;
    
    int seed = 123456789;

    double discount_factor_;

    std::vector<std::shared_ptr<shared::Obstacle> > obstacles_;

    std::vector<std::shared_ptr<shared::Terrain> > terrains_;

    std::vector<std::pair<std::string, std::string> > obstacle_terrain_map;

    shared::PathPlanner path_planner_;    

    utils::Utils utils_;
    
    bool first_update_;
    
    std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution_;
    
    std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution_;
    
    boost::mt19937 generator_;
    
    MatrixXd observation_covariance_matrix_;
    
    MatrixXd observation_covariance_matrix_inverse_;
    
    double normal_dist_term1_;
    
    std::string env_path_;
    
    std::string robot_path_;
    //std::random_device rd_;
    
    MatrixXd build_covariance_matrix_(std::string covariance_type, double &error);
};
} /* namespace manipulator */

#endif /* MANIPULATOR_MODEL_HPP_ */
