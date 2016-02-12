/** @file position_history.cpp
 *
 * Contains the implementations for PositionData and PositionDataTextSerializer.
 */
#include "position_history.hpp"

#include <iostream>
#include <sstream>


#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"

namespace manipulator_continuous {
/* ---------------------- PositionData --------------------- */
PositionData::PositionData(ManipulatorModel *model,
        const shared::ManipulatorState &manipulator_state, int old_step) :
        model_(model),
        manipulator_state_(manipulator_state),
        current_step_(old_step + 1) {
}

PositionData::PositionData(ManipulatorModel *model,
        const shared::ManipulatorState &manipulator_state, int old_step, bool /*copy_const*/) :
        model_(model),
        manipulator_state_(manipulator_state),
        current_step_(old_step) {        
}

std::unique_ptr<solver::HistoricalData> PositionData::copy() const {
    return std::make_unique<PositionData>(model_, manipulator_state_, current_step_, true);
}

std::unique_ptr<solver::HistoricalData> PositionData::createChild(
        solver::Action const &action,
        solver::Observation const &/*observation*/) const {
    ManipulatorAction const &rsAction = static_cast<ManipulatorAction const &>(action);
    std::vector<double> jv(rsAction.getJointInputs());
    bool isLegal;
    bool collided;
    std::unique_ptr<shared::ManipulatorState> next_state;    
    std::tie(next_state, isLegal, collided) = model_->makeNextState(manipulator_state_, rsAction);
    if (!isLegal) {
        //debug::show_message("ERROR: An illegal action was taken!?");
    }
    return std::make_unique<PositionData>(model_, *next_state, current_step_);
}


void PositionData::print(std::ostream &os) const {
    os << "JointValues: " << manipulator_state_.getJointValues() << std::endl;
}


int PositionData::getCurrentStep() const {
    return current_step_;
}

const shared::ManipulatorState* PositionData::getManipulatorState() const{
    return &manipulator_state_;
}


/* --------------------- PositionDataTextSerializer -------------------- */
void PositionDataTextSerializer::saveHistoricalData(solver::HistoricalData const *data,
        std::ostream &os) {    
	PositionData const &position_data = static_cast<PositionData const &>(*data);
	std::vector<double> joint_values = position_data.getManipulatorState()->asVector();    
	for (auto &k: joint_values) {
	    os << k << " ";
	}
	os << position_data.getCurrentStep() << " END ";
}

std::unique_ptr<solver::HistoricalData> PositionDataTextSerializer::loadHistoricalData(
        std::istream &is) {    
	std::vector<std::string> strings;
	std::string s;
	int current_step;
	std::vector<double> joint_values;    
	is >> s;
	if (s == "NULL") {
	    return nullptr;
	}
	
	while (s != "END") {
	    strings.push_back(s);
	    is >> s;
	}
	    
	std::istringstream(strings[strings.size() - 1]) >> current_step;
	for (size_t i = 0; i < strings.size() - 1; i++) {
	    double val;
	    std::istringstream(strings[i]) >> val;
	    joint_values.push_back(val);
	}
	    
	JointValues jv(joint_values);
	    
    ManipulatorModel *model = dynamic_cast<ManipulatorModel *>(getModel());
	boost::shared_ptr<shared::Robot> robot = model->getRobot();
	const shared::ManipulatorState st(jv, robot);	
	return std::make_unique<PositionData>(model, st, current_step, true);
}

} /* namespace manipulator_continuous */
