/** @file RockSampleAction.hpp
 *
 * Defines the RockSampleAction class, which represents an action for the RockSample problem, and
 * also the ActionType enumeration, which enumerates the different types of actions for RockSample.
 */
#ifndef MANIPULATOR_ACTION_HPP_
#define MANIPULATOR_ACTION_HPP_

#include <cstddef>                      // for size_t
#include <cstdint>

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "f_point_test.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/mappings/actions/continuous_actions.hpp"

using std::cout;
using std::endl;

namespace manipulator_continuous {

class ContActionConstructionData final: public solver::ContinuousActionConstructionDataBase {	
public:

	ContActionConstructionData() = default;
	
	ContActionConstructionData(const double* constructionDataVector)	    
	{
		for (size_t i = 0; i < storage.size(); i++) {
			storage[i] = *(constructionDataVector+i);
		}
		
	}	
	
	ContActionConstructionData(std::vector<double> &joint_inputs) {
		for (size_t i = 0; i < storage.size(); i++) {
			storage[i] = joint_inputs[i];
		}
	}
    
	virtual const double* data() const override { return storage.data(); }

	size_t size() const { return storage.size(); }
	double& operator[](size_t index) { return storage[index]; }
	const double& operator[](size_t index) const { return storage[index]; }
	std::vector<double> asVector() const {
	    return storage;
	}	

public:
	/* Infrastructure for use in a ContinuousActionContainer */
	struct HashEqualOptions{
		HashEqualOptions(const double theTolerance): tolerance(theTolerance) {}

		/* Defines the tolerance used when hashing and comparing elements. */
		const double tolerance;
	};

	size_t hash(const HashEqualOptions& options) const {
		size_t result = 0;
		for (auto i : storage) {
			tapir::hash_combine(result, snapToTolerance(i, options.tolerance));
		}
		return result;
	}
 
	bool equal(const ContActionConstructionData& other, const HashEqualOptions& options) const {
		for (size_t i = 0; i<storage.size(); i++) {
			if ( snapToTolerance(storage[i], options.tolerance) != snapToTolerance(other.storage[i], options.tolerance) ) {
				return false;
			}
		}
		return true;
	}

private:
	static double snapToTolerance(const double value, const double tolerance) { return std::round(value/tolerance)*tolerance; }
	//std::array<double, 3> storage = {{0, 0, 0}};
	std::vector<double> storage = {0, 0, 0};
};

/** A class representing an action in the Manipulator continuous action POMDP
 *
 * This class also implements solver::ContinuousAction so that the solver can use a simplistic
 * continuous action mapping approach (ContinuousActionPool) to store the available actions from
 * each belief node.
 */
class ManipulatorAction : public solver::ContinuousAction {    
  public:
    typedef ContActionConstructionData ConstructionData;    

    ManipulatorAction(std::vector<double> joint_inputs): storage(joint_inputs) {
    }

    ManipulatorAction(const double* constructionDataVector):
    	storage(constructionDataVector) {
    	
    } 
    
    ManipulatorAction(const ConstructionData& data):
    	storage(data) {
    	
    }
    
    virtual ~ManipulatorAction() = default;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &other) const override;
    void print(std::ostream &os) const override;     

    std::vector<double> getJointInputs() const;

    virtual const solver::ContinuousActionConstructionDataBase& getConstructionData() const override {
    	return storage;
    }

    virtual bool equals(Point const &otherPoint) const override;

    virtual std::size_t hash() const override;

  private:
  
    ConstructionData storage;  
    
};
} /* namespace rocksample */

#endif /* MANIPULATOR_ACTION_HPP_ */
