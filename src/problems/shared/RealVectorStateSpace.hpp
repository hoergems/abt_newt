#ifndef RRT_MANIP_STATE_SPACE_HPP_
#define RRT_MANIP_STATE_SPACE_HPP_
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/spaces/RealVectorBounds.h"
#include <ompl/base/State.h>
#include "ompl/base/StateSpace.h"
#include "fcl/collision_object.h"

using std::cout;
using std::endl;

namespace shared {

/** \brief State sampler for the R<sup>n</sup> state space */
class RealVectorStateSampler : public ompl::base::RealVectorStateSampler  {
        public:
            /** \brief Constructor */
            RealVectorStateSampler(const ompl::base::StateSpace *space) : ompl::base::RealVectorStateSampler(space)
            {
            }

            virtual void sampleUniform(ompl::base::State *state);
            /** \brief Sample a state such that each component state[i] is
                uniformly sampled from [near[i]-distance, near[i]+distance].
                If this interval exceeds the state space bounds, the
                interval is truncated. */
            virtual void sampleUniformNear(ompl::base::State *state, 
            		                       const ompl::base::State *near, 
            		                       const double distance);
            /** \brief Sample a state such that each component state[i] has
                a Gaussian distribution with mean mean[i] and standard
                deviation stdDev. If the sampled value exceeds the state
                space boundary, it is thresholded to the nearest boundary. */
            virtual void sampleGaussian(ompl::base::State *state, 
            		                    const ompl::base::State *mean, 
            		                    const double stdDev);
        };

class RealVectorStateSpace: public ompl::base::RealVectorStateSpace {
public:
	class StateType : public ompl::base::State  {
	    public:
	        StateType() : ompl::base::State(), collision_objects_() {
	        	
	        }

	        /** \brief Access element i of values.  This does not
	             check whether the index is within bounds */
	        double operator[](unsigned int i) const
	        {
	            return values[i];
	        }

	        /** \brief Access element i of values.  This does not
	             check whether the index is within bounds */
	        double& operator[](unsigned int i)
	        {
	            return values[i];
	        }

	        /** \brief The value of the actual vector in R<sup>n</sup> */
	        double *values;
	        
	        void setCollisionObjects(std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const {
	        	if (collision_objects.size() > 0) {
	        		cout << "setting collision objects for state: " << values[0] << ", " << values[1] << ", " << values[2] << endl;
	        	}
	        	collision_objects_.clear();
	            for (auto &k: collision_objects) {
	        		collision_objects_.push_back(k);
	        	}
	        }
	        	    
	        std::vector<std::shared_ptr<fcl::CollisionObject>> getCollisionObjects() const {
	        	return collision_objects_;
	        	/**for (auto &k: collision_objects_) {
	        		collision_objects.push_back(k);
	        	}*/
	        }
	        
	        void clearCollisionObjects() {
	        	collision_objects_.clear();
	        }
	        
	    private:
	        mutable std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_;
	};
	
	RealVectorStateSpace(unsigned int dim = 0);
	
	void setBounds(const ompl::base::RealVectorBounds &bounds);
	
	void setBounds(double low, double high);
	
	/** \brief Get the bounds for this state space */
	const ompl::base::RealVectorBounds& getBounds() const
	{
	     return bounds_;
	}
	
	void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);
	
	void addDimension(double minBound = 0.0, double maxBound = 0.0);
	
	virtual void enforceBounds(ompl::base::State *state) const;
	
	virtual bool satisfiesBounds(const ompl::base::State *state) const;
	
	virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;
	
	virtual void serialize(void *serialization, const ompl::base::State *state) const;
	
	virtual void deserialize(ompl::base::State *state, const void *serialization) const;
	
	virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
	
	virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;
	
	virtual ompl::base::State* allocState() const;
	
	virtual void freeState(ompl::base::State *state) const;
	
	virtual double* getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const;
	
	virtual void printState(const ompl::base::State *state, std::ostream &out) const;
	
	virtual void registerProjections();
	
	virtual void setup();
	
	virtual double getMaximumExtent() const;
	
	virtual unsigned int getDimension() const;
	
	const std::string& getDimensionName(unsigned int index) const;
	
	int getDimensionIndex(const std::string &name) const;
	
	void setDimensionName(unsigned int index, const std::string &name);
	
	virtual double getMeasure() const;
	
	virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
	
	virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
	
	virtual unsigned int getSerializationLength() const;

protected:
	/** \brief The dimension of the space */
	unsigned int                        dimension_;
	
	/** \brief The bounds of the space (used for sampling) */
	ompl::base::RealVectorBounds                    bounds_;
	
	/** \brief Optional names for individual dimensions */
	std::vector<std::string>            dimensionNames_;

	/** \brief Map from names to index values for dimensions */
	std::map<std::string, unsigned int> dimensionIndex_;
	
private:
	/** \brief The size of a state, in bytes */
	std::size_t                         stateBytes_;
	
};

}

#endif