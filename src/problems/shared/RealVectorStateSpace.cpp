#include "RealVectorStateSpace.hpp"
#include "ompl/util/Exception.h"
#include "ompl/base/spaces/RealVectorStateProjections.h"

using std::cout;
using std::endl;

namespace shared {

void shared::RealVectorStateSampler::sampleUniform(ompl::base::State *state)
{
    const unsigned int dim = space_->getDimension();
    const ompl::base::RealVectorBounds &bounds = static_cast<const shared::RealVectorStateSpace*>(space_)->getBounds();

    shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    for (unsigned int i = 0 ; i < dim ; ++i) {
        rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
    }
    rstate->clearCollisionObjects();
    cout << "COLLISION OBJECTS SIZE: " << rstate->getCollisionObjects().size() << endl;
}

void shared::RealVectorStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
{
    const unsigned int dim = space_->getDimension();
    const ompl::base::RealVectorBounds &bounds = static_cast<const shared::RealVectorStateSpace*>(space_)->getBounds();

    shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    const shared::RealVectorStateSpace::StateType *rnear = static_cast<const shared::RealVectorStateSpace::StateType*>(near);
    for (unsigned int i = 0 ; i < dim ; ++i)
        rstate->values[i] =
            rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance),
                             std::min(bounds.high[i], rnear->values[i] + distance));
}

void shared::RealVectorStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
{
    const unsigned int dim = space_->getDimension();
    const ompl::base::RealVectorBounds &bounds = static_cast<const shared::RealVectorStateSpace*>(space_)->getBounds();

    shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    const shared::RealVectorStateSpace::StateType *rmean = static_cast<const shared::RealVectorStateSpace::StateType*>(mean);
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
        double v = rng_.gaussian(rmean->values[i], stdDev);
        if (v < bounds.low[i])
            v = bounds.low[i];
        else
            if (v > bounds.high[i])
                v = bounds.high[i];
        rstate->values[i] = v;
    }
}

shared::RealVectorStateSpace::RealVectorStateSpace(unsigned int dim):
	ompl::base::RealVectorStateSpace(dim),
	dimension_(dim), 
	bounds_(dim),  
	stateBytes_(dim * sizeof(double)){
	type_ = 10;
	setName("RealVector" + getName());
	dimensionNames_.resize(dim, "");
}

void shared::RealVectorStateSpace::setBounds(const ompl::base::RealVectorBounds &bounds)
{
	ompl::base::RealVectorStateSpace::setBounds(bounds);	
	bounds.check();
    if (bounds.low.size() != dimension_)
        throw ompl::Exception("Bounds do not match dimension of state space: expected dimension " +
                              boost::lexical_cast<std::string>(dimension_) + " but got dimension " +
                              boost::lexical_cast<std::string>(bounds.low.size()));
    
    bounds_ = bounds;
}

void shared::RealVectorStateSpace::setBounds(double low, double high)
{
    ompl::base::RealVectorStateSpace::setBounds(low, high);
	ompl::base::RealVectorBounds bounds(dimension_);
    bounds.setLow(low);
    bounds.setHigh(high);
    setBounds(bounds);
}


void shared::RealVectorStateSpace::enforceBounds(ompl::base::State *state) const
{
	shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        if (rstate->values[i] > bounds_.high[i])
            rstate->values[i] = bounds_.high[i];
        else
            if (rstate->values[i] < bounds_.low[i])
                rstate->values[i] = bounds_.low[i];
    }
}

bool shared::RealVectorStateSpace::satisfiesBounds(const ompl::base::State *state) const
{
    const shared::RealVectorStateSpace::StateType *rstate = static_cast<const shared::RealVectorStateSpace::StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i) {
    	//cout << "i " << i << endl;
    	
        if (rstate->values[i] - std::numeric_limits<double>::epsilon() > bounds_.high[i] ||
            rstate->values[i] + std::numeric_limits<double>::epsilon() < bounds_.low[i]) {        	
            return false;
        }
    }
    return true;
}

void shared::RealVectorStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{    
	memcpy(static_cast<shared::RealVectorStateSpace::StateType*>(destination)->values,
           static_cast<const shared::RealVectorStateSpace::StateType*>(source)->values, stateBytes_);
    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects(
    		static_cast<const shared::RealVectorStateSpace::StateType*>(source)->getCollisionObjects());
    //static_cast<const shared::RealVectorStateSpace::StateType*>(source)->getCollisionObjects(collision_objects);
    cout << "copy (" << static_cast<const shared::RealVectorStateSpace::StateType*>(source)->values[0] << ", "
    		         << static_cast<const shared::RealVectorStateSpace::StateType*>(source)->values[1] << ", "
    		         << static_cast<const shared::RealVectorStateSpace::StateType*>(source)->values[2] << ") to ("
    		         << static_cast<const shared::RealVectorStateSpace::StateType*>(destination)->values[0] << ", "
    		         << static_cast<const shared::RealVectorStateSpace::StateType*>(destination)->values[1] << ", "
    		         << static_cast<const shared::RealVectorStateSpace::StateType*>(destination)->values[2] << ")" << endl;
    static_cast<shared::RealVectorStateSpace::StateType*>(destination)->setCollisionObjects(collision_objects);
    
}

void shared::RealVectorStateSpace::serialize(void *serialization, const ompl::base::State *state) const
{
    memcpy(serialization, state->as<shared::RealVectorStateSpace::StateType>()->values, stateBytes_);
}

void shared::RealVectorStateSpace::deserialize(ompl::base::State *state, const void *serialization) const
{
    memcpy(state->as<shared::RealVectorStateSpace::StateType>()->values, serialization, stateBytes_);
}

unsigned int shared::RealVectorStateSpace::getSerializationLength() const
{
    return stateBytes_;
}

double shared::RealVectorStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const shared::RealVectorStateSpace::StateType*>(state1)->values;
    const double *s2 = static_cast<const shared::RealVectorStateSpace::StateType*>(state2)->values;

    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        double diff = (*s1++) - (*s2++);
        dist += diff * diff;
    }
    return sqrt(dist);
}

bool shared::RealVectorStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    const double *s1 = static_cast<const shared::RealVectorStateSpace::StateType*>(state1)->values;
    const double *s2 = static_cast<const shared::RealVectorStateSpace::StateType*>(state2)->values;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        double diff = (*s1++) - (*s2++);
        if (fabs(diff) > std::numeric_limits<double>::epsilon() * 2.0)
            return false;
    }
    return true;
}

ompl::base::State* shared::RealVectorStateSpace::allocState() const
{
	shared::RealVectorStateSpace::StateType *rstate = new shared::RealVectorStateSpace::StateType();
    rstate->values = new double[dimension_];
    return rstate;
}

void shared::RealVectorStateSpace::freeState(ompl::base::State *state) const
{
	shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    delete[] rstate->values;
    delete rstate;
}

double* shared::RealVectorStateSpace::getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const
{
    return index < dimension_ ? static_cast<shared::RealVectorStateSpace::StateType*>(state)->values + index : NULL;
}

void shared::RealVectorStateSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
    out << "RealVectorState [";
    if (state)
    {
        const shared::RealVectorStateSpace::StateType *rstate = static_cast<const shared::RealVectorStateSpace::StateType*>(state);
        for (unsigned int i = 0 ; i < dimension_ ; ++i)
        {
            out << rstate->values[i];
            if (i + 1 < dimension_)
                out << ' ';
        }
    }
    else
        out << "NULL" << std::endl;
    out << ']' << std::endl;
}

void shared::RealVectorStateSpace::setup()
{    
	bounds_.check();	
    ompl::base::StateSpace::setup();
}

double shared::RealVectorStateSpace::getMaximumExtent() const
{
    double e = 0.0;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        double d = bounds_.high[i] - bounds_.low[i];
        e += d*d;
    }
    return sqrt(e);
}

void shared::RealVectorStateSpace::addDimension(const std::string &name, double minBound, double maxBound)
{
    addDimension(minBound, maxBound);
    setDimensionName(dimension_ - 1, name);
}

void shared::RealVectorStateSpace::addDimension(double minBound, double maxBound)
{
    dimension_++;
    stateBytes_ = dimension_ * sizeof(double);
    bounds_.low.push_back(minBound);
    bounds_.high.push_back(maxBound);
    dimensionNames_.resize(dimension_, "");
}

unsigned int shared::RealVectorStateSpace::getDimension() const
{
    return dimension_;
}

const std::string& shared::RealVectorStateSpace::getDimensionName(unsigned int index) const
{
    if (index < dimensionNames_.size())
        return dimensionNames_[index];
    throw ompl::Exception("Index out of bounds");
}

int shared::RealVectorStateSpace::getDimensionIndex(const std::string &name) const
{
    std::map<std::string, unsigned int>::const_iterator it = dimensionIndex_.find(name);
    return it != dimensionIndex_.end() ? (int)it->second : -1;
}

void shared::RealVectorStateSpace::setDimensionName(unsigned int index, const std::string &name)
{
    if (index < dimensionNames_.size())
    {
        dimensionNames_[index] = name;
        dimensionIndex_[name] = index;
    }
    else
        throw ompl::Exception("Cannot set dimension name. Index out of bounds");
}

double shared::RealVectorStateSpace::getMeasure() const
{
    double m = 1.0;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        m *= bounds_.high[i] - bounds_.low[i];
    }
    return m;
}

void shared::RealVectorStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
    const shared::RealVectorStateSpace::StateType *rfrom = static_cast<const shared::RealVectorStateSpace::StateType*>(from);
    const shared::RealVectorStateSpace::StateType *rto = static_cast<const shared::RealVectorStateSpace::StateType*>(to);
    const shared::RealVectorStateSpace::StateType *rstate = static_cast<shared::RealVectorStateSpace::StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
        rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}

ompl::base::StateSamplerPtr shared::RealVectorStateSpace::allocDefaultStateSampler() const
{
    return ompl::base::StateSamplerPtr(new shared::RealVectorStateSampler(this));
}

void shared::RealVectorStateSpace::registerProjections()
{
    // compute a default random projection	
    if (dimension_ > 0)
    {
        if (dimension_ > 2)
        {
            int p = std::max(2, (int)ceil(log((double)dimension_)));
            registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ompl::base::RealVectorRandomLinearProjectionEvaluator(this, p)));
        }
        else
            registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ompl::base::RealVectorIdentityProjectionEvaluator(this)));
    }
}

}