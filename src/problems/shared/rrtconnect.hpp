#ifndef RRT_CONNECT_MANIP_HPP_
#define RRT_CONNECT_MANIP_HPP_

#include "ompl/control/planners/PlannerIncludes.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include "ManipulatorGoalRegion.hpp"
#include "RealVectorStateSpace.hpp"

using namespace ompl;

namespace shared { 
    class RRTConnect: public ompl::geometric::RRTConnect {
    public:
    	RRTConnect(const base::SpaceInformationPtr &si);
    	
    	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
    
    private:
         GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);
         
    };
    
    
}

#endif