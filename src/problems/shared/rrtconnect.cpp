#include "rrtconnect.hpp"

using std::cout;
using std::endl;

namespace shared {

RRTConnect::RRTConnect(const base::SpaceInformationPtr &si):
	 ompl::geometric::RRTConnect(si){
    
}

ompl::base::PlannerStatus shared::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{    
	checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree      = true;
    bool solved         = false;

    while (ptc == false)
    {
        TreeData &tree      = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not NULL. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion*> mpath1;
                while (solution != NULL)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion*> mpath2;
                while (solution != NULL)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
                    path->append(mpath2[i]->state);

                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                solved = true;
                break;
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

ompl::geometric::RRTConnect::GrowState shared::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    //dstate->as<shared::ManipState>();
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
    // so we check that one first
    bool validMotion = false;
    if (tgi.start) {     	
    	validMotion = si_->checkMotion(nmotion->state, dstate);    	
    }
    else {
    	validMotion = si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);
    }
    
    //bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) : si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (validMotion)
    {
        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);
        if (reach)
            return REACHED;
        else
            return ADVANCED;
    }
    else
        return TRAPPED;
}

}