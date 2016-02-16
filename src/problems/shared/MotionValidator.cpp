#include "MotionValidator.hpp"

using std::cout;
using std::endl;

using namespace fcl;

namespace shared {

MotionValidator::MotionValidator(const ompl::base::SpaceInformationPtr &si,
		                         boost::shared_ptr<shared::Robot> &robot,
                                 bool continuous_collision,
                                 bool dynamics):    
    ompl::base::MotionValidator(si),
    si_(si), 
    robot_(robot),
    continuous_collision_(continuous_collision),
    obstacles_(),
    dim_(si_->getStateSpace()->getDimension())
{	
    if (dynamics) {
    	dim_ = si_->getStateSpace()->getDimension() / 2;
    }
    
}

bool MotionValidator::checkMotion(const std::vector<double> &s1, 
                                  const std::vector<double> &s2, 
                                  const bool &continuous_collision) const {
    if (continuous_collision) {    	
    	return !collidesContinuous(s1, s2); 
    } 
    else {    	
        return !collidesDiscrete(s2);
    }
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {	
    std::vector<double> angles1;
    std::vector<double> angles2;
    const shared::RealVectorStateSpace::StateType *s1_real = s1->as<shared::RealVectorStateSpace::StateType>();
    const shared::RealVectorStateSpace::StateType *s2_real = s2->as<shared::RealVectorStateSpace::StateType>();
    for (unsigned int i = 0; i < dim_; i++) {
        angles1.push_back(s1_real->values[i]);        
        angles2.push_back(s2_real->values[i]);        
    }
    
    //return checkMotion(angles1, angles2, continuous_collision_);
    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_start;
    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal; 
    
    
    
    if (!s1_real->getCollisionObjects()) {
        robot_->createRobotCollisionObjects(angles1, collision_objects_start);
        std::shared_ptr<std::vector<std::shared_ptr<fcl::CollisionObject>>> coll_objects =
        		std::make_shared<std::vector<std::shared_ptr<fcl::CollisionObject>>>(collision_objects_start);       
        s1_real->setCollisionObjects(coll_objects);        
    }
    else {
    	collision_objects_start = *(s1_real->getCollisionObjects().get());    	
    }
    
    robot_->createRobotCollisionObjects(angles2, collision_objects_goal);
    return checkMotion(collision_objects_start,
                       collision_objects_goal,
                       continuous_collision_);
}

bool MotionValidator::checkMotion(std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects_start,
            		              std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects_goal,
            		              const bool &continuous_collision) const {	
    if (continuous_collision) {    	
    	for (size_t i = 0; i < obstacles_.size(); i++) {
            //if (!obstacles_[i]->isTraversable()) {
    		    for (size_t j = 0; j < collision_objects_start.size(); j++) {                	
    		        if (obstacles_[i]->in_collision(collision_objects_start[j], collision_objects_goal[j])) {    		        	
    		            return false;
    		        }
    		    }
    		//}
        }
    	
    	return true;
	}
	
	for (size_t i = 0; i < obstacles_.size(); i++) {
        //if (!obstacles_[i]->getTerrain()->isTraversable()) {        	
		    if (obstacles_[i]->in_collision(collision_objects_goal)) {		    	
		        return false;
		    }
		//}
    }	
    return true; 
	
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, 
                                  const ompl::base::State *s2, 
                                  std::pair< ompl::base::State *, double > &/*lastValid*/) const {	
    return checkMotion(s1, s2);
}

bool MotionValidator::satisfiesConstraints(const std::vector<double> &s1) const {
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(s1[i]);
	}
	std::vector<double> lower_bounds = si_->getStateSpace()->as<shared::RealVectorStateSpace>()->getBounds().low;
	std::vector<double> upper_bounds = si_->getStateSpace()->as<shared::RealVectorStateSpace>()->getBounds().high;	
	for (size_t i = 0; i < dim_; i++) {
		if (s1[i] < lower_bounds[i]) {
			cout << "RETURN FALSE1" << endl;
			sleep(1);
			return false;			
		}
		else if (s1[i] > upper_bounds[i]) {
			cout << "RETURN FALSE2" << endl;
			sleep(1);
			return false;			
		}
	}
	
	return true;
}

bool MotionValidator::isValid(const ompl::base::State *state) const{
	std::vector<double> angles;	
	const shared::RealVectorStateSpace::StateType *state_real = state->as<shared::RealVectorStateSpace::StateType>();
	for (unsigned int i = 0; i < dim_; i++) {
	    angles.push_back(state_real->values[i]);
	}
    
	return isValid(angles);
}

bool MotionValidator::isValid(const std::vector<double> &s1) const {	
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(s1[i]);
	}
	if (collidesDiscrete(joint_angles))
		return false;
	
	return true;
}

bool MotionValidator::collidesDiscrete(const std::vector<double> &state) const{
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(state[i]);
	}
	
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
	robot_->createRobotCollisionObjects(joint_angles, collision_objects);    
	for (size_t i = 0; i < obstacles_.size(); i++) {
	    if (!obstacles_[i]->getTerrain()->isTraversable()) {        	
	        if (obstacles_[i]->in_collision(collision_objects)) {        		
	        	return true;
	        }
	    }
	}    
	return false; 
	
}

bool MotionValidator::collidesContinuous(const std::vector<double> &state1,
            		                     const std::vector<double> &state2) const {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_start;    	
	robot_->createRobotCollisionObjects(state1, collision_objects_start);
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal;
    robot_->createRobotCollisionObjects(state2, collision_objects_goal);
	for (size_t i = 0; i < obstacles_.size(); i++) {
	    if (!obstacles_[i]->isTraversable()) {
	        for (size_t j = 0; j < collision_objects_start.size(); j++) {                	
	            if (obstacles_[i]->in_collision(collision_objects_start[j], collision_objects_goal[j])) {
	                return true;
	            }
	        }
	    }
	}
	
	return false;
}

void MotionValidator::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
    obstacles_.clear();
    for (size_t i = 0; i < obstacles.size(); i++) {       
        obstacles_.push_back(obstacles[i]);
    }    
}

void MotionValidator::setContinuousCollisionCheck(bool continuous_collision_check) {
	continuous_collision_ = continuous_collision_check;
}

}
