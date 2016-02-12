/** @file HistoryEntry.cpp
 *
 * Contains the implementation of the HistoryEntry class.
 */
#include "solver/HistoryEntry.hpp"

#include <iostream>

#include "solver/BeliefNode.hpp"
#include "solver/changes/ChangeFlags.hpp"              // for ChangeFlags
#include "solver/StateInfo.hpp"                // for StateInfo

using std::cout;
using std::endl;

namespace solver {
class HistorySequence;

HistoryEntry::HistoryEntry() :
    HistoryEntry(nullptr, 0) {
}

HistoryEntry::HistoryEntry(HistorySequence* owningSequence, HistoryEntry::IdType entryId) :
    owningSequence_(owningSequence),
    associatedBeliefNode_(nullptr),
    stateInfo_(nullptr),
    action_(nullptr),
    transitionParameters_(nullptr),
    observation_(nullptr),
    immediateReward_(0),
    entryId_(entryId),
    changeFlags_(ChangeFlags::UNCHANGED) {
}

HistoryEntry::~HistoryEntry() {
}

/* ----------------- Simple getters ------------------- */
HistoryEntry::IdType HistoryEntry::getId() const {
    return entryId_;
}
double HistoryEntry::getImmediateReward() const {
    return immediateReward_;
}
State const *HistoryEntry::getState() const {
    return stateInfo_->getState();
}
StateInfo const *HistoryEntry::getStateInfo() const {
    return stateInfo_;
}
Action const *HistoryEntry::getAction() const{
    return action_.get();
}
Observation const *HistoryEntry::getObservation() const {
    return observation_.get();
}
TransitionParameters const *HistoryEntry::getTransitionParameters() const {
    return transitionParameters_.get();
}
BeliefNode *HistoryEntry::getAssociatedBeliefNode() const {
    return associatedBeliefNode_;
}


/* ============================ PRIVATE ============================ */


/* -------------- Registration methods ---------------- */
void HistoryEntry::registerNode(BeliefNode *node, bool addParticle) {
    if (associatedBeliefNode_ == node) {
        return;
    }
    if (associatedBeliefNode_ != nullptr) {
        associatedBeliefNode_->removeParticle(this);
        associatedBeliefNode_ = nullptr;
    }
    if (node != nullptr) {
        associatedBeliefNode_ = node;
        if (addParticle) {
            associatedBeliefNode_->addParticle(this);
        }
    }
}
void HistoryEntry::registerState(StateInfo *info) {
    if (stateInfo_ == info) {
        return;
    }
    if (stateInfo_ != nullptr) {
        stateInfo_->removeHistoryEntry(this);
        stateInfo_ = nullptr;
    }
    if (info != nullptr) {
        stateInfo_ = info;
        stateInfo_->addHistoryEntry(this);
    }
}

/* ----------------- Change flagging ------------------- */
void HistoryEntry::resetChangeFlags() {
    changeFlags_ = ChangeFlags::UNCHANGED;
}
void HistoryEntry::setChangeFlags(ChangeFlags flags) {
    changeFlags_ |= flags;
}
} /* namespace solver */
