
/* Author: John Noonan */

#include "ompl/control/planners/rrt/n1RRT2.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/ConstrainedSpaceInformation.h"
#include <limits>

ompl::control::n1RRT2::n1RRT2(const SpaceInformationPtr &si) : base::Planner(si, "nRRT2-dec")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &n1RRT2::setGoalBias, &n1RRT2::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("wx", this, &n1RRT2::setWx, &n1RRT2::getWx, "0.:1.0:1.");
    Planner::declareParam<double>("wc", this, &n1RRT2::setWc, &n1RRT2::getWc, "0.:0.08:1.");
}

ompl::control::n1RRT2::~n1RRT2()
{
    freeMemory();
}

void ompl::control::n1RRT2::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) 
    { 
        if (!isInit)
        {
            return distanceFunction(a, b);
        }
        else
        {
            double stateDistSqr = pow(distanceFunction(a,b), 2);
            double costDistSqr = pow(a->cost - b->cost, 2);
            
            double dist = sqrt(wx_*stateDistSqr + wc_*costDistSqr);
            return dist;
        } 
    });
}

void ompl::control::n1RRT2::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::n1RRT2::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::n1RRT2::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    double solution_cost = std::numeric_limits<double>::infinity();
    Motion *approxsol = nullptr;
    double approxsol_dist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    
    base::State *xstate = si_->allocState();
    
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        base::StateSpacePtr space = si_->getStateSpace();
        base::ScopedState<> rs(space, rstate);
        rs[3] = 0.1;
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        double *conVals = rctrl->as<RealVectorControlSpace::ControlType>()->values;
        conVals[0] = 0.; 
        if (cd >= siC_->getMinControlDuration())
        {
            /* create a motion */
            auto *motion = new Motion(siC_);
            // motion->state
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;
            motion->cost = nmotion->cost + distanceFunction(nmotion, motion);
            nn_->add(motion);
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            if (solv)
            {
                cmax_ = motion->cost;
                OMPL_INFORM("Initial cmax = %f", cmax_);
                isInit = true;
                break;
            }
        }
    }
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);
    clear();

    setup();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    rmotion = new Motion(siC_);
    rstate = rmotion->state;
    rctrl = rmotion->control;
    xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Sample the cost
        rmotion->cost = rng_.uniformReal(0, cmax_);
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        
        if (cd >= siC_->getMinControlDuration())
        {
            /* create a motion */
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;
            motion->cost = nmotion->cost + distanceFunction(nmotion, motion);
            nn_->add(motion);
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);

            if (solv && motion->cost < solution_cost)
            {
                approxsol_dist = dist;
                solution = motion;
                solution_cost = motion->cost;
                // break;
            }
            if (dist < approxsol_dist)
            {
                approxsol_dist = dist;
                approxsol = motion;
            }
        }
        iter += 1;
    }
    if (solution)
        OMPL_INFORM("Final cost = %f", solution->cost);

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxsol_dist, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}



void ompl::control::n1RRT2::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
