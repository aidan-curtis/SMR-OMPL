///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Aidan Curtis & Patrick Han
//////////////////////////////////////

#include "SMR.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>

#include <limits>

ompl::control::SMR::SMR(const SpaceInformationPtr &si) : base::Planner(si, "SMR")
{
	specs_.approximateSolutions = true;
	siC_ = si.get();

	Planner::declareParam<double>("goal_bias", this, &SMR::setGoalBias, &SMR::getGoalBias, "0.:.05:1.");
	Planner::declareParam<bool>("intermediate_states", this, &SMR::setIntermediateStates, &SMR::getIntermediateStates);
}

ompl::control::SMR::~SMR() // destructor
{
	freeMemory();
}

void ompl::control::SMR::setup()
{
	base::Planner::setup();
	if (!nn_)
		nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Node *>(this));
	nn_->setDistanceFunction([this](const Node *a, const Node *b)
							 {
								 return distanceFunction(a, b);
							 });
}

void ompl::control::SMR::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();
	lastGoalNode_ = nullptr;
}

void ompl::control::SMR::freeMemory()
{
	if (nn_)
	{
		std::vector<Node *> nodes;
		nn_->list(nodes);
		for (auto &node : nodes)
		{
			if (node->state)
				si_->freeState(node->state);
			if (node->control)
				siC_->freeControl(node->control);
			delete node;
		}
	}
}


void ompl::control::SMR::GetTransisions(Node *start_state, Control *control, int num_transitions){
	// create an empty list of nodes (R)
	for (int m = 0; m < num_transitions; m += 1){
		ompl::base::State *prop_state = si_->allocState();
		siC_->propagate(start_state->state, control, 1, prop_state);
		auto *new_node = new Node(siC_);
		new_node->state = prop_state;
		Node *neighbor = nn_->nearest(new_node);
		if(control->as<ompl::control::DiscreteControlSpace::ControlType>()->value == 0) {
			start_state->state_control_0.push_back(neighbor);
		} else {
			start_state->state_control_1.push_back(neighbor);
		}
	}

}
void ompl::control::SMR::BuildSMR(){
	int NUM_SAMPLES = 1000;
	int M = 20;
	for (int i = 0; i < NUM_SAMPLES; i+=1){
		// Uniformly sample a state and add it to the data structure
		auto *new_node = new Node(siC_);
		ompl::base::State *new_state = si_->allocState();
		sampler_->sampleUniform(new_state);
		new_node->state = new_state;
		nn_->add(new_node);
	}

	std::vector<Node *> state_list;
	nn_->list(state_list);
	for (int i = 0; i < int(state_list.size()); i+=1) {

			// std::cout<<r2->values[0]<<std::endl;
			// std::cout<<r2->values[1]<<std::endl;
			auto icontrol = state_list[i]->control->as<ompl::control::DiscreteControlSpace::ControlType>(); // cast control to desired type
			icontrol->value = 0;
			GetTransisions(state_list[i], state_list[i]->control, M);

			// for (double t = 0; t < transitions.size(); t+=1) {
				// TODO: What was the point of this again?				
			// }
	}
}



ompl::base::PlannerStatus ompl::control::SMR::solve(const base::PlannerTerminationCondition &ptc)
{
	checkValidity();
	base::Goal *goal = pdef_->getGoal().get();
	auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
	while (const base::State *st = pis_.nextStart())
	{
		auto *node = new Node(siC_);
		si_->copyState(node->state, st);
		siC_->nullControl(node->control);
		nn_->add(node); // add our input motions to the nearest neighbor structure
	}

	if (nn_->size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();
	std::cout<<"Starting to build smr"<<std::endl;
	BuildSMR();
	std::cout<<"Done building smr"<<std::endl;
	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
	return {false, false};
}

void ompl::control::SMR::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Node *> nodes;
	if (nn_)
		nn_->list(nodes);

	double delta = siC_->getPropagationStepSize();

	if (lastGoalNode_)
		data.addGoalVertex(base::PlannerDataVertex(lastGoalNode_->state));

	for (auto m : nodes)
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