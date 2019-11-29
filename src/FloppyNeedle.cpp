///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
// The collision checker produced in project 2
#include "CollisionChecking.h"
#include <random>
#include <fstream>
#include <cmath>
#include <math.h>
#include "SMR.h"

using namespace std;

 class FloppyNeedleStatePropagator : public ompl::control::StatePropagator
 {
 public:
 
	FloppyNeedleStatePropagator(ompl::control::SpaceInformation *si) : ompl::control::StatePropagator(si)
	{
	}

	void propagate(const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result) const override
	{

		double r = 2.5; // add gaussian noise later
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::normal_distribution<double> r_distribution(0, 0.5);
		std::normal_distribution<double> delta_distribution(0.5, 0.1);
		std::default_random_engine generator(seed);

		double d_samp = delta_distribution(generator);
		double r_samp = r_distribution(generator);
		double new_duration = duration+d_samp;
		r += r_samp;
		// cout<< r_samp<<", "<<d_samp<<endl;

		auto compound_state = state->as<ompl::base::CompoundState>();

		const ompl::base::RealVectorStateSpace::StateType* r2;
		r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

		const ompl::base::SO2StateSpace::StateType* so2;
		so2 = compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

		const ompl::base::DiscreteStateSpace::StateType* d;
		d = compound_state->as<ompl::base::DiscreteStateSpace::StateType>(2);

		auto control_data = control->as<ompl::control::DiscreteControlSpace::ControlType>();

		// auto control_data = control->as<ompl::control::CompoundControlSpace::ControlType>();
		// auto control_data_d = control_data->as<ompl::control::DiscreteControlSpace::ControlType>(2);

		
		double direction = -1;
		if(control_data->value == 0){
		// if(control_data_d->value == 0){
			direction = 1;
		}


		// cout<<"Old Propagate"<<endl;
		// cout<<r2->values[0]<<endl;
		// cout<<r2->values[1]<<endl;
		// cout<<so2->value<<endl;
		// cout<<direction<<endl;

		double new_x = r2->values[0] + r*(cos(so2->value + direction * new_duration + control_data->value*M_PI)-cos(so2->value+ control_data->value*M_PI));
		double new_y = r2->values[1] + r*(sin(so2->value + direction * new_duration+ control_data->value*M_PI)-sin(so2->value+ control_data->value*M_PI));
		double new_theta = so2->value + direction * new_duration;
		double new_d = control_data->value;

		// Open if you want to see all the propagation steps
		// cout<<new_x<<","<<new_y<<","<<new_theta<<","<<new_d<<endl;

		auto result_compound_state = result->as<ompl::base::CompoundState>();

		ompl::base::RealVectorStateSpace::StateType* result_r2;
		result_r2 = result_compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

		ompl::base::SO2StateSpace::StateType* result_so2;
		result_so2 = result_compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

		ompl::base::DiscreteStateSpace::StateType* result_d;
		result_d = result_compound_state->as<ompl::base::DiscreteStateSpace::StateType>(2);

		result_r2->values[0] = new_x;
		result_r2->values[1] = new_y;
		result_so2->value = new_theta;
		result_d->value = new_d;




		// auto debug_result_compound_state = result->as<ompl::base::CompoundState>();

		// ompl::base::RealVectorStateSpace::StateType* debug_result_r2;
		// debug_result_r2 = debug_result_compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

		// ompl::base::SO2StateSpace::StateType* debug_result_so2;
		// debug_result_so2 = debug_result_compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

		// ompl::base::DiscreteStateSpace::StateType* debug_result_d;
		// debug_result_d = debug_result_compound_state->as<ompl::base::DiscreteStateSpace::StateType>(2);

		// cout<<"Other propagate"<<endl;
		// cout<<debug_result_r2->values[0]<<endl;
		// cout<<debug_result_r2->values[1]<<endl;
		// cout<<debug_result_so2->value<<endl;
		// cout<<debug_result_d->value<<endl;

	}
 };
 

class Environment1Goal : public ompl::base::Goal
{
public:
	Environment1Goal(const ompl::control::SpaceInformationPtr &si) : ompl::base::Goal(si)
	{
	}
	virtual bool isSatisfied(const ompl::base::State *st) const
	{
		// perform any operations and return a truth value
		auto compound_state = st->as<ompl::base::CompoundState>();

		const ompl::base::RealVectorStateSpace::StateType* r2;
		r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

		const ompl::base::SO2StateSpace::StateType* so2;
		so2 = compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

		const ompl::base::DiscreteStateSpace::StateType* d;
		d = compound_state->as<ompl::base::DiscreteStateSpace::StateType>(2);

		double x = r2->values[0];
		double y = r2->values[1];
		double goal_distance = sqrt((x-7.5)*(x-7.5)+(y-7.5)*(y-7.5));
		// cout<<goal_distance<<endl;
		return goal_distance < 0.5;
	}
};


void makeBones(std::vector<Rectangle> &  obstacles )
{
	Rectangle obstacle1;
	obstacle1.x = 1;
	obstacle1.y = 7;
	obstacle1.width = 3;
	obstacle1.height = 2;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 3;
	obstacle2.y = 3;
	obstacle2.width = 3;
	obstacle2.height = 3;
	obstacles.push_back(obstacle2);
}

// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::control::SpaceInformation *si, const ompl::base::State *state, std::vector<Rectangle> &obstacles)
{
	// Cast the state to a compound state
	auto compound_state = state->as<ompl::base::CompoundState>();

	const ompl::base::RealVectorStateSpace::StateType* r2;
	r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

	const ompl::base::SO2StateSpace::StateType* so2;
	so2 = compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

	const ompl::base::DiscreteStateSpace::StateType* d;
	d = compound_state->as<ompl::base::DiscreteStateSpace::StateType>(2);

	// Extract x, y
	double x = r2->values[0];
	double y = r2->values[1];
	if(!si->satisfiesBounds(state)){
		return false;
	}
	return isValidPoint(x, y, obstacles);
}


ompl::control::SimpleSetupPtr createFloppy(std::vector<Rectangle> &  obstacles )
{
	// TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
	// STATE SPACE SETUP
	ompl::base::StateSpacePtr r2so2d;

	// Create R^2 component of the State Space
	auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

	// Set bounds on R^2
	ompl::base::RealVectorBounds r2_bounds(2);

	r2_bounds.setLow(0, 0.0);  // x
	r2_bounds.setHigh(0, 10.0); 

	r2_bounds.setLow(1, 0.0);  // y
	r2_bounds.setHigh(1, 10.0); 

	r2->setBounds(r2_bounds);

	auto so2 = std::make_shared<ompl::base::SO2StateSpace>(); // theta

	auto d = std::make_shared<ompl::base::DiscreteStateSpace>(0, 1); // direction

	// Create compound state space
	r2so2d =  r2+so2+d;    

	auto controlSpace = std::make_shared<ompl::control::DiscreteControlSpace>(r2so2d, 0, 1); // Take our state space plus two for control
	// auto controlSpace = std::make_shared<ompl::control::CompoundControlSpace>(r2so2d);

	// Define a simple setup class
	ompl::control::SimpleSetup ss(controlSpace);

	// Return simple setup ptr
	ompl::control::SimpleSetupPtr ssptr = std::make_shared<ompl::control::SimpleSetup>(ss);
	
	// set state validity checking for this space
	ompl::control::SpaceInformation *si = ssptr->getSpaceInformation().get();
	ssptr->setStateValidityChecker([&ssptr, &obstacles](const ompl::base::State *state) { return isValidStatePoint( ssptr->getSpaceInformation().get(), state, obstacles); });

	// set the state propagation routine
	auto propagator(std::make_shared<FloppyNeedleStatePropagator>(si));
	ssptr->setStatePropagator(propagator);

	// ss->setStatePropagator(propagate);
	ssptr->getSpaceInformation()->setPropagationStepSize(0.05);

	// TODO: Do some motion planning for the car
	// choice is what planner to use.
	auto space  = ssptr->getStateSpace();





	// Create start state
	ompl::base::ScopedState<> start(space);
	start[0] = 0; // Initial x
	start[1] = 5; // Initial y
	start[2] = -M_PI/2.0; // Initial th
	// start[3] = 0; // Initial vel
	start->as<ompl::base::CompoundState>()->as<ompl::base::DiscreteStateSpace::StateType>(2)->value = 0;
	// set the start and goal states
	ssptr->addStartState(start);
	auto goal(std::make_shared<Environment1Goal>(ssptr->getSpaceInformation()));
	ssptr->setGoal(goal);

	return ssptr;
}


void planFloppy(ompl::control::SimpleSetupPtr & ss)
{    

	ompl::base::PlannerPtr planner(new ompl::control::SMR(ss->getSpaceInformation()));
	ss->setPlanner(planner);
   
	// attempt to solve the problem within one second of planning time
	ompl::base::PlannerStatus solved = ss->solve(100.0);

	std::cout<<"got past solve()"<<std::endl;

	if (solved)
	{
		std::cout << "Found solution:" << std::endl;
		
		// std::ofstream fout("car_path.txt");
		// print the path to screen
		// ss->getSolutionPath().printAsMatrix(fout);
		// fout.close();
	} 
	else
	{
		std::cout << "No solution found" << std::endl;
	}
	
}



int main(int /* argc */, char ** /* argv */)
{
	std::vector<Rectangle> obstacles;
	makeBones(obstacles);
	ompl::control::SimpleSetupPtr ss = createFloppy(obstacles);
	planFloppy(ss);
	return 0;
}
