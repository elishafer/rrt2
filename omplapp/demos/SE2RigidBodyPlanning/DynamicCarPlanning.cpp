
/* Author: John Noonan */

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/rrt/RRT2.h>
#include <ompl/control/planners/rrt/n1RRT2.h>
#include <ompl/control/planners/rrt/n2RRT2.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/DynamicCarPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void dynamicCarSetup2(app::DynamicCarPlanning &setup)
{
    setup.setEnvironmentMesh("/home/work/code/omplapp/resources/2D/BugTrap_planar_env.dae");
    setup.setRobotMesh("/home/work/code/omplapp/resources/2D/car1_planar_robot.dae");

    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-55);
    bounds.setHigh(55);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE2StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<> start(stateSpace);
    start[0] = 6.0;
    start[1] = 12.0;
    start[2] = 0.0;
    start[3] = 0.3;
    start[4] = 0.0;

    // define goal state
    base::ScopedState<> goal(stateSpace);
    goal[0] = -39.0;
    goal[1] = 0.0; 
    goal[2] = 0.0;
    goal[3] = 0.3;
    goal[4] = 0.0;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, 5);    
}

void dynamicCarSetup3(app::DynamicCarPlanning &setup)
{
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE2StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<> start(stateSpace);
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    start[3] = 0.3;
    start[4] = 0.0;

    // define goal state
    base::ScopedState<> goal(stateSpace);
    goal[0] = 5.0;
    goal[1] = 1.0; 
    goal[2] = 0.0;
    goal[3] = 0.3;
    goal[4] = 0.0;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .5); 
}

void dynamicCarDemo(app::DynamicCarPlanning &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    setup.setPlanner(std::make_shared<control::n1RRT2>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(20))
    {
        control::PathControl &path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
}
void dynamicCarBenchmark(app::DynamicCarPlanning &setup)
{
    tools::Benchmark::Request request(180., 20000., 40);  // runtime (s), memory (MB), run count

    setup.setup();

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<control::RRT2>(setup.getSpaceInformation()));
    // b.addPlanner(std::make_shared<control::n1RRT2>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<control::n2RRT2>(setup.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char ** /*unused*/)
{
    app::DynamicCarPlanning car;
    dynamicCarSetup2(car);

    if (argc > 1)
        dynamicCarBenchmark(car);
    else
        dynamicCarDemo(car);
    return 0;
}
